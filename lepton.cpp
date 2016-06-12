/******************************************************************************
 * Copyright (c) 2016 Charles R. Allen
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 ******************************************************************************
 * lepton.cpp
 *
 *  Created on: Mar 7, 2016
 *      Author: Charles Allen
 */

#include <fcntl.h>
#include <error.h>
#include <errno.h>
#include <inttypes.h>
#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include <mraa.hpp>
#include <string>
#include <cstdlib>
#include <linux/i2c-dev.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netinet/ip.h>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <upm/st7735.hpp>

// Something doesn't translate properly to VLC
//#define USE_16BIT_GRAY_RAW

// Attempt to auto-scale the data
#define AUTO_SCALE

using namespace cv;
using namespace std;

static const char *LEPTON_DEV_NAME = "/dev/lepton";
static const char *ST7735_DEV_NAME = "/dev/st7735";
void sig_handler(int signum);
static sig_atomic_t volatile isrunning = 1;
uint32_t frame_counter = 0;
#define exitIfMRAAError(x) exitIfMRAAError_internal(x, __FILE__, __LINE__)

static void exitIfMRAAError_internal(int result, const char *filename,
		unsigned int linenum) {
	if (__builtin_expect(result != MRAA_SUCCESS, 0)) {
		const char *errMsg = NULL;
		switch (result) {
		case MRAA_ERROR_FEATURE_NOT_IMPLEMENTED:
			errMsg = "MRAA_ERROR_FEATURE_NOT_IMPLEMENTED";
			break;
		case MRAA_ERROR_FEATURE_NOT_SUPPORTED:
			errMsg = "MRAA_ERROR_FEATURE_NOT_SUPPORTED";
			break;
		case MRAA_ERROR_INVALID_VERBOSITY_LEVEL:
			errMsg = "MRAA_ERROR_INVALID_VERBOSITY_LEVEL";
			break;
		case MRAA_ERROR_INVALID_PARAMETER:
			errMsg = "MRAA_ERROR_INVALID_PARAMETER";
			break;
		case MRAA_ERROR_INVALID_HANDLE:
			errMsg = "MRAA_ERROR_INVALID_HANDLE";
			break;
		case MRAA_ERROR_NO_RESOURCES:
			errMsg = "MRAA_ERROR_NO_RESOURCES";
			break;
		case MRAA_ERROR_INVALID_RESOURCE:
			errMsg = "MRAA_ERROR_INVALID_RESOURCE";
			break;
		case MRAA_ERROR_INVALID_QUEUE_TYPE:
			errMsg = "MRAA_ERROR_INVALID_QUEUE_TYPE";
			break;
		case MRAA_ERROR_NO_DATA_AVAILABLE:
			errMsg = "MRAA_ERROR_NO_DATA_AVAILABLE";
			break;
		case MRAA_ERROR_INVALID_PLATFORM:
			errMsg = "MRAA_ERROR_INVALID_PLATFORM";
			break;
		case MRAA_ERROR_PLATFORM_NOT_INITIALISED:
			errMsg = "MRAA_ERROR_PLATFORM_NOT_INITIALISED";
			break;
		case MRAA_ERROR_UNSPECIFIED:
			errMsg = "MRAA_ERROR_UNSPECIFIED";
			break;
		default:
			errMsg = "Completely unknown error in MRAA";
		}
		error_at_line(result, errno, filename, linenum, "Error %d in MRAA: %s",
				(int) result, errMsg);
	}
}

/**
 static void exitIfMRAAError_internal(mraa::Result result, const char *filename,
 unsigned int linenum) {
 exitIfMRAAError_internal((int) result, filename, linenum);
 }
 static void exitIfMRAAError_internal(mraa_result_t result, const char *filename,
 unsigned int linenum) {
 exitIfMRAAError_internal((int) result, filename, linenum);
 }
 **/

static int captureImage(uint16_t *image, int fd) {
	const int line_len = 164;
	int result, i = 0, retval = 0;
	uint8_t buff[line_len * 60];
	while (i < 60 && isrunning) {
		uint8_t *line = &buff[i * line_len];
		uint16_t *line16 = (uint16_t *) line;
		memset(line, 0, line_len);
		result = read(fd, line, line_len);
		if (__builtin_expect(result == -1, 0)) {
			error(0, errno, "Error reading from [%s]", LEPTON_DEV_NAME);
			break;
		} else if (__builtin_expect(result != line_len, 0)) {
			error(0, errno, "Size did not match, read %d, expected %d",
					(int) result, line_len);
			break;
		}
		line[0] &= 0x0F;
		if (__builtin_expect(line[0] != 0x0F, 0)) {
			uint16_t lineNum = ntohs(line16[0]);
			if (__builtin_expect(i != lineNum, 0)) {
				printf("Unexpected line. Expected %d found %d\n", (int) i,
						(int) lineNum);
				break;
			}
			++i;
		} else {
			// Each line is 1/27/60 of a second's worth of data, which is ~617us. We sleep for 1/12th of this time between tries for a new line.
			usleep(100);
		}
	}
	retval = i - 60;
	if (__builtin_expect(retval, 0)) {
		return retval;
	}
	for (i = 0; i < 60; ++i) {
		uint8_t *line = &buff[i * line_len];
		memcpy(&image[i * 80], &line[4], 160);
	}

	return retval;
}

#define BROKEN_COLUMN 28

// image is still BigEndian when it gets here
static int printImg(uint16_t *image, uint16_t *out_image) {
	uint16_t min_v = (uint16_t) -1, max_v = 0;
#ifdef AUTO_SCALE
	for (int i = 0; i < 60; ++i) {
		uint16_t *line = &image[i * 80];
		for (int j = 0; j < 78/*80*/; ++j) {
			if (BROKEN_COLUMN == j) {
				continue;
			}
			uint16_t v = line[j] = ntohs(line[j]);
			if (__builtin_expect(v > max_v, 0)) {
				max_v = v;
			}
			if (__builtin_expect(v < min_v, 0)) {
				min_v = v;
			}
		}
	}
#else
	// Hard set
	min_v = 0x2000;
	max_v = 0x25C0;
#endif

	//printf("Found min %04X max %04X\n", min_v, max_v);

	Mat preScale(60, 80, CV_16UC3);
	//Mat postBlur(120, 160, CV_16UC3);
	Mat postScale(120, 160, CV_16UC3);
	const uint16_t scale = max_v - min_v;
	const float scale_f = 1.0f / scale;

	for (int i = 0; i < 60; ++i) {
		const int idex = i * 80;
		uint16_t *line_out = &image[idex];
		// Currently last 2 pixels are bugged...
		line_out[78] = line_out[79] = line_out[77];
		line_out[BROKEN_COLUMN] = (line_out[BROKEN_COLUMN - 1] >> 1)
				+ (line_out[BROKEN_COLUMN + 1] >> 1);

		for (int j = 0; j < 80; ++j) {
			if (__builtin_expect(line_out[j] < min_v, 0)) {
				line_out[j] = min_v;
			} else if (__builtin_expect(line_out[j] > max_v, 0)) {
				line_out[j] = max_v;
			}
			const float frac = (line_out[j] - min_v) * scale_f;
			float r = frac - 0.75f, g = frac - 0.5f, b = frac - 0.25f;

			r *= 4.0f;
			if (r > 0) {
				r *= -1.0f;
			}
			r += 1.5f;
			if (r < 0.0f) {
				r = 0.0f;
			} else if (r > 1.0f) {
				r = 1.0f;
			}

			b *= 4.0f;
			if (b > 0) {
				b *= -1.0f;
			}
			b += 1.5f;
			if (b < 0.0f) {
				b = 0.0f;
			} else if (b > 1.0f) {
				b = 1.0f;
			}

			g *= 4.0f;
			if (g > 0) {
				g *= -1.0f;
			}
			g += 1.5f;
			if (g < 0.0f) {
				g = 0.0f;
			} else if (g > 1.0f) {
				g = 1.0f;
			}

			Vec3w &pixel = preScale.at<Vec3w>(i, j);
			pixel[0] = r * ((uint16_t)-1);
			pixel[1] = g * ((uint16_t)-1);
			pixel[2] = b * ((uint16_t)-1);
		}
	}
	cv::resize(preScale, postScale, postScale.size(), 0, 0, INTER_LINEAR);
	// WAAAYYYY too slow
	//cv::fastNlMeansDenoisingColored(postScale, postBlur);
	for (int i = 0; i < 120; ++i) {
		uint16_t *line_out_f = (uint16_t *) &out_image[i * 160];
		for (int j = 0; j < 160; ++j) {
			Vec3w &pixel = postScale.at<Vec3w>(i, j);
			uint16_t r_final = (pixel[0] >>  0) & ST7735_RED;
			uint16_t g_final = (pixel[1] >>  5) & ST7735_GREEN;
			uint16_t b_final = (pixel[2] >> 11) & ST7735_BLUE;
			line_out_f[j] = htons(r_final | g_final | b_final);
		}
	}
	return 0;
}

static inline void useRegister(int fid, uint16_t reg) {
	reg = htons(reg);

	if (ioctl(fid, I2C_SLAVE, 0x2A) < 0) {
		error(-1, errno, "Error setting slave address");
	}
	errno = 0;
	if (sizeof(uint16_t) != write(fid, &reg, sizeof(uint16_t))) {
		error(-1, errno, "Error writing register");
	}
}

void test_i2c() {
	int fid = open("/dev/i2c-1", O_RDWR);
	if (fid == -1) {
		error(-1, errno, "Could not open /dev/i2c-1");
	}
	uint16_t result;

	uint16_t status_reg = 0x0002;
	useRegister(fid, status_reg);

	if (ioctl(fid, I2C_SLAVE, 0x2A) < 0) {
		error(-1, errno, "Error setting slave address");
	}
	if (sizeof(result) != read(fid, (uint8_t *) &result, sizeof(result))) {
		error(-1, errno, "Error reading data");
	}
	result = ntohs(result);
	printf("Found result %04X\n", result);
	if (close(fid) == -1) {
		error(-1, errno, "Error closing /dev/i2c-1");
	}
}

const uint16_t colors[] = {
ST7735_BLACK,
ST7735_BLUE,
ST7735_RED,
ST7735_GREEN,
ST7735_CYAN,
ST7735_MAGENTA,
ST7735_YELLOW,
ST7735_WHITE };

int main(int argc, char **argv) {
	int lcd_fd = open(ST7735_DEV_NAME, O_WRONLY), lepton_fd = open(
			LEPTON_DEV_NAME, O_RDONLY);
	upm::ST7735 *lcd = new upm::ST7735(31, 45, 32, 46);
	lcd->lcdCSOff();
	mraa::Gpio lepton_cs = mraa::Gpio(36), light = mraa::Gpio(20);
	uint16_t in_image[80 * 60], out_image[160 * 120];
	uint8_t out_buff[160 * 128 * 2];
	if (lcd_fd == -1) {
		error(-1, errno, "Error opening %s", ST7735_DEV_NAME);
	}

	if (lepton_fd == -1) {
		error(-1, errno, "Error opening %s", LEPTON_DEV_NAME);
	}

	exitIfMRAAError(lepton_cs.dir(mraa::DIR_OUT_HIGH));
	exitIfMRAAError(light.dir(mraa::DIR_OUT_HIGH));

	while (isrunning) {
		uint16_t color = colors[std::rand() % 8];
		uint16_t *out_buff16 = (uint16_t *) out_buff;
		for (int i = 0; i < 160 << 7; ++i) {
			out_buff16[i] = color;
		}
		usleep(200000);
		exitIfMRAAError(lepton_cs.write(0)); // Select device
		if (captureImage(in_image, lepton_fd)) {
			exitIfMRAAError(lepton_cs.write(1)); // High to de-select
			continue;
		}
		exitIfMRAAError(lepton_cs.write(1)); // High to de-select
		printImg(in_image, out_image);

		for (int i = 0; i < 160; ++i) {
			uint16_t *line = (uint16_t *) &out_buff[i << 8];
			for (int j = 0; j < 120; ++j) {
				// transpose
				const int idex = j * 160 + i;
				line[j] = out_image[idex];
			}
		}

		lcd->lcdCSOn();
		ssize_t written = 0;
		for (ssize_t remaining = sizeof(out_buff); remaining > 0; remaining -=
				written) {
			ssize_t this_write = remaining > 16384 ? 16384 : remaining;
			written = write(lcd_fd, &out_buff[sizeof(out_buff) - remaining],
					this_write);
			if (-1 == written) {
				lcd->lcdCSOff();
				error(-1, errno, "Error writing to %s", ST7735_DEV_NAME);
			}
		}
		lcd->lcdCSOff();
	}

	if (-1 == close(lcd_fd)) {
		error(0, errno, "Error closing %s", ST7735_DEV_NAME);
	}
	if (-1 == close(lepton_fd)) {
		error(0, errno, "Error closing %s", LEPTON_DEV_NAME);
	}
	delete lcd;
	return 0;
}

void sig_handler(int signum) {
	if (signum == SIGINT)
		isrunning = 0;
}
