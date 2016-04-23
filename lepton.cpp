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
#include <linux/i2c-dev.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netinet/ip.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Something doesn't translate properly to VLC
//#define USE_16BIT_GRAY_RAW

// Attempt to auto-scale the data
#define AUTO_SCALE

using namespace cv;
using namespace std;

static const char *dev_name = "/dev/lepton";
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
		case MRAA_ERROR_PLATFORM_ALREADY_INITIALISED:
			errMsg = "MRAA_ERROR_PLATFORM_ALREADY_INITIALISED";
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

static int captureImage(uint16_t *image, mraa_gpio_context cs, int fd) {
	const int line_len = 164;
	int result, i = 0, retval = 0;
	uint8_t buff[line_len * 60];
	while (i < 60 && isrunning) {
		uint8_t *line = &buff[i * line_len];
		uint16_t *line16 = (uint16_t *) line;
		memset(line, 0, line_len);
		result = read(fd, line, line_len);
		if (__builtin_expect(result == -1, 0)) {
			error(0, errno, "Error reading from [%s]", dev_name);
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
			usleep(50);
			;
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

#define BOUNDARY "hfihuuhf782hf4278fh2h4f7842hfh87942h"

static const char boundary_end_str[] = "\r\n--" BOUNDARY "\r\n";

static int safe_write(int fd, uint8_t *ptr, ssize_t len) {
	for (ssize_t write_remaining = len, my_write = 0; write_remaining > 0;
			write_remaining -= my_write) {
		my_write = write(fd, ptr, write_remaining);
		if (__builtin_expect(-1 == my_write, 0)) {
			// Skip things like EAGAIN for now
			error(0, errno, "Error writing image data");
			return -1;
		}
		ptr = &ptr[my_write];
	}
	return len;
}

// image is still BigEndian when it gets here
static int printImg(uint16_t *image, int out_fd) {
	uint16_t min_v = (uint16_t) -1, max_v = 0;
#ifdef AUTO_SCALE
	for (int i = 0; i < 60; ++i) {
		uint16_t *line = &image[i * 80];
		for (int j = 0; j < 78/*80*/; ++j) {
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

	Mat grayScaleImage(60, 80, CV_16UC1), colorizedImage(60, 80, CV_16UC3);
	uint16_t *img_out = (uint16_t *) &grayScaleImage.data[0];
	const uint16_t scale = max_v - min_v;
	const float scale_f = 1.0f / scale;

	memcpy(img_out, image, 80 * 60 * sizeof(uint16_t));
	memset(colorizedImage.data, 0, 60 * 80 * 3 * sizeof(uint16_t));
	for (int i = 0; i < 60; ++i) {
		const int idex = i * 80;
		uint16_t *line_out = &img_out[idex];
		uint16_t *line_out_f = (uint16_t *) &colorizedImage.data[i * 80 * sizeof(uint16_t) * 3];
		// Currently last 2 pixels are bugged...
		line_out[78] = line_out[79] = min_v;

		for (int j = 0; j < 80; ++j) {
			if (__builtin_expect(line_out[j] < min_v, 0)) {
				line_out[j] = min_v;
			} else if (__builtin_expect(line_out[j] > max_v, 0)) {
				line_out[j] = max_v;
			}
			const float frac = (line_out[j] - min_v) * scale_f;
			float r = frac - 0.75f, g = frac - 0.5f, b = frac - 0.25f;

			r *= 4.0f;
			if(r > 0) {
				r *= -1.0f;
			}
			r += 1.5f;
			if(r < 0.0f) {
				r = 0.0f;
			} else if (r > 1.0f) {
				r = 1.0f;
			}

			b *= 4.0f;
			if(b > 0) {
				b *= -1.0f;
			}
			b += 1.5f;
			if(b < 0.0f) {
				b = 0.0f;
			} else if (b > 1.0f) {
				b = 1.0f;
			}

			g *= 4.0f;
			if(g > 0) {
				g *= -1.0f;
			}
			g += 1.5f;
			if(g < 0.0f) {
				g = 0.0f;
			} else if (g > 1.0f) {
				g = 1.0f;
			}

			uint16_t *pixel = &line_out_f[3 * j];
			pixel[0] = b * ((uint16_t)-1);
			pixel[1] = g * ((uint16_t)-1);
			pixel[2] = r * ((uint16_t)-1);
		}
	}

	uint8_t strbuff[4098 * 4];
	uint16_t out_len = 0;

	std::vector<uchar> buff;
	try {
		// TODO: get full 16 bits per channel for high fidelity colors
		colorizedImage.convertTo(colorizedImage, CV_8U, 1.0/256);
		if (!imencode(".jpeg", colorizedImage, buff)) {
			error(0, 0, "Error writing jpeg image to buffer");
			return -1;
		}
	} catch (cv::Exception &ex) {
		std::cout << "Error writing image to buffer: " << ex.what()
				<< std::endl;
		return -1;
	}

	ssize_t out_str_len = snprintf((char *) strbuff, sizeof(strbuff),
			"Content-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n",
			buff.size());
	if (__builtin_expect(out_str_len < 0, 0)) {
		error(0, errno, "Error writing output header");
		return -1;
	}
	out_len += out_str_len;

	if (__builtin_expect(
			sizeof(strbuff)
					< buff.size() + out_str_len + sizeof(boundary_end_str),
			0)) {
		error(0, 0, "Output data too big for buffer! %d bytes in image",
				(uint32_t) buff.size());
		return -1;
	}
	memcpy(&strbuff[out_len], buff.data(), buff.size());

	out_len += buff.size();

	memcpy(&strbuff[out_len], boundary_end_str, sizeof(boundary_end_str) - 1);
	out_len += sizeof(boundary_end_str) - 1; // Don't include '\0'

	if (safe_write(out_fd, strbuff, out_len) < 0) {
		return -1;
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

int main(int argc, char **argv) {
	int fd;
	uint32_t frameNum = 0;
	uint16_t image[80 * 60];
	mraa_gpio_context cs;
	Size size(80, 60);

	socklen_t sin_size = sizeof(struct sockaddr_in);
	int socket_fd = socket(PF_INET, SOCK_STREAM, 0);
	struct sockaddr_in sin, their_sin;
	sin.sin_family = AF_INET;
	sin.sin_addr.s_addr = INADDR_ANY;
	sin.sin_port = htons(8888);
	memset(&(sin.sin_zero), '\0', 8);
	if (-1
			== bind(socket_fd, (const struct sockaddr *) &sin,
					sizeof(sockaddr_in))) {
		error(-1, errno, "Error binding to 8888");
	}

	unsigned int n, m = sizeof(n);
	if (-1 == getsockopt(socket_fd, SOL_SOCKET, SO_SNDBUF, (void *) &n, &m)) {
		error(-1, errno, "Error getting socket information");
	}
	printf("Socket buffer size %d\n", (int) n);
	n = 4096; // Small buffers
	if (-1 == setsockopt(socket_fd,
	SOL_SOCKET,
	SO_SNDBUF, (void *) &n, sizeof(n))) {
		error(-1, errno, "Error setting socket size");
	}

	if (-1 == getsockopt(socket_fd, SOL_SOCKET, SO_SNDBUF, (void *) &n, &m)) {
		error(-1, errno, "Error getting socket information");
	}
	printf("Socket buffer size reset to %d\n", (int) n);
	n = 1;
	if (-1 == setsockopt(socket_fd,
	IPPROTO_TCP,
	TCP_NODELAY, (void *) &n, sizeof(n))) {
		error(-1, errno, "Error setting no delay");
	}
	if (-1 == listen(socket_fd, 1)) {
		error(-1, errno, "Error listening");
	}

	signal(SIGINT, &sig_handler);
	cs = mraa_gpio_init(45);
	if (cs == NULL) {
		error(1, errno, "Error initializing cs on gpio10");
	}
	exitIfMRAAError(mraa_gpio_dir(cs, MRAA_GPIO_OUT));
	//exitIfMRAAError(mraa_gpio_write(cs, 0)); // write CS`
	//test_i2c();
	exitIfMRAAError(mraa_gpio_write(cs, 1)); // write CS`
	printf("Syncing with Lepton...\n");
	usleep(200000);
	fd = open(dev_name, O_RDONLY);
	if (fd == -1) {
		error(-1, errno, "Error opening %s", dev_name);
	}

	printf("Activating Lepton...\n");
	int out_fd = -1;
	char out_buffer[4096] = { 0 };
	while (isrunning) {
		if (-1 == out_fd || captureImage(image, cs, fd) < 0) {
			exitIfMRAAError(mraa_gpio_write(cs, 1)); // High to de-select
			while (-1 == out_fd && isrunning) {
				out_fd = accept(socket_fd, (struct sockaddr *) &their_sin,
						&sin_size);
				if (-1 == out_fd) {
					error(0, errno, "Error accepting socket, retrying");
				} else {
					snprintf(out_buffer, sizeof(out_buffer),
							"HTTP/1.1 20 OK\r\n"
									"Content-Type: multipart/x-mixed-replace;boundary=" BOUNDARY "\r\n"
							"\r\n--" BOUNDARY "\r\n");
					int len = strlen(out_buffer);
					if (len != write(out_fd, out_buffer, len)) {
						error(0, 0, "Error writing bytes");
						close(out_fd);
						out_fd = -1;
					}
				}
			};
			frameNum = 0;
			// Actually only needs to be ~185ms
			usleep(200000);
			exitIfMRAAError(mraa_gpio_write(cs, 0)); // Select device
		} else {
			if (frameNum++ % 3 == 0) {
				int flag = 1;
				if(setsockopt(out_fd, SOL_TCP, TCP_CORK, &flag, sizeof(flag))) {
					error(0, errno, "Error setting TCP_CORK on socket");
				}
				if (-1 == printImg(image, out_fd)) {
					error(0, 0, "Problem with socket, closing. %s",
							isrunning ? "" : "Also is done");
					close(out_fd);
					out_fd = -1;
				}
				flag = 0;
				if(setsockopt(out_fd, SOL_TCP, TCP_CORK, &flag, sizeof(flag))) {
					error(0, errno, "Error removing TCP_CORK on socket");
				}
			}
		}
	}
	exitIfMRAAError(mraa_gpio_write(cs, 1)); // High to de-select
	exitIfMRAAError(mraa_gpio_close(cs));
	if (out_fd != -1) {
		close(out_fd);
	}
	printf("Exiting\n");
	close(fd);
	return 0;
}

void sig_handler(int signum) {
	if (signum == SIGINT)
		isrunning = 0;
}
