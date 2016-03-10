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
#include <mraa.h>
#include <string>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

static const char *dev_name = "/dev/lepton";
void sig_handler(int signum);
static sig_atomic_t volatile isrunning = 1;
uint32_t frame_counter = 0;
#define exitIfMRAAError(x) exitIfMRAAError_internal(x, __FILE__, __LINE__)

static void exitIfMRAAError_internal(mraa_result_t result, const char *filename,
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
	uint16_t min_v = (uint16_t) -1;
	uint16_t max_v = 0;
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

	uint16_t scale = max_v - min_v;
	Mat grayScaleImage(60, 80, CV_8UC1);
	uint8_t *img_out = (uint8_t *) &grayScaleImage.data[0];
	for (int i = 0; i < 60; ++i) {
		const int idex = i * 80;
		uint16_t *line = &image[idex];
		uint8_t *line_out = &img_out[idex];
		for (int j = 0; j < 78/*80*/; ++j) {
			line_out[j] = (((line[j] - min_v) << 8) / scale) & 0xFF;
		}
		line_out[78] = line_out[79] = 0;
	}

	uint8_t strbuff[1024];

	std::vector<uchar> buff;
	try {
		if (!imencode(".jpeg", grayScaleImage, buff)) {
			error(0, 0, "Error writing jpeg image to buffer");
			return -1;
		}
	} catch (cv::Exception &ex) {
		std::cout << "Error writing image: " <<ex.what() << std::endl;
		return -1;
	}

	ssize_t out_str_len = snprintf((char *) strbuff, sizeof(strbuff),
			"Content-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n", buff.size());
	if (out_str_len < 0) {
		error(0, errno, "Error writing output header");
		return -1;
	}
	if (safe_write(out_fd, strbuff, out_str_len) < 0) {
		return -1;
	}
	if (safe_write(out_fd, buff.data(), buff.size()) < 0) {
		return -1;
	}
	if (safe_write(out_fd, (uint8_t *) boundary_end_str,
			sizeof(boundary_end_str) - 1) < 0) {
		return -1;
	}
	return 0;
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

	if (-1 == listen(socket_fd, 1)) {
		error(-1, errno, "Error listening");
	}

	signal(SIGINT, &sig_handler);
	cs = mraa_gpio_init(45);
	if (cs == NULL) {
		error(1, errno, "Error initializing cs on gpio10");
	}
	exitIfMRAAError(mraa_gpio_dir(cs, MRAA_GPIO_OUT));
	exitIfMRAAError(mraa_gpio_write(cs, 1)); // write CS`
	printf("Syncing with Lepton...\n");
	usleep(200000);
	fd = open(dev_name, O_RDONLY);
	if (fd == -1) {
		error(-1, errno, "Error opening %s", dev_name);
	}

	printf("Activating Lepton...\n");
	int out_fd = -1;
	while (isrunning) {
		if (-1 == out_fd || captureImage(image, cs, fd) < 0) {
			exitIfMRAAError(mraa_gpio_write(cs, 1)); // High to de-select
			while (-1 == out_fd && isrunning) {
				out_fd = accept(socket_fd, (struct sockaddr *) &their_sin,
						&sin_size);
				if (-1 == out_fd) {
					error(0, errno, "Error accepting socket, retrying");
				} else {
					char out_buffer[4096] = { 0 };
					snprintf(out_buffer, sizeof(out_buffer),
							"HTTP/1.0 20 OK\r\n"
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
				if (-1 == printImg(image, out_fd)) {
					error(0, 0, "Problem with socket, closing. %s", isrunning ? "" : "Also is done");
					close(out_fd);
					out_fd = -1;
				}
			}
		}
	}
	exitIfMRAAError(mraa_gpio_write(cs, 1)); // High to de-select
	exitIfMRAAError(mraa_gpio_close(cs));
	if (out_fd != -1) {
		close(out_fd);
	}
	close(fd);
	return 0;
}

void sig_handler(int signum) {
	if (signum == SIGINT)
		isrunning = 0;
}
