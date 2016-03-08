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
 *  Created on: Mar 5, 2016
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
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>

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

// image is still BigEndian when it gets here
static void printImg(uint16_t *image, VideoWriter &videoWriter) {
	//char fname[128];
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
	// gst-launch-1.0 filesrc location=video.avi ! avidemux ! jpegdec ! theoraenc ! queue ! rtptheorapay config-interval=2 ! udpsink host=192.168.1.105 port=8554
	uint16_t scale = max_v - min_v;
	Mat grayScaleImage(60, 80, CV_8UC1);
	for (int i = 0; i < 60; ++i) {
		const int idex = i * 80;
		uint16_t *line = &image[idex];
		uint8_t *line8 = &grayScaleImage.data[idex];
		for (int j = 0; j < 78/*80*/; ++j) {
			line8[j] = (((line[j] - min_v)<<8) / scale) & 0xFF;
		}
		line8[78] = line8[79] = 0;
	}
	try {
		videoWriter << grayScaleImage;
	}catch (cv::Exception &ex) {
		std::cout << "Error writing video frame" << std::endl << ex.what() << std::endl;
		isrunning = 0;
	}
	//isrunning = 0;
}

int main(int argc, char **argv) {
	int fd;
	uint32_t frameNum = 0;
	uint16_t image[80 * 60];
	mraa_gpio_context cs;
	Size size(80, 60);
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
	printf("Linking video\n");
	VideoWriter videoWriter;
	if (!videoWriter.open(
			"/home/root/video.avi",
			CV_FOURCC('M', 'J', 'P', 'G'),
			9.0, size, 0) || !videoWriter.isOpened()) {
		error(-1, 0, "Error opening video");
	}

	printf("Activating Lepton...\n");
	exitIfMRAAError(mraa_gpio_write(cs, 0)); // Select device
	while (isrunning) {
		if (captureImage(image, cs, fd) < 0) {
			exitIfMRAAError(mraa_gpio_write(cs, 1)); // High to de-select
			frameNum = 0;
			usleep(200000);
			exitIfMRAAError(mraa_gpio_write(cs, 0)); // Select device
		} else {
			if (frameNum++ % 3 == 0) {
				printImg(image, videoWriter);
			}
		}
	}
	exitIfMRAAError(mraa_gpio_write(cs, 1)); // High to de-select
	exitIfMRAAError(mraa_gpio_close(cs));
	close(fd);
	printf("Finalizing video\n");
	//videoWriter.release();
	return 0;
}

void sig_handler(int signum) {
	if (signum == SIGINT)
		isrunning = 0;
}
