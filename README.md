lepton-edison-read
-------------------
Read from the FLIR Lepton using the Intel Edison

# Requirements
 * The patchset from @primiano [enabling DMA](https://github.com/drcrallen/meta-primiano-dma)
 * The module adding the `/dev/lepton` [device](https://github.com/drcrallen/lepton-edison-kernel-module)
 * `mraa` (should be on edison already)
 * OpenCV modules (`opencv-highgui`, `opencv-core`, `opencv-imgproc`)
 * C++ libraries

# Building
 * Building is a bit complicated right now. Currently it requires the Intel Eclipse Platform to build.
 * If anyone can make a good makefile to fix the build, and a list of dependent libraries, that would be awesome

# Running

1. The DMA patch set and lepton module should be running on the edison.
1. Run the executable on the edison
1. The expected output is at the end of this section
1. On a remote machine, open up [VLC](http://www.videolan.org/vlc/index.html)
1. Open up a network stream to the device on port `8888` like `http://192.168.1.139:8888/foo.mjpg`

The expected output on the Edison side before connecting looks like the following:
```
Syncing with Lepton...
Activating Lepton...
```

The stream is a stream of multiple jpeg images and consumes about 15KBs of network bandwidth in simple tests.
