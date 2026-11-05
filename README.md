# Optical Flow Motion Amplification

Real-time motion amplification from a high-speed monochrome camera using optical flow, a causal bandpass filter, and local image warping. Developed with a Sony IMX252-based camera and Jetson AGX Xavier.

**Demonstration:** https://youtu.be/WWkTftrcGMY

## How it works

The application captures 1024 x 768 grayscale frames through Linux V4L2 and estimates optical flow in two selected image regions relative to a stationary reference patch. A causal biquad bandpass filter isolates displacement near 10.45 Hz. The filtered motion is used to warp the selected regions with smooth Gaussian falloff. The original image is shown outside those regions.

Capture and analysis run independently of the X11 viewer. The preview rotates the image into a 768 x 1024 portrait window, showing fresh frames at up to 60 fps.

## Requirements

* Linux with a compatible V4L2 camera supporting 1024 x 768 GREY at more than 200 fps
* X11 display (MIT-SHM preferred)
* C++17 compiler, pthreads, Xlib and Xext

On Ubuntu, install the development packages `build-essential`, `libx11-dev`, and `libxext-dev`.

## Build and run

```bash
make
make test
./imx252_algo3_flow_v2 15
```

The application opens `/dev/video0`, negotiates a GREY stream close to 238 fps and uses the current X11 display. The argument is the nominal amplification gain (0 to 80, default 15). Check camera mode support before running. The running application adjusts the stream settings and attempts to restore the previous ones on normal shutdown.

## Controls

| Input | Action |
| --- | --- |
| Left click | Select first tracking region |
| Right click | Select second tracking region |
| Space | Switch between original and amplified views |
| + and - | Adjust gain |
| R | Reset tracking |
| Q or Escape | Quit |

## Files

* `imx252_algo3_flow_v2.cpp` - capture, tracking, warping and viewer
* `imx252_flow_biquad.hpp` - causal displacement filter
* `imx252_flow_biquad_test.cpp` - synthetic filter response test
* `Makefile` - build and test targets

## Measurement limits

The original setup sustained approximately 236 fps capture and 59 fps display. These figures are software observations, not measured optical glass-to-glass latency. The 10.45 Hz target comes from measured image motion and is not an independently confirmed excitation frequency. The displayed amplification gain is nominal rather than physically calibrated. Adapt the camera mode, tracking regions and filter to other experiments.

## License

MIT. Copyright (c) 2026 Elad Orbach.
