# Optical Flow Motion Amplification

Real-time motion amplification using a high-speed monochrome camera and optical flow. Developed on the Jetson AGX Xavier with a Sony IMX252 camera.

[![Motion amplification demonstration](https://img.youtube.com/vi/WWkTftrcGMY/hqdefault.jpg)](https://youtu.be/WWkTftrcGMY)

## How it works

The program captures 1024 × 768 grayscale frames through V4L2 and tracks motion in two selected regions relative to a reference patch. A biquad bandpass filter isolates movement around 10.45 Hz. The filtered displacement drives localized image warping, leaving the rest of the image unchanged.

The original setup captured approximately 236 fps and displayed a portrait preview at approximately 59 fps.

## Requirements

- Linux and a V4L2 camera supporting 1024 × 768 GREY at 200+ fps
- C++17, X11, Xext and pthreads

On Ubuntu: `build-essential`, `libx11-dev`, `libxext-dev`.

## Build and run

```bash
make
make test
./imx252_algo3_flow_v2 15
```

Uses `/dev/video0` and the current X11 display. The argument sets amplification gain (0–80; default 15). The capture mode targets approximately 238 fps.

## Controls

| Input | Action |
| --- | --- |
| Left click | Select first region |
| Right click | Select second region |
| Space | Toggle amplified/original view |
| + / - | Adjust gain |
| R | Reset tracking |
| Q / Esc | Quit |

## Source files

- `imx252_algo3_flow_v2.cpp` — capture, optical flow, image warping and preview
- `imx252_flow_biquad.hpp` — bandpass filter
- `imx252_flow_biquad_test.cpp` — filter response test
- `Makefile` — build and test targets

## License

MIT — © 2026 Elad Orbach.
