# Invisibility Cloak

> A Harry Potter–style invisibility effect in real time using colour-based background substitution.

---

## Table of Contents

- [How it works](#how-it-works)
- [Controls](#controls)
- [Step-by-step usage](#usage)
- [Understanding the code](#understanding-the-code)


---

## How it works

The effect relies on three steps:

1. **Calibrate** : sample the HSV colour of your cloth from a rectangular ROI. A fixed margin around the mean hue, saturation, and value gives a robust detection range.
2. **Capture background** : save a clean frame of the scene while you are out of shot.
3. **Mask & replace** : every frame, pixels that fall inside the calibrated HSV range are identified as the cloth (the mask). Those pixels are replaced with the corresponding pixels from the saved background, making the cloth appear transparent.

Morphological operations (opening + dilation) clean up the binary mask so small noise blobs are removed and the detected cloth region is solid.

---

## Controls

| Key | Action |
|-----|--------|
| `SPACE` | Calibrate cloth colour from the rectangle |
| `B` | Capture background (step out of frame first) |
| `R` | Reset (recalibrate from scratch) |
| `Q` | Quit |

---

## Usage

---

1. Open Terminal
2. Navigate to ../Pixels/4_cv_basics/9_invisibility_cloak
3. run   ```make clean``` to clean out any previous builds
4. run ```make build SRC=main.cpp link=src/invisibility_cloak.cpp``` to build the executable
5. run ```./invisibility_cloak```
6. You should now have an imshow window capturing a video.
7. Follow the steps printed on the terminal to calibrate and activate the invisibility cloak
9. Press ```Q``` to exit

> **Tip**: use a cloth with a single, saturated colour (bright red, green, or blue works best). Avoid colours that appear elsewhere in the scene background.

---

## Understanding the code

### `invisibility_cloak.cpp`

#### `putText()`

Centers a text string horizontally on the frame. It uses `cv::getTextSize` to measure the rendered width, then computes the x offset so the text sits in the middle regardless of string length.

#### `drawCalibrationRect()`

Draws the yellow guide rectangle and its label before calibration. The rectangle is defined once in `main.cpp` as a centred `cv::Rect` and passed to frame

#### `calibrateCloth()`

```cpp
cv::Mat roi = frame(rect);
cv::Mat hsvROI;
cv::cvtColor(roi, hsvROI, cv::COLOR_BGR2HSV);

cv::Scalar mean, stddev;
cv::meanStdDev(hsvROI, mean, stddev);

const int hueMargin = 15;
const int satMargin = 50;
const int valMargin = 50;

lowerBound = cv::Scalar(max(0.0, mean[0] - hueMargin), max(0.0, mean[1] - satMargin), max(0.0, mean[2] - valMargin));
upperBound = cv::Scalar(min(180.0, mean[0] + hueMargin), min(255.0, mean[1] + satMargin), min(255.0, mean[2] + valMargin));

```

Using **mean HSV with fixed margins** instead of a hand-picked colour makes the calibration automatic. The saturation and value margins tolerate minor lighting variation across frames.

#### `getClothMask()`

```cpp
cv::Mat hsv;
cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

cv::Mat mask;
cv::inRange(hsv, lowerBound, upperBound, mask);

cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
cv::morphologyEx(mask, mask, cv::MORPH_OPEN,   kernel);
cv::morphologyEx(mask, mask, cv::MORPH_DILATE, kernel);

return mask;
```

`inRange` produces a binary mask: 255 where the pixel colour falls within the HSV bounds, 0 elsewhere. The two morphological passes are critical: **opening** (erosion then dilation) removes small speckles, and the subsequent **dilation** restores the edges of the actual cloth region.

### `main.cpp`

The main loop ties everything together. Key logic:

```cpp
if (backgroundCaptured && calibrated) {
    cv::Mat mask = getClothMask(frame, lowerBound, upperBound);
    background.copyTo(display, mask);
}
```

`copyTo` with a mask argument copies pixels from `background` into `display` only where the mask is non-zero, replacing the cloth with the saved scene.

