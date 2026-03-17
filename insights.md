# Project Insights

## Minimum Crop Size for Neural Network Input

When cropping regions from a frame to feed a neural network, enforce a minimum crop size equal to the model's input resolution. For example, palm detection uses a 192x192 model — crops should be at least 192px on each side.

**Why:** Cropping a 50px region and upscaling to 192px produces blurry input with no more detail than the original pixels. Using ≥192 real pixels ensures the model sees full-resolution detail, maximizing detection accuracy at range.

**Applied in:** `person_palm_croppers.cpp` — `MIN_CROP_PX = 192.0f`
