# Image distortion correction tool
OpenCV library used for distortion correction.
Calibration method - Charuco.
Sample application expects image files in the folder where executable is located.

List of required images:

|Image File                  | Description                                         |
|----------------------------|-----------------------------------------------------|
|calibration_image_[1-4].jpg | images for calibration                              |
|image_for_correction.jpg    | image for demonstration of the distortion correction|

# BUILD

```
$ cd ImageDistortionCorrection
$ cmake .
$ make
```

# RUN

1. Locate required images in the ImageDistortionCorrection folder
2. Run sample
```
$ ./ImageDistortion
```
