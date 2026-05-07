# OpenCV library build for esp-idf

This component used to build and include OpenCV library into the esp-idf as a component for esp32, esp32s2, esp32s3 and esp32p4 and esp32s3 CPUs.
Now the opencv could be build with IDF version >= 4.4. 
This component was tested with opencv V 4.10.0.

More details about OpenCV can be found [here](https://opencv.org/).

To clone component from the github repository please follow a next command:

```
git clone https://github.com/espressif/esp-opencv-component.git espressif__opencv
cd espressif__opencv
git submodule update --recursive
```

The project has the following folders:

* opencv - OpenCV source code directory, added as git submodule
* opencv_contrib - additional modules for OpenCV, added as git submodule
* examples - directory with opencv examples running on Esp32-cam board.

## The examples:

* feature2d - extraction of 2d features form the camera image.
* motion_detection - example detect the motion on the picture from camera and print the coordinates of the object to the console.
* object_tracking - track the object located on center of the camera.
* people_detection - the example has to detect people on the image.
* text_area_detection - detect area on the image where text expected.

