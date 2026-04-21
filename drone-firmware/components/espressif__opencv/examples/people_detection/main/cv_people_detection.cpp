/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

#include <iostream>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using namespace std;
using namespace cv;

extern "C" void cv_init_people_detect();
extern "C" void cv_people_detect(uint8_t* data, int width, int heigth);

using namespace cv;
using namespace std;
class Detector
{
    enum Mode { Default, Daimler } m;
    HOGDescriptor hog, hog_d;
public:
    Detector() : m(Default), hog(), hog_d(Size(48, 96), Size(16, 16), Size(8, 8), Size(8, 8), 9)
    {
        hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
        hog_d.setSVMDetector(HOGDescriptor::getDaimlerPeopleDetector());
    }
    void toggleMode() { m = (m == Default ? Daimler : Default); }
    string modeName() const { return (m == Default ? "Default" : "Daimler"); }
    vector<Rect> detect(InputArray img)
    {
        // Run the detector with default parameters. to get a higher hit-rate
        // (and more false alarms, respectively), decrease the hitThreshold and
        // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).
        vector<Rect> found;
        if (m == Default)
            hog.detectMultiScale(img, found, 0, Size(8,8), Size(), 1.05, 2, false);
        else if (m == Daimler)
            hog_d.detectMultiScale(img, found, 0, Size(8,8), Size(), 1.05, 2, true);
        return found;
    }
    void adjustRect(Rect & r) const
    {
        // The HOG detector returns slightly larger rectangles than the real objects,
        // so we slightly shrink the rectangles to get a nicer output.
        r.x += cvRound(r.width*0.1);
        r.width = cvRound(r.width*0.8);
        r.y += cvRound(r.height*0.07);
        r.height = cvRound(r.height*0.8);
    }
};

extern "C" void cv_print_info();
void cv_print_info()
{
    printf("OpenCV info print: %s", cv::getBuildInformation().c_str());
}

void cv_init_people_detect()
{
    
}
Detector detector;

void cv_people_detect(uint8_t* data, int width, int heigth)
{
    cv::Mat frame( heigth, width, CV_8UC1, (void*)data );
    detector.toggleMode();
    vector<Rect> found = detector.detect(frame);
        for (size_t i = 0; i < found.size(); i++)
        {
            std::cout << "Found[" << i << "] = " << found[i] << std::endl;
        }
        std::cout << "Found: " << found.size() << ", Mode - " << detector.modeName() << std::endl;
}

