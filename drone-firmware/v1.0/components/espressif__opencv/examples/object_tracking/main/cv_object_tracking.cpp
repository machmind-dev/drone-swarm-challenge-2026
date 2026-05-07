/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>

#include <iostream>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using namespace std;
using namespace cv;

extern "C" void cv_init_object_tracking(int pos_x, int pos_y);
extern "C" void cv_object_tracking(uint8_t* data, int width, int heigth);

extern "C" void cv_print_info();
void cv_print_info()
{
    printf("OpenCV info print: %s", cv::getBuildInformation().c_str());
}

cv::Rect2i roi;
Ptr<Tracker> tracker;
cv::Mat last_frame;
bool track_initialized = false;
void cv_init_object_tracking(int pos_x, int pos_y)
{
    tracker = TrackerKCF::create();

    roi = cv::Rect2i(pos_x, pos_y, 30, 30);
    
}

void cv_object_tracking(uint8_t* data, int width, int heigth)
{
    cv::Mat frame( heigth, width, CV_8UC1, (void*)data );
    frame.copyTo(last_frame);
    if (!track_initialized)
    {
        track_initialized = true;
        tracker->init(last_frame, roi);
    }
    tracker->update(last_frame, roi);
    std::cout << "Pos: " << cv::Point2f(roi.x, roi.y) << std::endl;

}

