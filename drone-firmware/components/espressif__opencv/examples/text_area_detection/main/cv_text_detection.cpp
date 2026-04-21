/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/imgcodecs/legacy/constants_c.h>
#include <opencv2/core.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc.hpp>

#include <opencv2/core/utility.hpp>

#include "opencv2/opencv_modules.hpp"

#include "opencv2/core.hpp"
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/core/softfloat.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc_c.h>

#include <iostream>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using namespace std;
using namespace cv;

extern "C" void cv_text_detection(uint8_t* data, int width, int heigth);

extern "C" void cv_print_info();
void cv_print_info()
{
    printf("OpenCV info print: %s", cv::getBuildInformation().c_str());
}

void cv_text_detection(uint8_t* data, int width, int heigth)
{
    printf("OpenCV text detection.\n");
    cv::Mat small( heigth, width, CV_8UC1, (void*)data );

    cv::Mat grad;
    cv::Mat morphKernel = cv::getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
    cv::morphologyEx(small, grad, MORPH_GRADIENT, morphKernel);    
    // binarize
    cv::Mat bw;
    cv::threshold(grad, bw, 0.0, 255.0, THRESH_BINARY | THRESH_OTSU);
    // connect horizontally oriented regions
    cv::Mat connected;
    morphKernel = cv::getStructuringElement(MORPH_RECT, Size(9, 1));
    cv::morphologyEx(bw, connected, MORPH_CLOSE, morphKernel);
    // find contours
    cv::Mat mask = Mat::zeros(bw.size(), CV_8UC1);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours(connected, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    printf("Found hierarchy = %i\n", hierarchy.size());
    // filter contours
    for(int idx = 0; idx >= 0; idx = hierarchy[idx][0]){
        cv::Rect rect = boundingRect(contours[idx]);
        cv::Mat maskROI(mask, rect);
        maskROI = Scalar(0, 0, 0);
        // fill the contour
        drawContours(mask, contours, idx, Scalar(255, 255, 255), CV_FILLED);

        cv::RotatedRect rrect = minAreaRect(contours[idx]);
        Point2f pts[4];
        rrect.points(pts);
        float r = (float)countNonZero(maskROI) / (rrect.size.width * rrect.size.height);
        if (r > 0.25 && 
        (rrect.size.height > 32 && rrect.size.width > 32) // constraints on region size
        // these two conditions alone are not very robust. better to use something 
        //like the number of significant peaks in a horizontal projection as a third condition
        )
        {
			 std::cout << "found: P1 =" << pts[0] << ", P2 =" << pts[1] << ", P3 =" << pts[2] << ", P4 =" << pts[3] << std::endl;
        } else 
        {

        }
    }

    printf("========================\n\n");

}

