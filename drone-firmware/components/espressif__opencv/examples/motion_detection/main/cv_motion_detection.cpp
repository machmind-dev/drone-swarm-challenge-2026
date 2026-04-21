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

extern "C" void cv_motion_detection(uint8_t* data, int width, int heigth);

cv::Mat grayscale1, grayscale2;
cv::Mat absOut, thresholdImg;

extern "C" void cv_print_info();
void cv_print_info()
{
    printf("OpenCV info print: %s", cv::getBuildInformation().c_str());
}

void cv_motion_detection(uint8_t* data, int width, int heigth)
{
    cv::Mat picture( heigth, width, CV_8UC1, (void*)data );

    picture.copyTo(grayscale2);
    if (grayscale2.rows != grayscale1.rows)
    {
	    grayscale2.copyTo(grayscale1);
        thresholdImg = grayscale1;
    }

	absdiff(grayscale1, grayscale2, absOut); //input need to be in grayscale
		
    thresholdImg = grayscale1;
	blur(thresholdImg, thresholdImg, Size(16, 16));	//smooth effect
	threshold(absOut, thresholdImg, 30, 255, THRESH_BINARY);	//only black and white

	erode(thresholdImg, thresholdImg, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));	//noise reduction 
	dilate(thresholdImg, thresholdImg, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));	//noise reduction 

	Moments mome = moments(thresholdImg);
	double dM01 = mome.m01;
	double dM10 = mome.m10;
	double dArea = mome.m00;
    if (dArea > 10)
    {
        int posX = dM10 / dArea;
        int posY = dM01 / dArea;

        if (posX >= 0 && posY >= 0)
        {
                std::cout << "Position detected: " << Point(posX, posY) << std::endl;
        }
    }
    grayscale2.copyTo(grayscale1);
}

