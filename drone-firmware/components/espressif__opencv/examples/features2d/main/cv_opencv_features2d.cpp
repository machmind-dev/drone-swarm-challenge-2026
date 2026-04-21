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
#include <opencv2/imgcodecs.hpp>

#include <opencv2/core/utility.hpp>
#include "opencv2/opencv_modules.hpp"

#include "opencv2/core.hpp"
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/core/softfloat.hpp>
#include <opencv2/imgproc/types_c.h>

#include <iostream>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using namespace std;
using namespace cv;

extern "C" long int sysconf(int);

long int sysconf(int wtf)
{
    return 1;
}

extern "C" void init_opencv_features2d();
extern "C" void test_opencv_features2d(uint8_t* data, int width, int heigth);
cv::Ptr<cv::ORB> orb_detector;
std::vector<cv::KeyPoint> mvKeys;
cv::Mat mDescriptors;

void init_opencv_features2d()
{
    printf("OpenCV info print: %s", cv::getBuildInformation().c_str());
    orb_detector = cv::ORB::create();
    orb_detector->setFastThreshold(29);
    orb_detector->setNLevels(1);
    orb_detector->setMaxFeatures(40);
    orb_detector->setScaleFactor(1.2);
    orb_detector->setEdgeThreshold(35);
    orb_detector->setScoreType(cv::ORB::ScoreType::FAST_SCORE);
    orb_detector->setFirstLevel(0);

}

void test_opencv_features2d(uint8_t* data, int width, int heigth)
{
    printf("Extract 2d features from picture %ix%i\n", width, heigth);
    cv::Mat rawData( heigth, width, CV_8UC1, (void*)data );
    
	orb_detector->detect(rawData, mvKeys);
    orb_detector->compute(rawData, mvKeys, mDescriptors);
    printf("For picture: mvKeys.size = %i\n", mvKeys.size());
    for (int i=0 ; i< mvKeys.size() ; i++)
    {
        printf("mvKeys[%2i] x=%3.2f, y=%3.2f \n", i, mvKeys[i].pt.x, mvKeys[i].pt.y);
    }
    printf("========================\n\n");
}

