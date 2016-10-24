/*************************************************************************
    > File Name: ../include/video_generator.h
    > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Fri 19 Feb 2016 10:25:12 PM EST
 ************************************************************************/

#ifndef VIDEO_GENERATOR_H
#include<iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
using namespace std;

void video_generator(cv::VideoWriter writer, cv::Mat frame, const string video_name);

#endif
