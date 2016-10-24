/*************************************************************************
    > File Name: video_generator.cpp
    > Author: Yang Tian, Xiao Wang
    > Mail: {tianyang9310, wangxiaonku} @gmail.com 
    > Created Time: Fri 19 Feb 2016 10:23:06 PM EST
 ************************************************************************/

#include<iostream>
#include"../include/video_generator.h"
using namespace std;
using namespace cv;

void video_generator(cv::VideoWriter writer, cv::Mat frame, const string video_name)
{
	writer.write(frame);
	waitKey(1);
}
