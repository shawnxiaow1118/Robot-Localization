/*************************************************************************
    > File Name: ../include/particle_visualization.h
    > Author: Yang Tian
    > Email: tianyang9310@gmail.com 
    > Created Time: Sat 13 Feb 2016 01:38:13 PM EST
 ************************************************************************/

#ifndef PARTICLE_VISUALIZATION_H
#define PARTICLE_VISUALIZATION_H
#include<iostream>
#include<vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include"PF_core.h"
using namespace std;

cv::Mat particle_visualization(cv::Mat _mapImage, vector<RobotParticle> particles);

#endif
