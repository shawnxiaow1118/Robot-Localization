/*************************************************************************
    > File Name: particle_visualization.cpp
    > Author: Yang Tian, Xiao Wang
    > Email: {tianyang9310, wangxiaonku} @gmail.com 
    > Created Time: Sat 13 Feb 2016 01:38:01 PM EST
 ************************************************************************/

#include"../include/particle_visualization.h"
using namespace std;

cv::Mat particle_visualization(cv::Mat _mapImage, vector<RobotParticle> particles)
{
	int num_sample = particles.size();
	cv::Mat result_image;
	cv::cvtColor(_mapImage, result_image, CV_GRAY2RGB);
	for (int i=0; i<num_sample; i++)
	{
		int x = particles[i].getX();
		int y = particles[i].getY();
		double theta = particles[i].getTheta();
		cv::circle(result_image, cv::Point(y,x), 2,cv::Scalar(0, 0, 255));
	}
	result_image = result_image.t();
	cv::imshow("Image", result_image);
	cv::waitKey(1);
	cv::imwrite("Map_and_Particles_init.png", result_image);
	return result_image;
}
