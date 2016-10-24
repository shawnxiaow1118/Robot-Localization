/*************************************************************************
    > File Name: map_visualization.cpp
    > Author: Yang Tian, Xiao Wang
    > Email: {tianyang9310, wangxiaonku} @gmail.com 
    > Created Time: Sun 07 Feb 2016 03:25:36 PM EST
 ************************************************************************/

#include"../include/map_visualization.h"

cv::Mat map_visualization(map_type map)
{
	map_type tmp_map = map;
	cv::Mat _mapImage = cv::Mat::zeros(tmp_map.size_x, tmp_map.size_y, CV_8UC1);
	for (int i = 0; i<_mapImage.rows; i++)
	{
		for (int j = 0; j<_mapImage.cols; j++)
		{
			if (tmp_map.prob[i][j]<=0)
			{
				_mapImage.at<uint8_t>(i,j) = 0;
			}
			else
			{
				_mapImage.at<uint8_t>(i,j) = 255*tmp_map.prob[i][j];
			}
		}
	}
	cv::Mat result_image;
	cv::cvtColor(_mapImage, result_image, CV_GRAY2RGB);
	result_image = result_image.t();
	cv::imshow("Image", result_image);
	cv::waitKey(30);
	cv::imwrite("Map_Loading.png", result_image);
	return _mapImage;
}
