/*************************************************************************
    > File Name: main.cpp
    > Author: Yang Tian, Xiao Wang
    > Email: {tianyang9310, wangxiaonku} @gmail.com 
    > Created Time: Sun 07 Feb 2016 02:03:16 PM EST
 ************************************************************************/

#include<iostream>
#include<vector>
#include<array>
#include<string>
#include"time.h"
#include<opencv2/core/core.hpp>
#include<random>
#include<fstream>
#include"../include/bee-map.h" 
#include"../include/map_visualization.h"
#include"../include/particle_visualization.h"
#include"../include/PF_core.h"
#include"../include/read_log.h"
#include"../include/video_generator.h"

using namespace std;


int num_sample = 5000;

int main()
{
	vector<array<int,2>> empty_position_array;

	map_type map;
	string map_name = "../data/map/wean.dat";
	
	vector<DATA_log> data_log;
	string log_name = "robotdata2";
	string data_log_name = "../data/log/"+log_name+".log";
	string video_name = "../video/"+log_name;

	cv::VideoWriter writer;
	if (!writer.isOpened())
	{
		if (!writer.open(video_name+".mp4", CV_FOURCC('P','I','M','1'), 25.0, cv::Size(800,800), true))
		{
			cout<<"cannot open video writer...."<<endl;
		}
	}

	read_beesoft_map(map_name.c_str(), &map);
    cv::Mat mapImage = map_visualization(map);
	read_data_log(data_log_name.c_str(), &data_log);

	srand((unsigned)time(NULL));

	// find empty positions on the map
	cout<<"Map min x: "<<map.min_x<<endl;
	cout<<"Map max x: "<<map.max_x<<endl;
	cout<<"Map min y: "<<map.min_y<<endl;
	cout<<"Map max y: "<<map.max_y<<endl;
	
	for (int i=0; i<map.size_x; i++)
	{
		for (int j=0; j<map.size_y; j++)
		{
			if (map.prob[i][j] == 1)
			{
				empty_position_array.push_back({i, j});
			}
		}
	}
	int empty_position_size = empty_position_array.size();
	RobotParticle robot;
	vector<RobotParticle> particles(num_sample);

	for (int t=0; t<num_sample; t++)
	{
		particles[t].set(empty_position_array[rand()%empty_position_size]);
	}
    
    vector<double> variance;

    video_generator(writer, particle_visualization(mapImage, particles), video_name);
    double pre_x, pre_y, pre_theta;
    pre_x = data_log[0].x;
    pre_y = data_log[0].y;
    pre_theta = data_log[0].theta;

    ofstream myfile;
    myfile.open("variance5.txt",ios::out | ios::app | ios::binary);
	int data_log_size = data_log.size();
	for (int i =0; i < data_log_size; i++)
	{
		vector<double> weight(num_sample); 
		for (int j = 0; j < num_sample; j++)
		{  
			particles[j].sense(data_log[i]);
		}
		motion control;
		control.delta_x = data_log[i].x - pre_x;
		control.delta_y = data_log[i].y - pre_y;
		
		// robot kidnap               
		if (control.delta_x > 20 || control.delta_y > 20)
		{
			cout<<"change to another log"<<endl;
			cout<<"re-initialze particles"<<endl;
			for (int t=0; t<num_sample; t++)
			{
				particles[t].set(empty_position_array[rand()%empty_position_size]);
			}
			pre_x = data_log[i].x;
			pre_y = data_log[i].y;
			pre_theta = data_log[i].theta;
			control.delta_x = data_log[i].x - pre_x;
			control.delta_y = data_log[i].y - pre_y;
		}

		control.pre_theta = pre_theta;
		control.cur_theta = data_log[i].theta;

		pre_y = data_log[i].y;
		pre_x = data_log[i].x;
		pre_theta = data_log[i].theta;

       double motion_theta = control.cur_theta - control.pre_theta;
       if(motion_theta!=motion_theta)
        {
        	motion_theta = 0;
        }
		
		if(fabs(control.delta_x) < 0.1 && fabs(control.delta_y) < 0.1 && fabs(motion_theta) < 0.01)
		{
			if (data_log[i].data_type == "L" && i%3==0)
			{
				for (int j = 0; j < num_sample; j++)
				{  	
					weight[j] = particles[j].measurement_prob(map);
				}
				double temp = find_var(particles);
				myfile << temp << endl;
				variance.push_back(temp);
				if(temp > 0.05)
				{
				resampling(particles, weight);
			     }
			}
		video_generator(writer, particle_visualization(mapImage, particles), video_name);
			continue;	
		}
		else
		{
			for (int j = 0; j < num_sample; j++)
			{  
				particles[j].move(control,map);
			}
				double temp = find_var(particles);
				myfile << temp << endl;
				cout << temp << endl;
				variance.push_back(temp);

			if (data_log[i].data_type == "L")
			{
				for (int j = 0; j < num_sample; j++)
				{  	
					weight[j] = particles[j].measurement_prob(map);
				}
				double temp = find_var(particles);
				myfile << temp << endl;
				cout << temp << endl;
				if(temp > 0.05)
				{
					resampling(particles, weight);
				}
				variance.push_back(temp);	
			}
		video_generator(writer, particle_visualization(mapImage, particles), video_name);
		}
	    
			
	}

	
	return 0;
}
