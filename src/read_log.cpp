/*************************************************************************
    > File Name: ../src/read_log.cpp
    > Author: Yang Tian, Xiao Wang
    > Email: {tianyang9310, wangxiaonku} @gmail.com 
    > Created Time: Sat 13 Feb 2016 04:46:14 PM EST
 ************************************************************************/

#include<iostream>
#include"../include/read_log.h"

using namespace std;

DATA_log::DATA_log()
{
	//data_type=0;
	x=0;
	y=0;
	theta=0;
	laser_x=0;
	laser_y=0;
	laser_theta=0;
	ts=0;
	// do not initialize range[NUM_RANGE]
}

int read_data_log(const char *data_log_name, vector<DATA_log> *data_log)
{
	ifstream fp(data_log_name);
	if(!fp.is_open())
	{
		fprintf(stderr, "# Could not open file %s\n", data_log_name);
		return -1;
	}
	fprintf(stderr, "# Reading data log: %s\n", data_log_name);

	string data_line;
	data_log->clear();

	while(getline(fp, data_line))
	{
		DATA_log data_log_item;
		istringstream stringin(data_line);

		string buffer;
		vector<string> tokens;
		while(stringin >> buffer)
		{
			tokens.push_back(buffer);
		}
		//divided by 10 because the resolution, the move distane on the map is 1/10 the true dist
		data_log_item.data_type = tokens.at(0).c_str();
		data_log_item.x = stod(tokens.at(1).c_str())/10;
		data_log_item.y = stod(tokens.at(2).c_str())/10;
		data_log_item.theta = stod(tokens.at(3).c_str());
		data_log_item.ts = stod(tokens.back().c_str());
		
		if (tokens.front()[0] == 'L')
		{
			data_log_item.laser_x = stod(tokens.at(4).c_str())/10;
			data_log_item.laser_y = stod(tokens.at(5).c_str())/10;
			data_log_item.laser_theta = stod(tokens.at(6).c_str());
			for (int i = 7; i< 187; i++)
			{
				data_log_item.range[i-7] = stod(tokens.at(i).c_str())/10;
			}
		}
		data_log->push_back(data_log_item);
	}

	fp.close();
	return 1;
}
