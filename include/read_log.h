/*************************************************************************
    > File Name: read_log.h
    > Author: Yang Tian
    > Email: tianyang9310@gmail.com 
    > Created Time: Sat 13 Feb 2016 04:37:03 PM EST
 ************************************************************************/

#ifndef READ_LOG_H
#define READ_LOG_H

#include<iostream>
#include<vector>
#include<string>
#include<fstream>
#include<sstream>
#include<stdlib.h>
#define NUM_RANGE 180
using namespace std;


class DATA_log
{
public:
	DATA_log();
	string data_type;
	double x;
	double y;
	double theta;
	double laser_x;
	double laser_y;
	double laser_theta;
	double ts;
	double range[NUM_RANGE];
};

int read_data_log(const char *data_log_name, vector<DATA_log> *data_log);


#endif
