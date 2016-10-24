/*************************************************************************
    > File Name: ../include/PF_core.h
    > Author: Yang Tian
    > Email: tianyang9310@gmail.com 
    > Created Time: Sat 13 Feb 2016 11:32:24 AM EST
 ************************************************************************/

#include <iostream>
#include <array>
#include <vector>
#include <numeric>
#include <algorithm>
#include <random>
#include <math.h>
#include <chrono>
#include <cmath>
#include"../include/read_log.h"
#include"../include/bee-map.h"
#define NUM_RANGES 180
using namespace std;

#ifndef PF_CORE_H
#define PF_CORE_H
//new struct to contails the motion information
typedef struct motion {
	double delta_x;
	double delta_y;
	double pre_theta;
	double cur_theta;
} motion;

class RobotParticle
{
public:
	RobotParticle();
	void set_noise(double motion_noise, double sense_noise);
	void set(array<int, 2> empty_pair);
	void move(motion control,map_type map);

	void sense(DATA_log data);
	double measurement_prob(map_type map);
	double find_dist(double laser_x, double laser_y, double laser_theta,double laser_data, map_type map);
	void print();
	int getX();
	int getY();
	double getTheta();
	double laser[NUM_RANGES];

protected:
	double _x;
	double _y;
	double _theta;
	double _motion_noise;
	double _sense_noise;
	//some parameters
    int num_sample;
    double _threshold;
    double z_hit;
    double z_short;
    double z_max;
    double z_rand;
    double sigma;
    double max_laser_range;
    double lambda;
    double alpha_1;
    double alpha_2;
    double alpha_3;
    double alpha_4;
    double Short_noise(double laser_data, double exp_data);
    double Gaussian(double mu, double sigma, double x);
    double Max_noise(double laser_data);
    double Rand_noise();
};

void resampling(vector<RobotParticle> &old_particles, vector<double> weight);
double sample();
void max_heapify(vector<double> arr, int start, int end);
void heap_sort(vector<double> arr, int len);
double find_var(vector<RobotParticle> particles);

#endif
