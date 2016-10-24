/*************************************************************************
    > File Name: PF_core.cpp
    > Author: Yang Tian
    > Email: tianyang9310@gmail.com 
    > Created Time: Sat 13 Feb 2016 11:23:32 AM EST
 ************************************************************************/

#include<iostream>
#include<array>
#include<time.h>
#include<math.h>
#include<random>
#include"../include/PF_core.h"
#define PI 3.141592653
#define NUM_RANGES 180

using namespace std;

extern int num_sample;

RobotParticle::RobotParticle():_motion_noise(0),_sense_noise(0)
{
	_theta = rand()/double(RAND_MAX)*2*PI;
    _threshold = 0.9;
    z_hit = 0.9;
    z_short = 0;
    z_max = 0.0;
    z_rand = 0.1;
    //adjust sigma can control the probbility
    sigma = 9;
    max_laser_range = 800;
    lambda = 0.0005;
    alpha_1 = 0.002;
    alpha_2 = 0.002;
    alpha_3 = 0.002;
    alpha_4 = 0.002;
}

void RobotParticle::set_noise(double motion_noise, double sense_noise)
{
	_motion_noise = motion_noise;
	_sense_noise = sense_noise;
}

void RobotParticle::set(array<int, 2> empty_pair)
{
	_x = empty_pair[0];
	_y = empty_pair[1];
	_theta = rand()/double(RAND_MAX)*2*PI;
}
//motion model on the book
void RobotParticle::move(motion control,map_type map)
{
	double delta_theta = control.cur_theta - control.pre_theta;
	//handler the nan issue
	if(delta_theta != delta_theta)
        {
        	delta_theta = 0;
        }

	double rot1 = atan2(control.delta_y, control.delta_x) - control.pre_theta;
	double trans = sqrt(pow(control.delta_x,2) + pow(control.delta_y,2));
	double rot2 = control.cur_theta - control.pre_theta - rot1;

	double n_x, n_y, n_theta;

	double rot1_hat = rot1 - sample()*sqrt((alpha_1*pow(rot1,2) + alpha_2*pow(trans,2)));
	double trans_hat = trans - sample()*sqrt((alpha_3*pow(trans,2) + alpha_4*pow(rot1,2) + alpha_4*pow(rot2,2)));
	double rot2_hat = rot2 - sample()*sqrt((alpha_1*pow(rot2,2) + alpha_2*pow(trans,2)));
	//new possible position
	n_x = _x + trans_hat*cos(_theta + rot1_hat);
	n_y = _y + trans_hat*sin(_theta + rot1_hat);
	n_theta = (_theta + rot1_hat + rot2_hat);
	//check whether in the area
	if (map.prob[(int)(n_x+0.5)][(int)(n_y+0.5)] > 0.8)
	{
		_x = n_x;
		_y = n_y; 
		_theta = n_theta;
	}
 }

//read log data
void RobotParticle::sense(DATA_log data)
{
	if (data.data_type == "L")
	{
		for (int i = 0; i < NUM_RANGES; i++)
		{
			laser[i] = data.range[i];
		}
	}
}

//measurement model need take care of this
double RobotParticle::measurement_prob(map_type map)
{
	
	double laser_offset = 25.0/10.0;
	double laser_x = _x + laser_offset*cos(_theta);
	double laser_y = _y + laser_offset*sin(_theta);
	if (laser_x >= map.max_x - 0.001|| laser_x <= map.min_x || laser_y >= map.max_y-0.001 || laser_y <= map.min_y)
	{
		return 0;
	}
	double prob = map.prob[(int)laser_x][(int)laser_y];

	if (prob < _threshold)
	{
		return 0;		
	}
	//dist vector in case to find the best k's probability
	double p_prob = 1.0;
	for (int i = 0; i < NUM_RANGES; i+=10)
	{
		double laser_theta = _theta + ((double)i)*PI/180 - PI/2;
		double x_end = laser_x + (int)laser[i]*cos(laser_theta);
		double y_end = laser_y + (int)laser[i]*sin(laser_theta);
		double dist = find_dist(laser_x, laser_y, laser_theta,laser[i], map);
		double p_i = 1/12000.0;
		if(laser[i] > 0 && laser[i] < max_laser_range && dist < max_laser_range && dist > 0)
		{
			double p_hit   = Gaussian(dist, dist/1.5 + 2, laser[i]);
			double p_short = Short_noise(laser[i], dist);
			double p_max   = Max_noise(laser[i]);
			double p_rand  = Rand_noise();
			p_i = z_hit*p_hit + z_short*p_short + z_max*p_max + z_rand*p_max;
		}
		else if(laser[i] >= max_laser_range && dist < max_laser_range && dist > 0)
		{
			double p_hit   = Gaussian(dist, dist/1.5 + 2, max_laser_range);
			double p_short = Short_noise(max_laser_range, dist);
			double p_max   = Max_noise(max_laser_range);
			double p_rand  = Rand_noise();
			p_i = z_hit*p_hit + z_short*p_short + z_max*p_max + z_rand*p_max;
			p_i = z_hit*p_hit + z_short*p_short + z_max*p_max + z_rand*p_max;			
		} 

        p_prob = p_prob*p_i;
    }

    //int len = (int)sizeof(p_particle)/sizeof(double);
    //heap sort in order to find the first k best prob.
	//heap_sort(p_particle,len);
	//double p_p = 1;
	//can asjust 100 to 0-179 to different num of point.
	//for (int j = 0; j < 20; j++)
	//{
	//	p_p = p_p*p_particle[j];
	//}
	//cout << p_particle[0] << endl;
	return p_prob;
}
// for a particle and heading find the expected distance
double RobotParticle::find_dist(double laser_x, double laser_y,double laser_theta, double laser_data, map_type map)
{

	double dx = 0.5*cos(laser_theta);
	double dy = 0.5*sin(laser_theta);
	double r = 0.5; 

	double prob = 1.0;
	while(prob > 0.5 && r < max_laser_range)
	{
		laser_x += dx;
		laser_y += dy;
		r += 0.5;

		if(laser_x<0)
		{
			laser_x = 0;
		}
		else if(laser_x > map.max_x - 0.001)
		{
			laser_x = map.max_x - 1;
		}

		if(laser_y < 0)
		{
			laser_y = 0;
		}
		else if(laser_y > map.max_y -0.001)
		{
			laser_y = map.max_y - 1;
		}

		prob = map.prob[(int)(laser_x+0.5)][(int)(laser_y+0.5)];
	}

	if(r < 2)
	{
		r = -1;
	}
	return r;
}

//fcuntions calculate the prob 
double RobotParticle::Gaussian(double mu, double sigma, double x)
{
	double prob = exp(-pow(mu-x,2)/(pow(sigma,2)*2.0))/(sqrt(2.0*PI)*sigma); 
    return prob;
}

double RobotParticle::Rand_noise()
{
	return 1.0/max_laser_range;
}

double RobotParticle::Max_noise(double laser_data)
{
	return (fabs(laser_data - max_laser_range) <= 0.001 ? 0:1);
}

double RobotParticle::Short_noise(double laser_data, double exp_data)
{
	double z = 1./(1 - exp(-lambda*laser_data));
	return  (lambda*z*exp(-lambda*laser_data));
}



void RobotParticle::print()
{
	cout<<"x = "<<_x<<", y = "<<_y<<", theta = "<<_theta<<endl;
}

int RobotParticle::getX()
{
	return _x;
}

int RobotParticle::getY()
{
	return _y;
}

double RobotParticle::getTheta()
{
	return _theta;
}



double sample()
{
	/*
    static double V1, V2, S;
    static int phase = 0;
    double X;
     
    if ( phase == 0 ) {
        do {
            double U1 = (double)rand() / RAND_MAX;
            double U2 = (double)rand() / RAND_MAX;
             
            V1 = 2 * U1 - 1;
            V2 = 2 * U2 - 1;
            S = V1 * V1 + V2 * V2;
        } while(S >= 1 || S == 0);
         
        X = V1 * sqrt(-2 * log(S) / S);
    } else
        X = V2 * sqrt(-2 * log(S) / S);
         
    phase = 1 - phase;
 
    return X;
	*/
	
	static default_random_engine generator;
	static normal_distribution<double> my_normal_distribution(0.0, 1.0);
	double sample_result = my_normal_distribution(generator);
	return sample_result;
}


void resampling(vector<RobotParticle> &old_particles, vector<double> weight)
{
	// normalize the weight vector
	double sum = accumulate(weight.begin(), weight.end(), 0.0);
	
	/*
	//naive summation
	double sum = 0;
	for (int k = 0; k < num_sample;k++)
	{
		sum = sum + weight[k];
	}	
	*/
	for (int i = 0; i<weight.size(); i++)
	{
		weight[i]=weight[i]/sum;
	}

//	for (int i = 0; i<weight.size(); i++)
//	{
//		cout<<weight[i]<<" ";
//	}
//	cout<<endl;


	// wheel resampling
	int particle_size = old_particles.size();
	double beta = 0;

	double weight_max = *max_element(weight.begin(), weight.end());
	//cout << weight_max << " ";
	static default_random_engine generator;
	static uniform_real_distribution<double> weight_distribution(0.0, 2*weight_max);

	int weight_index = rand()%particle_size;

	vector<RobotParticle> new_particles;
	for (int i = 0; i<particle_size; i++)
	{
		beta = beta + weight_distribution(generator);	
		while (beta>weight[weight_index])
		{
			beta = beta - weight[weight_index];
			weight_index = (weight_index + 1)%particle_size;
		}
		new_particles.push_back(old_particles[weight_index]);
	}
	old_particles = new_particles;
}

//traditional heap adjust
void max_heapify(vector<double> arr, int start, int end)
{
	int dad = start;
	int son = dad*2 + 1;
	while(son < end)
	{
		if(son+1 < end && arr[son] < arr[son + 1])
		{
			son++;
		}
		if(arr[dad] > arr[son])
		{
			return;
		}
		else
		{
			swap(arr[dad], arr[son]);
			dad = son;
			son = dad*2 + 1;
		}
	}
}

//traditional heap sort
void heap_sort(vector<double> arr, int len)
{
	for(int i = len/2-1; i >= 0; i--)
		max_heapify(arr, i, len);
	for (int i = len - 1; i > 0;i--)
	{
		swap(arr[0],arr[i]);
		max_heapify(arr,0,i);
	}
}

double find_var(vector<RobotParticle> particles)
{
	vector<int> X;
	vector<int> Y;
	vector<double> Theta;
	for (int i = 0; i < num_sample; i++)
	{
		X.push_back(particles[i].getX());
		Y.push_back(particles[i].getY());
		Theta.push_back(particles[i].getTheta());
	}

	int sum_x = accumulate(X.begin(), X.end(), 0);
	int sum_y = accumulate(Y.begin(), Y.end(), 0);
	double sum_theta = accumulate(Theta.begin(),Theta.end(),0.0);
	double mean_x = (double(sum_x))/num_sample;
	double mean_y = (double(sum_y))/num_sample;
	double mean_theta = (double(sum_theta))/num_sample;
	double s_x,s_y,s_theta;
	for (int j = 0; j < num_sample; j++)
	{
		s_x += pow(X[j]-mean_x,2);
		s_y += pow(Y[j]-mean_y,2);
		s_theta += pow(Theta[j]-mean_theta,2);
	}
	double var_x = s_x/(num_sample-1);
	double var_y = s_y/(num_sample-1);
	double var_theta = s_theta/(num_sample-1);
	double var = (var_x + var_y + var_theta)/3;
	return var;	
}

