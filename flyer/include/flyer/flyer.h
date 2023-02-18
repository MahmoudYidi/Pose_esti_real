#ifndef FLYER_H
#define FLYER_H

#include "opti/euler_values.h"
#include "ros/ros.h"
#include "math.h"
#include <iostream>
#include <eigen3/Eigen/Dense>
#include "std_msgs/Empty.h"
#include <geometry_msgs/Twist.h> 
#include "tgt_pstion/target_position_estimation.h"
#include <stdlib.h> 
#include <vector>


using namespace std;
using namespace Eigen;
const char x=0;
const char y=1;
const char z=2;
typedef Matrix<double, 6, 1> Vector6d;


class flight {
private:
	ros::NodeHandle n;
	ros::Subscriber position;
	ros::Publisher Velocity;
	ros::Rate loop_rate;

	//geometry_msgs::Twist vel_msg;
	//double x;
	//double y;

public:
geometry_msgs::Twist vel_msg;
	double a;
	double b;
flight(ros::NodeHandle* nh, double pos_freq);
virtual ~flight(void);
//void OptitrackCallback(const opti::euler_values::ConstPtr&);  
vector<Vector3d> doing();
};
#endif
