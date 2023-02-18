#ifndef GET_H
#define GET_H

#include "gazebo_msgs/ModelStates.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "ros/ros.h"
#include <iostream>
#include <math.h>

#include <chrono>
#include <random>
#include "std_msgs/Empty.h"
#include <geometry_msgs/Twist.h> 
#include <stdlib.h> 
#include <vector>
#include "tf/transform_datatypes.h"
#include "opti/euler_values.h"
#include <eigen3/Eigen/Dense>
using namespace std;
using namespace Eigen;

class getter{
private:
	ros::NodeHandle n;
	ros::Subscriber position_opti;
        ros::Subscriber second_pos;
	ros::Publisher Eulers;
	ros::Rate loop_rate;

	//geometry_msgs::Twist vel_msg;
	//double x;
	//double y;
 double roll_, pitch_, yaw_;

public:
opti::euler_values pose_euler_msg;
//nav_msgs::Odometry msg;
geometry_msgs::PoseStamped msg;
sensor_msgs::Imu msg1;

	double x;
	double y;
getter(ros::NodeHandle* nh, double op_freq);
virtual ~getter(void);
//void OptiCall(const nav_msgs::Odometry::ConstPtr&); 
void OptiCall(const geometry_msgs::PoseStamped::ConstPtr&);  
void caller(const sensor_msgs::Imu::ConstPtr&);
};
#endif
