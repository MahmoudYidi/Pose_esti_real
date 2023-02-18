#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include "std_msgs/Empty.h"
#include <geometry_msgs/Twist.h>
 

#include <stdlib.h> 
#include <vector>
#include "flyer/flyer.h"

using namespace Eigen;
using namespace std;
typedef Matrix<double, 6, 1> Vector6d;
int main(int argc, char **argv)
	{

ros::init(argc, argv, "flyer_node");
    ros::NodeHandle n;
		flight flight(&n, 20);
		flight.doing();
	
	ros::shutdown();	
		
	}
