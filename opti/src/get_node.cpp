#include <iostream>
#include <ros/ros.h>
#include "opti/get.h"
#include <stdlib.h>

int main (int argc, char **argv)
{
    
    ros::init(argc, argv, "opti");
    ros::NodeHandle n;
	getter getter(&n, 20);
	
 while(ros::ok())
        {


ros::spinOnce();
        }

return 0;
}
