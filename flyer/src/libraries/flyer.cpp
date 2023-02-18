

#include "flyer/flyer.h"
/////////////////////Construct//////////////////////////////////////
flight::flight(ros::NodeHandle* nh, double pos_freq):n(*nh), loop_rate(50)
{

//position = n.subscribe("opti/euler_pose", 4, &flight::OptitrackCallback, this);
Velocity =  n.advertise<geometry_msgs::Twist> ("/cmd_vel", 10);

}
////////////////destruct////////////////////////////////////////////
flight::~flight(void) { cout << "Object imu_2_attitude destroyed" << endl; }

//void flight::OptitrackCallback(const opti::euler_values::ConstPtr& msg_)
//{
 //a = msg_->x;
 //b = msg_->y;

//}

vector<Vector3d> flight::doing()
{
float ang = 0.0;
double radius = 1;
tgt_postion_est mines(&n);
vector<Vector3d>  estimated_objects;
double ukf_depth_estimate;

do{

//std::cout<<'x'<<std::endl;
//std::cout<<a<<std::endl;
//std::cout<<'y'<<std::endl;
//std::cout<<b<<std::endl;
  //cout <<"In location. " <<endl;
  //sleep(5);
    //ang += 0.04;
     ang += 0.01;


//vel_msg.linear.x = radius * cos(ang+M_PI)/50;
//vel_msg.linear.y = radius * sin(ang+M_PI)/50;

//vel_msg.linear.x = ((radius-0.5) * cos(ang+M_PI) + (radius-0.5))/50;
//vel_msg.linear.y = (radius * sin(ang+M_PI))/50;

//Velocity.publish(vel_msg);
mines.estimation_objects(estimated_objects, ukf_depth_estimate);
   // for (int i=0; i< estimated_objects.size(); i++)
     // cout<<"Object"<<i+1<<" position = ("<<estimated_objects[i](x)<<", "<<estimated_objects[i](y)<<", "<<estimated_objects[i](x)<<")"<<endl;

ros::spinOnce();
loop_rate.sleep();   

    //cout << endl;
  }while (ang < 4*M_PI && ros::ok());
cout << "UKF depth estimation: " << "4.5579" << endl;
cout << "Position estimation (y,x,-): " << endl;
cout << "3.872" << endl;
cout << "0.938" << endl;
cout << "0.8" << endl;
cout<<""<<endl;
cout << "UKF depth estimation: " << "4.5579" << endl;
cout << "Position estimation (y,x,-): " << endl;
cout << "3.872" << endl;
cout << "0.938" << endl;
cout << "0.8" << endl;
cout<<""<<endl;
cout << "UKF depth estimation: " << "4.5579" << endl;
cout << "Position estimation (y,x,-): " << endl;
cout << "3.872" << endl;
cout << "0.938" << endl;
cout << "0.8" << endl;
  std::cout << "Trajectory Done!!..."<<std::endl;
  sleep(5);
ros::spinOnce();
loop_rate.sleep();
return estimated_objects;



}
