#include "opti/get.h"
/////////////////////Construct//////////////////////////////////////
getter::getter(ros::NodeHandle* nh, double op_freq):n(*nh), loop_rate(200)
{

position_opti = n.subscribe("/vrpn_client_node/RigidBody/pose", 4, &getter::OptiCall, this);
//position_opti = n.subscribe("/ardrone/odometry", 4, &getter::OptiCall, this);
Eulers =  n.advertise<opti::euler_values>("opti/euler_pose",1);
///
second_pos = n.subscribe("/ardrone/imu", 4, &getter::caller, this);
////

}
////////////////destruct////////////////////////////////////////////
getter::~getter(void) { cout << "Object imu_2_attitude destroyed" << endl; }
/////////////////////////////////////////////////////////////////////
void getter::OptiCall(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
tf::Quaternion q_;

int model_no =1;
tf::quaternionMsgToTF(msg->pose.orientation, q_);
tf::Matrix3x3(q_.normalize()).getRPY(roll_, pitch_, yaw_);
pose_euler_msg.header.stamp = ros::Time::now();
pose_euler_msg.roll       = roll_;
pose_euler_msg.pitch      = pitch_;
pose_euler_msg.yaw        = yaw_;
///////////////////////////////////////////
//pose_euler_msg.roll_rate  = msg->pose.orientation.x;
//pose_euler_msg.pitch_rate = msg->pose.orientation.y;
//pose_euler_msg.yaw_rate   = msg->pose.orientation.z;
//////////////////////////////////////////////
pose_euler_msg.x = msg->pose.position.x;
pose_euler_msg.y = msg->pose.position.z;
pose_euler_msg.z = msg->pose.position.y;
//pose_euler_msg.x_rate = msg->linear.x;
//pose_euler_msg.y_rate = msg->linear.y;
//pose_euler_msg.z_rate = msg->linear.z;

Eulers.publish(pose_euler_msg);
ros::spinOnce();
loop_rate.sleep();
}


/*

void getter::OptiCall(const nav_msgs::Odometry::ConstPtr& msg)
{
tf::Quaternion q_;

int model_no =1;
tf::quaternionMsgToTF(msg->pose.pose.orientation, q_);
tf::Matrix3x3(q_.normalize()).getRPY(roll_, pitch_, yaw_);
pose_euler_msg.header.stamp = ros::Time::now();
pose_euler_msg.roll       = roll_;
pose_euler_msg.pitch      = pitch_;
pose_euler_msg.yaw        = yaw_;
//pose_euler_msg.roll_rate  = msg->pose.orientation.x;
//pose_euler_msg.pitch_rate = msg->pose.orientation.y;
//pose_euler_msg.yaw_rate   = msg->pose.orientation.z;

pose_euler_msg.x = msg->pose.pose.position.x;
pose_euler_msg.y = msg->pose.pose.position.y;
pose_euler_msg.z = msg->pose.pose.position.z;
//pose_euler_msg.x_rate = msg->linear.x;
//pose_euler_msg.y_rate = msg->linear.y;
//pose_euler_msg.z_rate = msg->linear.z;

Eulers.publish(pose_euler_msg);
ros::spinOnce();
loop_rate.sleep();
}

*/
void getter::caller(const sensor_msgs::Imu::ConstPtr& msg1)
{
pose_euler_msg.roll_rate  = msg1->angular_velocity.x;
pose_euler_msg.pitch_rate = msg1->angular_velocity.y;
pose_euler_msg.yaw_rate   = msg1->angular_velocity.z;

Eulers.publish(pose_euler_msg);
ros::spinOnce();
loop_rate.sleep();
}
