#ifndef TARGET_POSITION_ESTIMATION_H
#define TARGET_POSITION_ESTIMATION_H

#include <ros/ros.h>
#include <iostream>
///////////////////////////////////////////
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>
#include <tgt_pstion/tgt_kalman_filters.h>
#include "opti/azimut_elevation.h"
#include "opti/euler_values.h"
#include "detection_network/pixel.h"
#include <geometry_msgs/Point.h>
#include "opti/data_types.h"

using namespace std;
using namespace Eigen;

class tgt_postion_est
{
private:
  ros::NodeHandle nh_;
  
  ros::Subscriber image_sub_;
  
  
  ros::Publisher azimut_elevation_pub;
  ros::Subscriber angles_sub_;
  ros::Publisher estimation_pub;

  opti::azimut_elevation angles_objects;
  geometry_msgs::Point tgt_position_;


  vector<tgt_kalman_filters> estimator;
  tgt_kalman_filters estimator_2;

  double cam_x;
  double cam_y;

  string name_;
  int c;

  int first_detection;
 int ab;
  Vector3d attitude;
  Vector4d pstion;

  vector<Vector2d> meas;
 vector<Vector2d> measa;

  const double focal_length;
  const int x,y,z;
  const Vector3d camera_offset;
  Matrix4d drone2world;

public:
  tgt_postion_est(ros::NodeHandle* nh);
  ~tgt_postion_est();
  void estimation_objects(vector<Vector3d>& Xhat, double& depth_est);
  void angles_and_position(const opti::euler_values::ConstPtr& );
  double mod(double a, double m);
  void extract(const detection_network::pixel::ConstPtr& );
};

#endif // TARGET_POSITION_ESTIMATION_H

