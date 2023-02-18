 #include "tgt_pstion/target_position_estimation.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
tgt_postion_est::tgt_postion_est(ros::NodeHandle* nh): nh_(*nh), first_detection(1), ab(1), attitude(0,0,0), pstion(0,0,0,1), x(0), y(1), z(2),  cam_x(0), cam_y(0),
                                    camera_offset(0.21, 0, -0.04), focal_length(561.885)// real_platform_values camera(200wC 666.66 pixels) camera(202bC 1066.66 pixels)
{
image_sub_ = nh_.subscribe("/center_pixel", 1, &tgt_postion_est::extract, this);
angles_sub_ = nh_.subscribe("/opti/euler_pose" ,1 ,&tgt_postion_est::angles_and_position, this);

azimut_elevation_pub = nh_.advertise<opti::azimut_elevation>("/tgt_pstion/azimut_elevation_angles",1);
estimation_pub = nh_.advertise<geometry_msgs::Point>("/tgt_pstion/xyz_estimation_position",1);
c=0;  
drone2world.setZero();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
tgt_postion_est::~tgt_postion_est()
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double tgt_postion_est::mod(double a, double m)
{
  return a - m*floor(a/m);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void tgt_postion_est::extract(const detection_network::pixel::ConstPtr& msg_)
{
 cam_x = msg_->x;
 cam_y = msg_->y;
 //cam_x = 1 ;
 //cam_y = 1 ;
 

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void tgt_postion_est::angles_and_position(const opti::euler_values::ConstPtr& angles)
{
  attitude << angles->roll, angles->pitch, angles->yaw;
  pstion   << angles->x, angles->y, angles->z, 1.0;
  drone2world.block(0,0,3,3) = RPY_ROT(angles->roll, angles->pitch, angles->yaw);
  drone2world.block(0,3,4,1) = Eigen::Vector4d(angles->x,angles->y, angles->z, 1);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void tgt_postion_est::estimation_objects(vector<Vector3d>& Xhat, double& depth_est)
{
  
  Vector3d x_hat(0,0,0);
  
    if (first_detection == 1)
    {
      
        int tracking_objs = 1;
        estimator.resize(tracking_objs);
        Xhat.resize(tracking_objs);
        meas.resize(tracking_objs);
        measa.resize(tracking_objs);
        cout << "Objects detected :" << tracking_objs << endl;
        first_detection = 0;
        
      return;
    }
    else
    {
	for (int i=0; i<1; i++)
      {
          double u0 = 376.5;
          double v0 = 240.5;
          double rho= 6E-6;
          double fl = 0.0035;    
          MatrixXd K;   
          K.setZero(3,3);      
          K << 1.0/rho,      0,  u0,
                      0,1.0/rho,  v0,
                      0,      0, 1.0;

          Matrix4d cam2drone;
          cam2drone << 0, 0, 1, 0.00,
                      -1, 0, 0, 0.00,
                       0,-1, 0, 0.00,
                       0, 0, 0, 1.00;

          Matrix4d target2cam;
          target2cam <<-1, 0, 0, 0,
                        0,-1, 0, 0,
                        0, 0, 1, 0,
                        0, 0, 0, 1;

          Vector3d uvw_point;
  	//std::cout<<x_<<endl;
	//std::cout<<y_<<endl;


/////////////////////////////////////////////
double xa = cam_x + 112;
double ya = 112 - cam_y;

////////////////////////////////////////////
          uvw_point << xa, ya, 1.0;
          //uvw_point << 120, 109, 1.0;
         std::cout<<uvw_point<<std::endl;


        
          uvw_point << K.inverse() * uvw_point;
          
          Vector4d uvw_homogeneous(uvw_point(0), uvw_point(1), fl, 1.0);
          uvw_homogeneous << drone2world * cam2drone * uvw_homogeneous;
         // std::cout<<"pstion"<<uvw_homogeneous <<endl;
          uvw_homogeneous << pstion + uvw_homogeneous;
/////////////////////////////////////////////////////////////////////////////////////////////////
         double beta = atan(uvw_homogeneous(1) / uvw_homogeneous(0));
          double alpha = atan(uvw_homogeneous(2) / sqrt(pow(uvw_homogeneous(0),2) + pow(uvw_homogeneous(1),2)));
          // std::cout<<"attitude"<<attitude<<endl;
           //std::cout<<"pstion"<<pstion<<endl;
          meas[i] << beta, alpha;
          measa[0] << 0.368776, -0.203532;
	//std::cout<<"meas"<<meas[i]<<endl;
	//std::cout<<"mease1"<<meas[i+1]<<endl;
	//std::cout<<"mease1"<<meas[]<<endl;
          Vector3d position(pstion(0),pstion(1),pstion(2));
          position = position - camera_offset;
        // std::cout<<"position"<<position<<endl;
	 //std::cout<<position(1)<<endl;
          if (ab<15){x_hat = estimator_2.kalman_unscented_estimator(position, measa[0]);

		ab += 1;
             
		}

	else{
           
          x_hat = estimator_2.kalman_unscented_estimator(position, meas[i]);
          //std::cout<<"Xhat"<<x_hat<<endl;
	  //Xhat[i] = estimator[i].kalman_unscented_estimator(position, meas[i]);
	   //std::cout<<Xhat[1]<<endl;
          //std::cout<<position<<endl;
          Vector3d depth(x_hat - position);
          cout << "UKF depth estimation: " << depth.norm() << endl;
	  cout << "Position estimation (x,y,-): " << endl << x_hat<< endl << endl;
         }
          //depth_est = depth.norm();
          //Xhat[i] = estimator[i].kalman_extended_estimator(position, meas[i]) + camera_offset;
          // tgt_position_.x = Xhat[i](x);
          // tgt_position_.y = Xhat[i](y);
          // tgt_position_.z = Xhat[i](z);
          // //tgt_position_.samples = i;
          // estimation_pub.publish(tgt_position_)
        }

}
        
     

  return;

}

