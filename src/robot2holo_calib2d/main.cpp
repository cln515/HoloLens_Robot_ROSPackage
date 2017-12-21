#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <boost/concept_check.hpp>
#include <termios.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include<fstream>

#include "calibrator.h"


using namespace Eigen;

int main(int argc, char **argv)
{
	if(argc<3){
		std::cout<<"Usage: robot2holo_calib <robot's odom frame name> <robot's frame name linking to hololens> <robot's foot frame name> <optional: calib file>"<<std::endl;
		return 0;
		
	}
	std::string odomFrame(argv[1]);
	std::string holoLinkedFrame(argv[2]);	
	std::string robotFootFrame(argv[3]);
    std::string fpath;
    std::cout<<argc<<" "<<odomFrame<<","<<holoLinkedFrame<<","<<robotFootFrame<<std::endl;
    if(argc>=5){
        fpath=std::string(argv[4]);
        std::cout<<fpath<<std::endl;
    }
    
    ros::init(argc, argv, "robot2holo_calib");

    std::cout<<"pepper head-hololens calibration"<<std::endl
      <<"example steps"<<std::endl
      <<"space: record first position"<<std::endl
      <<"move forward and change robots pitch"<<std::endl
      <<"space: record second position"<<std::endl
      <<"move left or right side and change robots pitch"<<std::endl
      <<"space: recort third direction"<<std::endl
      <<"c: calibration"<<std::endl
      <<"q: start to broadcast tf"<<std::endl;


    // set up publish rate
    //tf::StampedTransform pep_posa,pep_posb,pep_posc,pep_posd,pep_pose;
    //tf::StampedTransform hol_posa,hol_posb,hol_posc,hol_posd,hol_pose;
	if(argc>=4){
		calibrator calib(holoLinkedFrame,odomFrame,robotFootFrame,fpath);
			calib.run();
	}else{
		calibrator calib(holoLinkedFrame,odomFrame,robotFootFrame);
			calib.run();
	}

    


    
    return 0;
}
