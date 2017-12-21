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
    if(argc>=5){
        fpath=std::string(argv[4]);
    }
    
    ros::init(argc, argv, "robot2holo_calib");

    std::cout<<"robot-hololens calibration"<<std::endl
      <<"space: record position"<<std::endl
      <<"c: conduct calibration"<<std::endl
      <<"i,j,k,l: move robot forward, leftward, backward, rightward"<<std::endl
      <<"u,o: rotate robot counter-clockwize, clockwize"<<std::endl
      <<"s: stop robot movement"<<std::endl
      <<"w: write calibration data"<<std::endl
      <<"z: write calibration parameter log as text file"<<std::endl
      <<"h: switch calibration mode (clear all recorded position)"<<std::endl
      <<"q: quit calibration program"<<std::endl;

    // set up publish rate
	if(argc>=4){
		calibrator calib(holoLinkedFrame,odomFrame,robotFootFrame,fpath);
			calib.run();
	}else{
		calibrator calib(holoLinkedFrame,odomFrame,robotFootFrame);
			calib.run();
	}

    


    
    return 0;
}
