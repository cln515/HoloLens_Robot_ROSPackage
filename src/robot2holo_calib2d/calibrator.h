#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <boost/concept_check.hpp>
#include <boost/thread.hpp>
#include <termios.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include<fstream>
#include "teleop.h"
#include "nonlinear_solver.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::AutoDiffCostFunction;
using ceres::NumericDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using namespace Eigen;

class calibrator{
public:
	calibrator(std::string holoLinkedFrame_, std::string odomFrame_, std::string robotFootFrame_);
	calibrator(std::string holoLinkedFrame_, std::string odomFrame_, std::string robotFootFrame_,std::string fpath_);
	void run();
private:
	void publish_thread();

	void calibration(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos, Matrix3d& R, Vector3d& T);
	void horizontalCalibration(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos, std::vector<Eigen::Vector3d> verticalVecs, std::vector<Eigen::Vector3d> normVecs, Matrix3d& R, Vector3d& T);
	void transition_log(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos,std::ofstream& ofs);	
	void linear_calibration(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos, Matrix3d& R, Vector3d& T);
	void nonlinear_calibration(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos, Matrix3d& R, Vector3d& T);	
	void horizontal_initialization(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos, std::vector<Eigen::Vector3d> verticalVecs, std::vector<Eigen::Vector3d> normVecs, Matrix3d& R, Vector3d& T);
	void nonlinear_horizontal_calibration(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos, std::vector<Eigen::Vector3d> verticalVecs, std::vector<Eigen::Vector3d> normVecs, Matrix3d& R, Vector3d& T);
	void poseStampedCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
	
	std::vector<tf::StampedTransform> pep_pos;
	std::vector<tf::StampedTransform> hol_pos;
	std::vector<Eigen::Vector3d> floor2holo;
	std::vector<Eigen::Vector3d> head2foot;
	std::vector<geometry_msgs::PoseStamped> floor_direc;
	
	geometry_msgs::PoseStamped latestPoseStamped;
	
	tf::StampedTransform transform;
	tf::StampedTransform transform2;
	tf::StampedTransform transform3;	
	tf::StampedTransform transform4;
	Matrix3d R;
	Vector3d T;
	//double publish_rate_ = 100;
        //ros::Rate loop_rate;    
	tf::TransformBroadcaster tf_br_;
	tf::StampedTransform tf_map_to_odom_;
	ros::Publisher cmdvel_pub_;
	ros::Subscriber holo_floor_sub_;
	ros::NodeHandle nh;
	
	bool horizontalCalibMode;
	
	std::string holoLinkedFrame,odomFrame,robotFootFrame,fpath;
	// set up parent and child frames
	

	tf::TransformListener listener;
	teleop tele;
	
	int getch()
	{
		static struct termios oldt, newt;
		tcgetattr( STDIN_FILENO, &oldt);           // save old settings
		newt = oldt;
		newt.c_lflag &= ~(ICANON);                 // disable buffering      
		tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

		int c = getchar();  // read character (non-blocking)

		tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
		return c;
	}

	Matrix4d btTrans2EigMat4d(tf::Transform t){
		tf::Matrix3x3 btm(t.getRotation());
		Matrix4d m;
		m<<btm.getRow(0).getX(),btm.getRow(0).getY(),btm.getRow(0).getZ(),t.getOrigin().getX(),
		btm.getRow(1).getX(),btm.getRow(1).getY(),btm.getRow(1).getZ(),t.getOrigin().getY(),
		btm.getRow(2).getX(),btm.getRow(2).getY(),btm.getRow(2).getZ(),t.getOrigin().getZ(),
		0,0,0,1;
		return m;
	}

	Vector3d mat2axis(Matrix4d m){
		double x,y,z;
		double r=sqrt((m(2,1)-m(1,2))*(m(2,1)-m(1,2))+(m(0,2)-m(2,0))*(m(0,2)-m(2,0))+(m(1,0)-m(0,1))*(m(1,0)-m(0,1)));
		x=(m(2,1)-m(1,2))/r;
		y=(m(0,2)-m(2,0))/r;
		z=(m(1,0)-m(0,1))/r;
		Vector3d t;
		t<<x,y,z;
		return t;
	}

	void mat2axis_angle(Matrix4d m,Vector3d& retv, double& angle){
		double x,y,z;
		double r=sqrt((m(2,1)-m(1,2))*(m(2,1)-m(1,2))+(m(0,2)-m(2,0))*(m(0,2)-m(2,0))+(m(1,0)-m(0,1))*(m(1,0)-m(0,1)));
		x=(m(2,1)-m(1,2))/r;
		y=(m(0,2)-m(2,0))/r;
		z=(m(1,0)-m(0,1))/r;
		Vector3d t;
		t<<x,y,z;
		retv=t;
		angle=acos((m(0,0)+m(1,1)+m(2,2)-1)/2);
	}
};
