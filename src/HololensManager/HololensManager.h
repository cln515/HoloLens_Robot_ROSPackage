#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include<netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include "linearSolver.h"

#include <map>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>

#include <boost/geometry/index/rtree.hpp>




using namespace std;
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud.h>
// to store queries results
#include <vector>
#include "Eigen/Core"
#include <Eigen/Geometry>
// just for output
#include <iostream>
#include <boost/foreach.hpp>
//#include <../../opt/ros/indigo/include/nav_msgs/OccupancyGrid.h>

#define PI 3.14159265359


#define CMD_IMG_REQ 4
#define FINISH_ALIGNMENT 8
#define LOC_FAILED 12
#define DEPTH_STREAM_REQ 16
#define DEPTH_STREAM_OFF_REQ 18
#define FINISH_RECV_MESH 20


#define HEADER_CAMERA 10
#define HEADER_IMAGE 20
#define HEADER_CHANC 30
#define HEADER_POSLOST 40
#define HEADER_TRACKLOST 50
#define HOLOLENS_HEIGHT 60
#define HEADER_DEPTHSTREAM 100

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<float, 2, bg::cs::cartesian> point;
typedef bg::model::box<point> box;
typedef std::pair<point, unsigned> value;

typedef map<string,tf::StampedTransform>MTF;


  namespace hololensManager
  {
    class hololensNode
      {
	public:
	  hololensNode(char* ip);
	  void start();
	  void stop(){};

	private:
	  void run();
	  void callbackInitPoseStamped(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& goalPose);
	  void callbackMap(const nav_msgs::OccupancyGrid::ConstPtr& subscribedmap);
	  void reqHoloMap(const std_msgs::Bool::ConstPtr& call);	
          void sendMessage(unsigned int val);
	  void recvMessage(unsigned int size,char* pointer);
	  
	  void dimensionRemover(tf::StampedTransform transform, double& _x,double& _y, double& _yaw,tf::Quaternion& _initq);
	  
	  void recvPosition();
	  void recvMap(cv::Mat& dst,double upper,double lower);
	  bool registration(cv::Mat dst,int iteration_start,int iteration_time);
	  
	  ros::Publisher holomapviz_pub_;
	  ros::Publisher obst_pub_;	  
	  ros::Publisher holoGrid_pub_;	
	  ros::Publisher floor2holo_pub_;
	  ros::Publisher pos_lost_pub_;
	  
	  ros::Subscriber initScriber;
	  ros::Subscriber mapSubscriber;
	  ros::Subscriber obst_sub_;
	  ros::Subscriber switchCallSub_;
	  
	  tf::TransformBroadcaster tf_br_;
          tf::StampedTransform tf_anchor_to_hololens_;
          tf::StampedTransform tf_originmap_to_holomap;
          tf::StampedTransform tf_map_to_anchor_;  
	  tf::TransformListener listener;

	  int sock;
          struct sockaddr_in holo;
	  ros::NodeHandle nh;
	  nav_msgs::OccupancyGrid grid;
	  
	  double holoheight;
	  Eigen::Vector3d floorpoint;

	  double initx,inity;
	  double yaw,scale;	  
	  tf::Quaternion initq;
	  bgi::rtree< value, bgi::quadratic<16> > rtree;  
	  bool bPoseSubscribed=false;
	  bool bPositionTracked=true;
	  bool bSpatialAnchorUpdate=false;
	  bool mapdateMutex=false;
	  bool mapdate=false;
	  MTF transLookUp;
	  ros::Time SATime;
	  
      };
    
    
    
  }

