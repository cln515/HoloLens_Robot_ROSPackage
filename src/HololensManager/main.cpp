#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include<netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include "linearSolver.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>

#include <boost/geometry/index/rtree.hpp>




using namespace std;
#include <nav_msgs/OccupancyGrid.h>
// to store queries results
#include <vector>

// just for output
#include <iostream>
#include <boost/foreach.hpp>
//#include <../../opt/ros/indigo/include/nav_msgs/OccupancyGrid.h>
#include "HololensManager.h"

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<float, 2, bg::cs::cartesian> point;
typedef bg::model::box<point> box;
typedef std::pair<point, unsigned> value;


int main(int argc,char **argv)
{
  
  cout<<argv[1]<<endl;
  if(argc<2){
    cout<<".exe hololens_ip"<<endl;
    return 0;
  }
  ros::init(argc, argv, "hololens_manager");
  hololensManager::hololensNode hn(argv[1]);
  hn.start();
}
