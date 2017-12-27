/*
Copyright (c) 2017 Ryoichi Ishikawa. All rights reserved.

This software is released under the MIT License.
http://opensource.org/licenses/mit-license.php
*/


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


class teleop{
public:
  teleop();
  void operation(int c);
  
private:
  ros::Publisher cmdvel_pub_;
  
};