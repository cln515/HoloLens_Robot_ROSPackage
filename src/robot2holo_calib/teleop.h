#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


class teleop{
public:
  teleop();
  void operation(int c);
  
private:
  ros::Publisher cmdvel_pub_;
  
};