#include "teleop.h"


teleop::teleop(){
  ros::NodeHandle nh;
	    cmdvel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
}


void teleop::operation(int c){
  	if(c=='i'){
	      geometry_msgs::Twist msgTwist;
	      msgTwist.linear.x = 0.1;
	      msgTwist.linear.y = 0;
	      msgTwist.linear.z = 0;
	      msgTwist.angular.x = 0;
	      msgTwist.angular.y = 0;
	      msgTwist.angular.z = 0;
	      cmdvel_pub_.publish(msgTwist);
	}else if(c=='l'){
	      geometry_msgs::Twist msgTwist;
	      msgTwist.linear.x = 0;
	      msgTwist.linear.y = -0.1;
	      msgTwist.linear.z = 0;
	      msgTwist.angular.x = 0;
	      msgTwist.angular.y = 0;
	      msgTwist.angular.z = 0;
	      cmdvel_pub_.publish(msgTwist);
	}else if(c=='j'){
	      geometry_msgs::Twist msgTwist;
	      msgTwist.linear.x = 0;
	      msgTwist.linear.y = 0.1;
	      msgTwist.linear.z = 0;
	      msgTwist.angular.x = 0;
	      msgTwist.angular.y = 0;
	      msgTwist.angular.z = 0;
	      cmdvel_pub_.publish(msgTwist);
	}else if(c=='u'){
	      geometry_msgs::Twist msgTwist;
	      msgTwist.linear.x = 0;
	      msgTwist.linear.y = 0;
	      msgTwist.linear.z = 0;
	      msgTwist.angular.x = 0;
	      msgTwist.angular.y = 0;
	      msgTwist.angular.z = 0.2;
	      cmdvel_pub_.publish(msgTwist);
	}else if(c=='o'){
	      geometry_msgs::Twist msgTwist;
	      msgTwist.linear.x = 0;
	      msgTwist.linear.y = 0;
	      msgTwist.linear.z = 0;
	      msgTwist.angular.x = 0;
	      msgTwist.angular.y = 0;
	      msgTwist.angular.z = -0.2;
	      cmdvel_pub_.publish(msgTwist);
	}else if(c=='k'){
	      geometry_msgs::Twist msgTwist;
	      msgTwist.linear.x = -0.1;
	      msgTwist.linear.y = 0;
	      msgTwist.linear.z = 0;
	      msgTwist.angular.x = 0;
	      msgTwist.angular.y = 0;
	      msgTwist.angular.z = 0;
	      cmdvel_pub_.publish(msgTwist);
	}else if(c=='s'){
	      geometry_msgs::Twist msgTwist;
	      msgTwist.linear.x = 0;
	      msgTwist.linear.y = 0;
	      msgTwist.linear.z = 0;
	      msgTwist.angular.x = 0;
	      msgTwist.angular.y = 0;
	      msgTwist.angular.z = 0;
	      cmdvel_pub_.publish(msgTwist);
	}
  
}