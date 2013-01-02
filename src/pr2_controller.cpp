/*
 * listener.cpp
 *
 *  Created on: Sep 20, 2012
 *      Author: njordan
 */


#include "ros/ros.h"

#include "std_msgs/String.h"

#include "gazebo_msgs/GetModelState.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"

#include <math.h>
#include <sstream>
#include <string>
#include <time.h>
#include <unistd.h>
#include <signal.h>
#include <termios.h>
//#include <boost/asio.hpp>

class pr2_navigation_tree
{
private:

	geometry_msgs::Twist cmd;
	ros::NodeHandle n_;
	ros::Publisher vel_pub_;
	gazebo_msgs::GetModelState modelStateRequest_;
	gazebo_msgs::GetModelStateResponse modelState_;
	ros::ServiceClient modelStateClient;
	float heading;

	void updateModelState();
	void moveToPoint(double x , double y );

public:

  void init()
  {
    cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;

    vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    modelStateClient = n_.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");

    modelStateRequest_.request.model_name = "pr2";

    heading = 0.0;

  }

  void nav_loop() {

	  while(true) {

		  //cmd.linear.x = 0.5;

		  //vel_pub_.publish(cmd);

		  //moveToPoint(10,10);

		  //break;

		  updateModelState();



	  	  }

  }

};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{

	tcsetattr(kfd, TCSANOW, &cooked);
	exit(0);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pr2_base_keyboard");

	pr2_navigation_tree pr2_tree;
	pr2_tree.init();

	signal(SIGINT,quit);

	pr2_tree.nav_loop();

	return(0);
}

void pr2_navigation_tree::updateModelState() {

	if (modelStateClient.call(modelStateRequest_)) {

			modelState_ = modelStateRequest_.response;

			float x,y,z,w;
			float ex,ey,ez;

			x = modelState_.pose.orientation.x;
			y = modelState_.pose.orientation.y;
			z = modelState_.pose.orientation.z;
			w = modelState_.pose.orientation.w;

			ex = atan2( 2 * (x*y + z*w) , 1 - 2*( pow(y,2) + pow(z,2) ) );

			ey = asin( 2 * ( x*z - w*y ) );

			ez = atan2( 2 * (x*w + y*z) , 1 - 2*( pow(z,2) + pow(w,2) ) );

			//printf("x: %f \n",modelState_.pose.position.x);
			//printf("y: %f \n",modelState_.pose.position.y);
			//printf("roll : %f \n",ex);

			//signal(SIGINT,quit);

			}

		else {

			ROS_ERROR("Failed to get model info! Is the PR2 running in gazebo?");

			signal(SIGINT,quit);

		  }

}

void pr2_navigation_tree::moveToPoint(double x, double y)
{

	geometry_msgs::Point destination(-20,-13,0);

	geometry_msgs::Point location = modelState_.pose.position;

	geometry_msgs::Point travelVector = destination - location;

	cmd.angular.z = 1.0;


}
