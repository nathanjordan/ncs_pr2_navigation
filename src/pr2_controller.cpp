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


#define PI 3.14159265359

const int QUADRANT_1 = 1;
const int QUADRANT_2 = 2;
const int QUADRANT_3 = 3;
const int QUADRANT_4 = 4;

const float MAX_ROATION_VELOCITY = 1.5;
const float MAX_LINEAR_VELOCITY  = 2.0;

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
	float determineAngle(double x, double y);
	int determineQuadrant(double x, double y);
	float convertModelAngle(float angle);
	float determineRotation(float desiredHeading, float currentHeading);

public:

	void init()
	{
		cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;

		vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

		modelStateClient = n_.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");

		modelStateRequest_.request.model_name = "pr2";

		heading = 0.0;
	}

	void nav_loop() 
	{
		while(true) 
		{

			moveToPoint(10.0,-5.0);

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

	//signal(SIGINT,quit);

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

			heading = convertModelAngle(ex);

			//printf("x: %f \n",modelState_.pose.position.x);
			//printf("y: %f \n",modelState_.pose.position.y);
			//ROS_INFO("roll : %f",ex);

			//signal(SIGINT,quit);

			}

		else {

			ROS_ERROR("Failed to get model info! Is the PR2 running in gazebo? [roslaunch pr2_gazebo pr2.launch]");

			signal(SIGINT,quit);

		  }

}

void pr2_navigation_tree::moveToPoint(double x, double y)
{

	geometry_msgs::Point destination;
	
	destination.x = x;
	
	destination.y = y;
	
	destination.z = 0.0;

	geometry_msgs::Point location = modelState_.pose.position;

	geometry_msgs::Point travelVector;
	
	travelVector.x = destination.x - location.x;
	
	travelVector.y = destination.y - location.y;
	
	travelVector.z = destination.z - location.z;
	
	float theta = determineAngle(x,y);
	
	float rotation = determineRotation( theta, heading );
	
	float angularVelocity = (rotation / PI) * MAX_ROATION_VELOCITY;
	
	cmd.angular.z = angularVelocity;
	
	//if( abs(rotation) < 0.01 )
	
	//	cmd.linear.x = 1.0;

	vel_pub_.publish(cmd);
	
	ROS_INFO("current : %f", heading);
	
	ROS_INFO("desired : %f", theta);
	
	ROS_INFO("rotation: %f", rotation);

}

float pr2_navigation_tree::determineAngle(double x, double y)
{
	
	int quadrant = determineQuadrant(x,y);
	
	//ROS_INFO("quad : %i",quadrant);
	
	//ROS_INFO("pi  : %f",PI);
	
	//ROS_INFO("x,y : %f,%f",x,y);
	
	//ROS_INFO("asin : %f",atan( -x / y ));
	
	if(quadrant == QUADRANT_1)
		
		return atan( y / x );
	
	else if(quadrant == QUADRANT_2)
	
		return (PI / 2) + atan( -x / y );
		
	else if(quadrant == QUADRANT_3)
	
		return PI + atan( -y / -x );
	
	else if(quadrant == QUADRANT_4)
	
		return (3 * PI / 2) + atan( x / -y );

}

int pr2_navigation_tree::determineQuadrant(double x, double y)
{
	//TODO: Special cases of angles 0, pi/2, -pi, and -pi/2
	if( x > 0 && y > 0)
		
		return QUADRANT_1;
	
	else if( x < 0 && y > 0 )
	
		return QUADRANT_2;
		
	else if( x < 0 && y < 0 )
	
		return QUADRANT_3;
		
	else if( x > 0 && y < 0 )
	
		return QUADRANT_4;
	
}

float pr2_navigation_tree::convertModelAngle(float angle)
{
	if( angle < 0 )
		
		return 2 * PI + angle;
	
	else
	
		return angle;
}

float pr2_navigation_tree::determineRotation(float desiredHeading, float currentHeading)
{
		
	float shortest = desiredHeading - currentHeading;
	
	if( shortest > PI )
	
		shortest -= 2 * PI;
		
	if( shortest < -PI )
	
		shortest += 2 * PI;
		
	return shortest;
	
}









