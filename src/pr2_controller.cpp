/*
 * listener.cpp
 *
 *  Created on: January 18, 2013
 *      Author: Nathan Jordan
 *       Email: natedagreat27274@gmail.com
 *
 */

///////////////////////////////////////////////////////////////////////////////////////////////////
////
////    Includes
////
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>

#include <std_msgs/String.h>

#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>

#include <gazebo_msgs/ModelState.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>

#include "../include/NavigationTree.h"

#include <math.h>
#include <sstream>
#include <string>
#include <time.h>
#include <unistd.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

///////////////////////////////////////////////////////////////////////////////////////////////////
////
////    Macros
////
///////////////////////////////////////////////////////////////////////////////////////////////////

#define PI 3.14159265359

///////////////////////////////////////////////////////////////////////////////////////////////////
////
////    Constants
////
///////////////////////////////////////////////////////////////////////////////////////////////////

const int QUADRANT_1 = 1;
const int QUADRANT_2 = 2;
const int QUADRANT_3 = 3;
const int QUADRANT_4 = 4;

const char INPUT_LEFT = 'l';
const char INPUT_RIGHT = 'r';

const float MAX_ANGULAR_VELOCITY = 4.0;
const float MAX_LINEAR_VELOCITY  = 1.5;
const float DISTANCE_WINDOW = 0.3;
const float ROTATION_WINDOW = 0.01;

const std::string MODEL_NAME = "pr2";
const std::string INPUT_FILE_LOCATION = "/home/njordan/fuerte_workspace/ncs_pr2_navigation/input/sample.txt";

///////////////////////////////////////////////////////////////////////////////////////////////////
////
////    Main
////
///////////////////////////////////////////////////////////////////////////////////////////////////

class pr2_navigation_tree
{
private:

	geometry_msgs::Twist cmd;
	
	ros::Publisher vel_pub_;
	
	gazebo_msgs::SetModelState setModelStateRequest_;
	gazebo_msgs::GetModelState getModelStateRequest_;
	gazebo_msgs::GetModelStateResponse modelState_;
	
	ros::ServiceClient getModelStateClient_;
	ros::ServiceClient setModelStateClient_;
	
	ros::NodeHandle n_;
	
	NavigationTree navTree;
	
	float heading;
	bool moving;

	void updateModelState();
	void moveToPoint(double x , double y );
	float determineAngle(double x, double y);
	int determineQuadrant(double x, double y);
	float convertModelAngle(float angle);
	float determineRotation(float desiredHeading, float currentHeading);
	void resetModel();

public:

	void init()
	{
		
		heading = 0.0;
		
		//Set up movement parameters
		cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;

		vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		
		//set up model state stuff
		getModelStateClient_ = n_.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
		
		getModelStateRequest_.request.model_name = MODEL_NAME;
		
		//The following is used for resetting the robot at 0,0
		setModelStateClient_ = n_.serviceClient<gazebo_msgs::SetModelState>("gazebo/set_model_state");
		
		moving = false;

		geometry_msgs::Pose start_pose;
		start_pose.position.x = 0.0;
		start_pose.position.y = 0.0;
		start_pose.position.z = 0.0;
		start_pose.orientation.x = 0.0;
		start_pose.orientation.y = 0.0;
		start_pose.orientation.z = 0.0;
		start_pose.orientation.w = 0.0;

		geometry_msgs::Twist start_twist;
		start_twist.linear.x = 0.0;
		start_twist.linear.y = 0.0;
		start_twist.linear.z = 0.0;
		start_twist.angular.x = 0.0;
		start_twist.angular.y = 0.0;
		start_twist.angular.z = 0.0;
		
		gazebo_msgs::ModelState modelState;
		
		modelState.model_name = MODEL_NAME;
		modelState.pose = start_pose;
		modelState.twist = start_twist;
		
		setModelStateRequest_.request.model_state = modelState;
		
		//load the point tree
		navTree.readFile(INPUT_FILE_LOCATION);
	}

	void nav_loop() 
	{
		char input;
		
		OrderedPair* p;
		
		while(true) 
		{
			
			if( !moving )
			{
				//TODO: Socket code for recieving 'left' or 'right' response from the NCS Brain Simulator
				input = 'r';

				if( input == INPUT_LEFT )
				
					p = navTree.getLeft();

				if( input == INPUT_RIGHT )
				
					p = navTree.getRight();

				if( p == 0 )
				{

					resetModel();

					continue;
				
				}

				ROS_INFO("Moving to point (%f,%f)" , p->x , p->y );

			}

			moveToPoint( p->x , p->y );

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

	if (getModelStateClient_.call(getModelStateRequest_))
	{

		modelState_ = getModelStateRequest_.response;

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

	}

	else
	{

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
	
	float theta = determineAngle(travelVector.x,travelVector.y);
	
	float rotation = determineRotation( theta, heading );
	
	float distance = sqrt( pow(travelVector.x,2) + pow(travelVector.y , 2) );
	
	float absrotation = ( rotation < 0 ) ? -rotation : rotation;
	
	float angularVelocity = (rotation / PI) * MAX_ANGULAR_VELOCITY;

	cmd.angular.z = angularVelocity;

	if( absrotation < ROTATION_WINDOW && distance > DISTANCE_WINDOW )
	
		cmd.linear.x = MAX_LINEAR_VELOCITY;
		
	else
		
		cmd.linear.x = 0.0;
	
	if( distance > DISTANCE_WINDOW )

		moving = true;

	else

		moving = false;


	vel_pub_.publish(cmd);
	
}

float pr2_navigation_tree::determineAngle(double x, double y)
{
	
	int quadrant = determineQuadrant(x,y);
	
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
	//       Mitigated by adding a small epsilon value to points in NavigationTree class

	if( x > 0 && y > 0)
		
		return QUADRANT_1;
	
	else if( x < 0 && y > 0 )
	
		return QUADRANT_2;
		
	else if( x < 0 && y < 0 )
	
		return QUADRANT_3;
		
	else if( x > 0 && y < 0 )
	
		return QUADRANT_4;
	
	else
	{

		ROS_ERROR("Invalid quadrant determined, possibly a special case issue, aborting");

		signal(SIGINT,quit);

	}

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

void pr2_navigation_tree::resetModel()
{
	navTree.reset();
	
	cmd.linear.x = 0.0;
	
	cmd.angular.z = 0.0;
	
	vel_pub_.publish(cmd);
	
	setModelStateClient_.call(setModelStateRequest_);
	
	ROS_INFO("Model Reset");

}









