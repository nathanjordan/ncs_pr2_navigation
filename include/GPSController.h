/*
 * GPSController.h
 *
 *  Created on: Jan 18, 2013
 *      Author: njordan
 */

#ifndef GPSCONTROLLER_H_
#define GPSCONTROLLER_H_

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

#include "NavigationTree.h"
#include "DirectionInterface.h"
#include "ArmNavigation.h"

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

const float PI = 3.14159265359;

const std::string MODEL_NAME = "pr2";

///////////////////////////////////////////////////////////////////////////////////////////////////
////
////    GPSController Class
////
///////////////////////////////////////////////////////////////////////////////////////////////////

class GPSController
{
public:

	void ros_loop();

	void init( std::string gpsfile , DirectionInterface* directionInterface );

private:

	///////////////////////////////////////////////////////////////////////////////////////////////////
	////    Variables
	///////////////////////////////////////////////////////////////////////////////////////////////////

	NavigationTree navTree_;

	DirectionInterface* directionInterface_;

	float heading;
	bool moving;

	geometry_msgs::Twist cmd;

	ros::Publisher vel_pub_;

	gazebo_msgs::SetModelState setModelStateRequest_;
	gazebo_msgs::GetModelState getModelStateRequest_;
	gazebo_msgs::GetModelStateResponse modelState_;

	ros::ServiceClient getModelStateClient_;
	ros::ServiceClient setModelStateClient_;

	ros::NodeHandle n_;

	///////////////////////////////////////////////////////////////////////////////////////////////////
	////    Functions
	///////////////////////////////////////////////////////////////////////////////////////////////////

	void updateModelState();

	void moveToPoint( double x , double y );

	float determineAngle( double x, double y );

	int determineQuadrant( double x, double y );

	float convertModelAngle( float angle );

	float determineRotation( float desiredHeading, float currentHeading );

	void resetModel();

};

#endif /* GPSCONTROLLER_H_ */
