/*
 *  GPSController.cpp
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

#include "../include/GPSController.h"
#include "../include/DirectionInterface.h"

///////////////////////////////////////////////////////////////////////////////////////////////////
////
////    Member Functions
////
///////////////////////////////////////////////////////////////////////////////////////////////////

void GPSController::init(std::string gpsfile , DirectionInterface* directionInterface)
{

	//set direction interface implementation
	directionInterface_ = directionInterface;

	//initialize heading
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
	navTree_.readFile(gpsfile);
}

void GPSController::ros_loop()
{
	char input;

	OrderedPair* p;

	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		//if the robot is not currently moving to a point
		if( !moving )
		{

			input = directionInterface_->getDirection();

			if( input == INPUT_LEFT )

				p = navTree_.getLeft();

			if( input == INPUT_RIGHT )

				p = navTree_.getRight();

			if( p == 0 )
			{

				resetModel();

				continue;

			}

			ROS_INFO("Moving to point (%f,%f)" , p->x , p->y );

		}

		moveToPoint( p->x , p->y );

		updateModelState();

		ros::spinOnce();

		loop_rate.sleep();
	}
}

void GPSController::updateModelState() {

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

		exit(1);

	}

}

void GPSController::moveToPoint(double x, double y)
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

float GPSController::determineAngle(double x, double y)
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

int GPSController::determineQuadrant(double x, double y)
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

		exit(1);

	}

}

float GPSController::convertModelAngle(float angle)
{
	if( angle < 0 )

		return 2 * PI + angle;

	else

		return angle;
}

float GPSController::determineRotation(float desiredHeading, float currentHeading)
{

	float shortest = desiredHeading - currentHeading;

	if( shortest > PI )

		shortest -= 2 * PI;

	if( shortest < -PI )

		shortest += 2 * PI;

	return shortest;

}

void GPSController::resetModel()
{
	navTree_.reset();

	cmd.linear.x = 0.0;

	cmd.angular.z = 0.0;

	vel_pub_.publish(cmd);

	setModelStateClient_.call(setModelStateRequest_);

	ROS_INFO("Model Reset");

}
