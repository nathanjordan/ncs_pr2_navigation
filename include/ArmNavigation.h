/*
 * ArmNavigation.h
 *
 *  Created on: Mar 27, 2013
 *      Author: njordan
 */

#ifndef ARMNAVIGATION_H_
#define ARMNAVIGATION_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <arm_navigation_msgs/utils.h>

class ArmNavigation {

public:

	ArmNavigation();

	virtual ~ArmNavigation();

	void resetArms();

	void setArmsHappy();

	void setArmsSad();
};

#endif /* ARMNAVIGATION_H_ */
