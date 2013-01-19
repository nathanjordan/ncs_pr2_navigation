/*
 *  ncs_pr2_navigation.cpp
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
#include "../include/NCSDirectionInterface.h"
#include "../include/KeyboardDirectionInterface.h"

///////////////////////////////////////////////////////////////////////////////////////////////////
////
////    Macros
////
///////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////////
////
////    Constants
////
///////////////////////////////////////////////////////////////////////////////////////////////////

const std::string INPUT_FILE_LOCATION = "/home/njordan/fuerte_workspace/ncs_pr2_navigation/input/gps.txt";

///////////////////////////////////////////////////////////////////////////////////////////////////
////
////    Main
////
///////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ncs_pr2_navigation");

	GPSController controller;

	NCSDirectionInterface* ncsInterface = new NCSDirectionInterface();

	KeyboardDirectionInterface* keyboardInterface = new KeyboardDirectionInterface();

	controller.init(INPUT_FILE_LOCATION, ncsInterface);

	controller.ros_loop();

	return(0);
}











