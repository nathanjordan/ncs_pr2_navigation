/*
 * DirectionInterface.h
 *
 *  Created on: Jan 18, 2013
 *      Author: njordan
 */

#ifndef DIRECTIONINTERFACE_H_
#define DIRECTIONINTERFACE_H_

class DirectionInterface {

public:

	static const char DIRECTION_LEFT = 'l';

	static const char DIRECTION_RIGHT = 'r';

	virtual char getDirection() = 0;

	virtual ~DirectionInterface() { };

};

#endif /* DIRECTIONINTERFACE_H_ */
