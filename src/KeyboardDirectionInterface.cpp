/*
 * KeyboardDirectionInterface.cpp
 *
 *  Created on: Jan 18, 2013
 *      Author: njordan
 */

#include "../include/KeyboardDirectionInterface.h"

char KeyboardDirectionInterface::getDirection()
{
	char c = '\0';
	struct termios cooked, raw;
	int kfd = 0;

	// get the console in raw mode
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	// Setting a new line, then end of file
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	while( ros::ok() && c != DirectionInterface::DIRECTION_LEFT && c != DirectionInterface::DIRECTION_RIGHT )
	{
		if(read(kfd, &c, 1) < 0)
		{
			perror("read():");
			exit(-1);
		}

		if( c == DirectionInterface::DIRECTION_LEFT )

			return DirectionInterface::DIRECTION_LEFT;

		if( c == DirectionInterface::DIRECTION_RIGHT )

			return DirectionInterface::DIRECTION_RIGHT;

	}

	tcsetattr(kfd, TCSANOW, &cooked);


}

KeyboardDirectionInterface::KeyboardDirectionInterface()
{
	//TODO: Implement keyboard constructor
}

