/*
 * NCSSocketInterface.cpp
 *
 *  Created on: Jan 18, 2013
 *      Author: njordan
 */

#include "../include/NCSDirectionInterface.h"

char NCSDirectionInterface::getDirection()
{
	int i = rand() % 2;

	return ( i == 0 ) ? 'l' : 'r';
}

NCSDirectionInterface::NCSDirectionInterface()
{
	srand ( time(NULL) );
}

