/*
 * PointReader.cpp
 *
 *  Created on: Jul 10, 2012
 *      Author: njordan
 */

#include "../include/NavigationTree.h"

#include <ros/ros.h>

#include <std_msgs/String.h>

#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <vector>

#define EPSILON 0.001

void NavigationTree::readFile( std::string filename ) {

	std::ifstream fin;
	std::vector<NavigationPoint*> pointList;

	std::string start;
	int startPoint;
	char endPoint;

	fin.open( filename.c_str() );

	if( !fin.good() ) {

		ROS_INFO("Cannot open point tree file, aborting");

		abort();

	}

	fin >> start;

	if( start.compare( "START" ) != 0 ) {

		ROS_INFO("Start element not found");

		abort();

		}

	fin >> startPoint;

	while( fin.good() ) {

		NavigationPoint* p = new NavigationPoint();

		pointList.push_back( p );

		fin >> p->label;

		fin >> endPoint;

		fin >> p->x;

		fin >> p->y;

		//TODO: might be necessary for quadrants
		p->x += EPSILON;

		p->y += EPSILON;

		if( endPoint == 'T' ) {

			p->endpoint = true;

			continue;

			}

		else if ( endPoint == 'F' ) {

			fin >> p->leftLabel;

			fin >> p->rightLabel;

			p->endpoint = false;

			}

		else {

			ROS_INFO("bad navigation file format");

			abort();

			}
		}

	std::vector<NavigationPoint*>::iterator i,j;

	for( i = pointList.begin() ; i < pointList.end() ; i++ ) {

		if( (*i)->endpoint )

			continue;

		for( j = pointList.begin() ; j < pointList.end() ; j++ ) {

			if( (*i)->leftLabel == (*j)->label )

				(*i)->left = *j;

			if( (*i)->rightLabel == (*j)->label )

				(*i)->right = *j;

			}

		}

	root = current = *pointList.begin();

	}

void NavigationTree::reset() {

	current = root;

	}

OrderedPair* NavigationTree::getLeft() {

	if( !current->left )

		return 0;

	current = current->left;

	OrderedPair* p = new OrderedPair();

	p->x = current->x;
	p->y = current->y;

	return p;

	}

OrderedPair* NavigationTree::getRight() {

	if( !current->left )

		return 0;

	current = current->right;

	OrderedPair* p = new OrderedPair();

	p->x = current->x;
	p->y = current->y;

	return p;

	}
