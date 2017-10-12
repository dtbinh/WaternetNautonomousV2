/**
    Sabertooth Propulsion Node
    sabertooth_propulsion_node.h
    Purpose: ROS Node to startup the Sabertooth Propulsion module

    @author Daan Zeeuwe
    @version 1.0 8/7/17 
*/

#ifndef SABERTOOTHPROPULSIONNODE_H_
#define SABERTOOTHPROPULSIONNODE_H_

#include <signal.h>

#include <ros/ros.h>

#include "sabertooth_propulsion.h"

void shutdownHandler(int sig);

SabertoothPropulsion* sabertooth_propulsion;

#endif /* SABERTOOTHPROPULSIONNODE_H_ */
