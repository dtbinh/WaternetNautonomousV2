/**
    Sabertooth Motor Driver
    sabertooth_motor_driver.h
    Purpose: Transform ROS messages to Sabertooth packets content.

    @author Daan Zeeuwe
    @version 1.0 8/7/17 
*/


#ifndef SABERTOOTHMOTORDRIVER_HPP_
#define SABERTOOTHMOTORDRIVER_HPP_

#include <stdio.h>                  // Need to be able to use standard output "putchar" command
#include <stdint.h>                 // Include your standard types definition file to accomodate the C99 uint_x types
#include <algorithm>                // std::max and std::min

#include <ros/ros.h>

#include <std_msgs/Float32.h>

#include <geometry_msgs/Twist.h>

#include "sabertooth_packet.h"

class SabertoothMotorDriver
{
    private: 

        const uint8_t max_command_value_ = 127;

        const uint8_t min_fast_ramp = 26;
        const uint16_t max_fast_ramp = 256;

        const uint16_t max_slow_ramp = 16787;

        int sabertoothScale(double value, double max_value, double min_value);

    public:

        SabertoothMotorDriver();

        void processMotorValue(SabertoothPacket* packet, double value, double max_value, double min_value, int positive_command, int negative_command);
    
        void setSerialTimeout(SabertoothPacket* packet, uint16_t timeout_ms);
        
        void setRamp(SabertoothPacket* packet, uint16_t ramp_time_ms);
};

#endif // SABERTOOTHMOTORDRIVER_HPP_
