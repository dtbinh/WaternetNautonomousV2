/**
    Sabertooth Propulsion
    sabertooth_propulsion.h
    Purpose: Delegating module that uses the serial, sabertooth and watchdog modules.

    @author Daan Zeeuwe
    @version 1.0 8/7/17 
*/

#ifndef SABERTOOTHPROPULSION_H_
#define SABERTOOTHPROPULSION_H_

#include <stdint.h>

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>

#include <geometry_msgs/Twist.h>

#include "actuation_watchdog.h"
#include "sabertooth_motor_driver.h"
#include "sabertooth_serial.h"
#include "sabertooth_packet.h"

class SabertoothPropulsion{
    private:
        ActuationWatchdog* actuation_watchdog_ = nullptr;
        SabertoothMotorDriver* sabertooth_motor_driver_ = nullptr;
        SabertoothSerial* sabertooth_serial_ = nullptr;

        SabertoothPacket sabertooth_packet_;

        ros::Publisher left_feedback_publisher_;
        ros::Publisher right_feedback_publisher_;

        ros::Subscriber left_propulsion_subscriber_;
        ros::Subscriber right_propulsion_subscriber_;
        ros::Subscriber twist_propulsion_subscriber_;

        bool differential_mode_;
        bool debug_;
        
        double minimum_forward_value_;
        double maximum_forward_value_;

        double minimum_turning_value_;
        double maximum_turning_value_; 

        double minimum_motor_value_;
        double maximum_motor_value_;

        void callbackPropulsionTwist(const geometry_msgs::Twist::ConstPtr& twist_message);

        void callbackPropulsionLeft(const std_msgs::Float32::ConstPtr& left_message);
        void callbackPropulsionRight(const std_msgs::Float32::ConstPtr& right_message);

        void sendPacket();
        
        void prepareMotorDriver(int serial_timeout, int ramp_time_ms);

        std_msgs::Int16 createFeedbackMessage();

    public:
        SabertoothPropulsion(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle);
        ~SabertoothPropulsion();

        void checkWatchdog();
        
        
};

#endif // SABERTOOTHPROPULSION_H_