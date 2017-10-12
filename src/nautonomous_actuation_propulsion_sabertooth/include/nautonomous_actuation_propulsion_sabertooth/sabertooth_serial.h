/**
    Sabertooth Serial
    sabertooth_serial.h
    Purpose: High level serial module for the sabertooth motor.

    @author Daan Zeeuwe
    @version 1.0 8/7/17 
*/

#ifndef SABERTOOTHSERIAL_H_
#define SABERTOOTHSERIAL_H_

#include <iostream>

#include <ros/ros.h>

#include "serial.h"
#include "sabertooth_packet.h"

class SabertoothSerial
{
    private:
        serial::Serial* propulsion_serial_;

        std::string serial_port_;
        bool serial_available_;
        int serial_baud_; 
        int serial_timeout_;

        bool initSerial();
        bool initialize();
        
    public:
        SabertoothSerial(ros::NodeHandle private_node_handle);
        ~SabertoothSerial();

        bool isOpen();

        std::string readStatus();
        void writePacket(SabertoothPacket packet);

        void flushInput();

};

#endif /* SABERTOOTHSERIAL_H_ */
