/**
    Actuation watchdog
    actuation_watchdog.h
    Purpose: Check, set and update the current state of the actuation platform

    @author Daan Zeeuwe
    @version 1.0 8/7/17 
*/

#ifndef ACTUATIONWATCHDOG_H_
#define ACTUATIONWATCHDOG_H_

#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticStatus.h>

/**
 Watchdog is responsible for translating the current state of the actuation platform
*/

class ActuationWatchdog
{
    private: 
        // ROS topic publisher for the current status of the actuation platform
        ros::Publisher watchdog_publisher_;

        // Current status message
        diagnostic_msgs::DiagnosticStatus status_msg_;

        // Create a status message
        diagnostic_msgs::DiagnosticStatus createStatus(std::string response);

        // Check if the given status message is different to the current status message, if so change the current status message.
        bool adjustStatus(diagnostic_msgs::DiagnosticStatus temp_status_msg);

    public:
        ActuationWatchdog(ros::NodeHandle node_handle);

        // Give the status response from the actuation platform and check if the current status is still the same.
        bool checkStatus(std::string response);

        // Check if the current tatus message has a valid status from the actuation platform
        bool isActive();

};

#endif /* ACTUATIONWATCHDOG_H_ */