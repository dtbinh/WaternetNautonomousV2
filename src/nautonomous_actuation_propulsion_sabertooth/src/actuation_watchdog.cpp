/**
    Actuation watchdog
    actuation_watchdog.cpp
    Purpose: Check, set and update the current state of the actuation platform

    @author Daan Zeeuwe
    @version 1.0 8/7/17 
*/

#include <nautonomous_actuation_propulsion_sabertooth/actuation_watchdog.h>

ActuationWatchdog::ActuationWatchdog(ros::NodeHandle node_handle){
    // Initialize status message
    status_msg_.level = -1;
    status_msg_.message = "Unknown";

    // Publisher
    watchdog_publisher_ = node_handle.advertise<diagnostic_msgs::DiagnosticStatus>("watchdog", 1000);

}

// Give the status response from the actuation platform and check if the current status is still the same.
bool ActuationWatchdog::checkStatus(std::string response)
{
    diagnostic_msgs::DiagnosticStatus temp_status_msg = createStatus(response);
    
    if (adjustStatus(temp_status_msg))
    {
        watchdog_publisher_.publish(status_msg_); 
    } 
}

// Create a status message
diagnostic_msgs::DiagnosticStatus ActuationWatchdog::createStatus(std::string response)
{
    diagnostic_msgs::DiagnosticStatus temp_status_msg;

    if (!response.empty())
    {
        temp_status_msg.level = 0; // OK status
        temp_status_msg.message = "Ok";    
    }
    else
    {
        temp_status_msg.level = 1; // ERROR status
        temp_status_msg.message = "Error: no hearthbeat";
    }

    return temp_status_msg;
}

// Check if the given status message is different to the current status message, if so change the current status message.
bool ActuationWatchdog::adjustStatus(diagnostic_msgs::DiagnosticStatus temp_status_msg){
    
    //Publish only new status
    if (temp_status_msg.level != status_msg_.level)
    {
        status_msg_.level = temp_status_msg.level;
        status_msg_.message = temp_status_msg.message;

        return true;
    }
    return false;
}

// Check if the current status message has a valid status from the actuation platform
bool ActuationWatchdog::isActive()
{
    return (status_msg_.level == 0);
}
