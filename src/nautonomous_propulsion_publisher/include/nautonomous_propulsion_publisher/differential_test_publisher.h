
#ifndef DIFFERENTIALTESTPUBLISHER_H_
#define DIFFERENTIALTESTPUBLISHER_H_

#include <stdint.h>                 // Include your standard types definition file to accomodate the C99 uint_x types

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>

class DifferentialTestPublisher {
 public:
  DifferentialTestPublisher(ros::NodeHandle nh, ros::NodeHandle nh_private);

 private:
  // Params
  float motor_propulsion_scalar_;
  uint8_t propulsion_mode_index_;

  // Motor left and right constants
  static const uint8_t propulsion_array_size_ = 13;
  float left_motor_propulsion_array_[propulsion_array_size_] =   {1.0, -1.0, 0.5, -0.5,  1.0, -1.0, 0.5, -0.5,  1.0, -0.5,  0.5, -0.25, 0.0};
  float right_motor_propulsion_array_[propulsion_array_size_] =  {1.0, -1.0, 0.5, -0.5, -1.0,  1.0, -0.5, 0.5, -0.5,  1.0, -0.25, 0.5, 0.0};
  
  // Subscribers
  ros::Subscriber differential_next_mode_subscriber_;
  ros::Subscriber differential_set_mode_subscriber_;
 
  // Publishers
  ros::Publisher differential_left_publisher_;
  ros::Publisher differential_right_publisher_; 

  //Helper function
  float getScaledMotorMode(uint8_t index, bool left_motor);

  // Subscriber callbacks
  void callbackNextPropulsionMode(const std_msgs::Bool& next_mode);
  void callbackSetPropulsionMode(const std_msgs::UInt8& set_mode);

  //Publish functions
  void publishDifferentialLeftRight(uint8_t index);

}; 

#endif // DIFFERENTIALTESTPUBLISHER_H_