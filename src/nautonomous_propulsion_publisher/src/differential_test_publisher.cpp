#include <nautonomous_propulsion_publisher/differential_test_publisher.h>

DifferentialTestPublisher::DifferentialTestPublisher(ros::NodeHandle nh, ros::NodeHandle nh_private)
{
    int propulsion_mode_index = 0;
    nh_private.param("propulsion_mode_index", propulsion_mode_index, 0);
    propulsion_mode_index_ = (uint8_t) propulsion_mode_index;

    nh_private.param("motor_propulsion_scalar", motor_propulsion_scalar_, (float) 1.0);
    ROS_INFO("%i %f", propulsion_mode_index_, motor_propulsion_scalar_);

    // Subscribers use "rostopic pub -1 /change_inputs std_msgs/Bool 1" to publish on this topic from the terminal...
    differential_next_mode_subscriber_ = nh.subscribe("next_topic", 1, &DifferentialTestPublisher::callbackNextPropulsionMode, this);
    differential_set_mode_subscriber_ = nh.subscribe("set_topic", 1, &DifferentialTestPublisher::callbackSetPropulsionMode, this);
    
    // Publishers
    differential_left_publisher_ = nh.advertise<std_msgs::Float32>("left_motor_topic", 1, true);
    differential_right_publisher_ = nh.advertise<std_msgs::Float32>("right_motor_topic", 1, true);

}

float DifferentialTestPublisher::getScaledMotorMode(uint8_t index, bool left_motor)
{
    if(left_motor)
    {
        return left_motor_propulsion_array_[index] * motor_propulsion_scalar_;
    }
    else
    {
        return right_motor_propulsion_array_[index] * motor_propulsion_scalar_;
    }
}

void DifferentialTestPublisher::publishDifferentialLeftRight(uint8_t index)
{
    if (index < propulsion_array_size_ && index >= 0) 
    {   
        std_msgs::Float32 left_message, right_message;

        left_message.data = getScaledMotorMode(index, true);
        right_message.data = getScaledMotorMode(index, false);

        differential_left_publisher_.publish(left_message);
        differential_right_publisher_.publish(right_message);
    } 
}

void DifferentialTestPublisher::callbackNextPropulsionMode(const std_msgs::Bool& next_mode) 
{
    if (!next_mode.data)
    {
        return;
    }

    publishDifferentialLeftRight(propulsion_mode_index_);
   
    propulsion_mode_index_ = (propulsion_mode_index_ + 1) % propulsion_array_size_;
}

void DifferentialTestPublisher::callbackSetPropulsionMode(const std_msgs::UInt8& set_mode) 
{
    publishDifferentialLeftRight(set_mode.data);
}