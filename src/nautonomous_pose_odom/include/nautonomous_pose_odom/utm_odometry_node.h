// ROS
#include <ros/ros.h>

// Messages
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>

// Custom message
#include <nautonomous_pose_msgs/PointWithCovarianceStamped.h>

// Odom publisher
ros::Publisher odom_pub;

// Params
std::string frame_id;
std::string child_frame_id;

double rot_cov;
bool old_time;

/**
 * Publishes odometry from the utm coordinate
 * @params sensor_msgs::NavSatFixConstPtr& fix utm fix coordinate
 */
void callback_UTM_fix(const nautonomous_pose_msgs::PointWithCovarianceStampedConstPtr& point_with_covariance_stamped);

/**
 * Subscribes to 'fix', publishes to 'odom'.
 */
int main (int argc, char **argv);
