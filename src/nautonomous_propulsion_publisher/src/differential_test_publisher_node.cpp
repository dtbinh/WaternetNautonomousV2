#include <nautonomous_propulsion_publisher/differential_test_publisher.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "differential_test_publisher_node");

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  
  DifferentialTestPublisher differential_test_publisher(nh, nh_private);

  ros::spin();
}
