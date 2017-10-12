/**
    Sabertooth Propulsion Node
    sabertooth_propulsion_node.cpp
    Purpose: ROS Node to startup the Sabertooth Propulsion module

    @author Daan Zeeuwe
    @version 1.0 8/7/17 
*/


#include <nautonomous_actuation_propulsion_sabertooth/sabertooth_propulsion_node.h>

/**
 * \brief Shutdown handler, deinit serial, call ros::shutdown()
 * \param int sig
 * \return null 
 */
void shutdownHandler(int sig)
{
  // Deinit serial before shutdown down.
  delete sabertooth_propulsion;
  sabertooth_propulsion = nullptr;
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

/**
 * Main for command actuation node, subscribes to topics, call init serial
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "actuation_propulsion_sabertooth_node");

    ros::NodeHandle node_handle;
    ros::NodeHandle private_node_handle("~");

    sabertooth_propulsion = new SabertoothPropulsion(node_handle, private_node_handle);

    // When the program exists, go to the shutdown handler first.
    signal(SIGINT, shutdownHandler);

    ros::Rate rate(10);

    while(ros::ok)
    {
        sabertooth_propulsion->checkWatchdog();
        rate.sleep();
        ros::spinOnce();
    }
    
    delete sabertooth_propulsion;
    sabertooth_propulsion = nullptr;

    return 0;
}
