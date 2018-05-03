#include <ros/ros.h>
#include <ros/console.h>

#include <cmath>
#include <nautonomous_mpc_msgs/Obstacle.h>
#include <nautonomous_mpc_msgs/Obstacles.h>

ros::Publisher message_pub;
nautonomous_mpc_msgs::Obstacle obstacle;
nautonomous_mpc_msgs::Obstacles obstacles;

float dt = 0.1;
int iter = 0;

int main (int argc, char** argv)
{
	srand (time(NULL));

	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
	   ros::console::notifyLoggerLevelsChanged();
	}

	ros::init (argc, argv,"TestFile");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

	message_pub = nh_private.advertise<nautonomous_mpc_msgs::Obstacles>("/Obstacle_detection/obstacles",1);

        ros::Rate loop_rate(20);

	obstacle.major_semiaxis = 5;
	obstacle.minor_semiaxis = 5;
	obstacle.pose.position.x = 10;
	obstacle.pose.position.y = 0;
	obstacle.twist.linear.x = 1;
	obstacle.pose.orientation.z = -1;

	obstacles.obstacles.push_back(obstacle);
	
	obstacle.major_semiaxis = 5;
	obstacle.minor_semiaxis = 5;
	obstacle.pose.position.x = 0;
	obstacle.pose.position.y = 10;
	obstacle.twist.linear.x = 1;
	obstacle.pose.orientation.z = 0;

	obstacles.obstacles.push_back(obstacle);

	ros::Duration(1).sleep();

	while (ros::ok())
	{	

		for (int i = 0; i < obstacles.obstacles.size(); i++)
		{
//			obstacles.obstacles[i].twist.linear.x += 0.001 * (rand() % 100 - 50);
			obstacles.obstacles[i].pose.position.x += dt * (cos(obstacles.obstacles[i].pose.orientation.z) * obstacles.obstacles[i].twist.linear.x);// + 0.01 * (rand() % 100 - 50));
			obstacles.obstacles[i].pose.position.y += dt * (sin(obstacles.obstacles[i].pose.orientation.z) * obstacles.obstacles[i].twist.linear.x);// + 0.01 * (rand() % 100 - 50));
		}
		message_pub.publish(obstacles);

		ros::spinOnce();
		loop_rate.sleep();
	
		iter++;
		if(iter == 50)
		{
			obstacles.obstacles[0].pose.position.x = 30;
			obstacles.obstacles[0].pose.position.y = 50;
			obstacles.obstacles[0].pose.orientation.z = 3.14;
			obstacles.obstacles[0].twist.linear.x = 1.5;
		}
		else if((iter >= 100) && (iter < 500))
		{
			obstacles.obstacles[0].twist.linear.x = fmax(0.5,obstacles.obstacles[0].twist.linear.x - 0.01);
			obstacles.obstacles[0].pose.orientation.z = 3.14;
			obstacles.obstacles[1].twist.linear.x = 1.0;
			obstacles.obstacles[1].pose.orientation.z = fmin(0.25,obstacles.obstacles[1].pose.orientation.z + 0.01);
		}
		else if((iter >= 500) && (iter < 1000))
		{
			obstacles.obstacles[0].twist.linear.x = fmin(1.5,obstacles.obstacles[0].twist.linear.x + 0.01);
			obstacles.obstacles[0].pose.orientation.z = fmax(1.25,obstacles.obstacles[0].pose.orientation.z - 0.01);
			obstacles.obstacles[1].twist.linear.x = fmax(0.5,obstacles.obstacles[1].twist.linear.x - 0.01);
			obstacles.obstacles[1].pose.orientation.z = fmin(2.5,obstacles.obstacles[1].pose.orientation.z + 0.01);
		}
		else if(iter == 1000)
		{	
			break;
		}
		ROS_INFO_STREAM("Iteration: " << iter );
	}
	return 0;	
}
