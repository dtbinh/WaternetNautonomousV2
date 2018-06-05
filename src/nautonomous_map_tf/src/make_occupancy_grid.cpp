#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nautonomous_pose_msgs/PointWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <cmath>

std::string turtle_name;

sensor_msgs::Imu imu_data;

tf::Quaternion q;

double start_time;

void utm_cb(const nautonomous_pose_msgs::PointWithCovarianceStamped::ConstPtr& msg){
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(msg->point.x, msg->point.y, 0.0) );
	transform.setRotation(tf::createQuaternionFromYaw(tf::getYaw(q) * -1 + 1.5 + 0.005 * fmax(0, (ros::Time::now().toSec()-start_time-85)) - 0.005 * fmax(0, (ros::Time::now().toSec()-start_time-175))));
	tf::quaternionMsgToTF(imu_data.orientation,q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "lidar_link"));
}

void imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
	imu_data = *msg;
}


int main(int argc, char** argv){
	ros::init(argc, argv, "tf_broadcaster");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

	start_time = ros::Time::now().toSec();
	ros::Subscriber sub_utm = nh.subscribe<nautonomous_pose_msgs::PointWithCovarianceStamped>("/utm", 10, utm_cb);
	ros::Subscriber sub_imu = nh_private.subscribe<sensor_msgs::Imu>("/sensor/imu/imu", 10, imu_cb);

	ros::spin();
	return 0;
};
