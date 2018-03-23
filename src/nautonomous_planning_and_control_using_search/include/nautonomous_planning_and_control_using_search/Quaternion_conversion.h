#include <cmath>

geometry_msgs::Quaternion quaternion;

geometry_msgs::Quaternion toQuaternion(double pitch, double roll, double yaw)
{
        // Abbreviations for the various angular functions
	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);

	quaternion.w = cy * cr * cp + sy * sr * sp;
	quaternion.x = cy * sr * cp - sy * cr * sp;
	quaternion.y = cy * cr * sp + sy * sr * cp;
	quaternion.z = sy * cr * cp - cy * sr * sp;

	return quaternion;
}
