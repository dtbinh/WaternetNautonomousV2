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

double toEulerAngle(geometry_msgs::Quaternion q)
{
	/*double roll;
	
	// roll (x-axis rotation)
	double sinr = +2.0 * (q.w * q.x + q.y * q.z);
	double cosr = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
	roll = atan2(sinr, cosr);

	double pitch;

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w * q.y - q.z * q.x);
	if (fabs(sinp) >= 1)
	{
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	}
	else
	{
		pitch = asin(sinp);
	}
*/
	double yaw;

	// yaw (z-axis rotation)
	double siny = +2.0 * (q.w * q.z + q.x * q.y);
	double cosy = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
	yaw = atan2(siny, cosy);

	return yaw;
}
