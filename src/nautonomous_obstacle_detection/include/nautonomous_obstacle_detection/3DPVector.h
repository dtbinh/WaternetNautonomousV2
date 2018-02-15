#include <ros/ros.h>


class 3DPVector
{
	private:
		float x, y, z;

	public:
		PVector (float x_, float y_, float z_);
		float getX();
		float getY();
		float getZ();
};

PVector::PVector (float x_, float y_,float z_)
{
	x = x_;
	y = y_;
	z = z_;
}

float PVector::getX()
{
	return x;
}

float PVector::getY()
{
	return y;
}

float PVector::getZ()
{
	return z;
}

