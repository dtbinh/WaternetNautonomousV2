#include <ros/ros.h>


class PVector
{
	private:
		float x, y;

	public:
		PVector (float x_, float y_);
		float getX();
		float getY();
};

PVector::PVector (float x_, float y_)
{
	x = x_;
	y = y_;
}

float PVector::getX()
{
	return x;
}

float PVector::getY()
{
	return y;
}

