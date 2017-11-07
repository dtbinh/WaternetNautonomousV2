#include <ros/ros.h>


class Obstacle
{
	private:
		float x, y, theta, a, b;

	public:
		Obstacle (float x_, float y_, float theta_, float a_, float b_);
		float getX();
		float getY();
		float getAngle();
		float getA();
		float getB();
};

Obstacle::Obstacle (float x_, float y_, float theta_, float a_, float b_)
{
	x = x_;
	y = y_;
	theta = theta_;
	a = a_;
	b = b_;
}

float Obstacle::getX()
{
	return x;
}

float Obstacle::getY()
{
	return y;
}

float Obstacle::getAngle()
{
	return theta;
}

float Obstacle::getA()
{
	return a;
}
float Obstacle::getB()
{
	return b;
}

