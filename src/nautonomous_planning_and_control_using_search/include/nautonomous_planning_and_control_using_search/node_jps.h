#include <iostream>
#include <vector>
#define INF 1000000

class node
{
	private:
		bool jump_point;
		bool isObstacle;
		bool jump_dirs[4];
		bool point_dirs[4];
		int jump_steps[8];

	public:
		node();
	
		void setObstacle();

		void setJumpPointLeft();
		void setJumpPointRight();
		void setJumpPointUp();
		void setJumpPointDown();

		void setPointLeft();
		void setPointRight();
		void setPointUp();
		void setPointDown();

		bool getJumpPoint();

		bool getObstacle();

		bool getJumpPointLeft();
		bool getJumpPointRight();
		bool getJumpPointUp();
		bool getJumpPointDown();

		bool getPointLeft();
		bool getPointRight();
		bool getPointUp();
		bool getPointDown();
};

node::node()
{
	jump_point = false;
	isObstacle = false;
	jump_dirs[0] = false;
	jump_dirs[1] = false;
	jump_dirs[2] = false;
	jump_dirs[3] = false;
	
	point_dirs[0] = false;
	point_dirs[1] = false;
	point_dirs[2] = false;
	point_dirs[3] = false;

	jump_steps[0] = 0;
	jump_steps[1] = 0;
	jump_steps[2] = 0;
	jump_steps[3] = 0;
	jump_steps[4] = 0;
	jump_steps[5] = 0;
	jump_steps[6] = 0;
	jump_steps[7] = 0;
}

void node::setObstacle()
{
	isObstacle = true;
}

void node::setJumpPointLeft()
{
	jump_point = true;
	jump_dirs[0] = true;
}

void node::setJumpPointRight()
{
	jump_point = true;
	jump_dirs[1] = true;
}

void node::setJumpPointUp()
{
	jump_point = true;
	jump_dirs[2] = true;
}

void node::setJumpPointDown()
{
	jump_point = true;
	jump_dirs[3] = true;
}

void node::setPointLeft()
{
	point_dirs[0] = true;
}

void node::setPointRight()
{
	point_dirs[1] = true;
}

void node::setPointUp()
{
	point_dirs[2] = true;
}

void node::setPointDown()
{
	point_dirs[3] = true;
}

bool node::getJumpPoint()
{
	return jump_point;
}

bool node::getObstacle()
{
	return isObstacle;
}

bool node::getJumpPointLeft()
{
	return jump_dirs[0];
}

bool node::getJumpPointRight()
{
	return jump_dirs[1];
}

bool node::getJumpPointUp()
{
	return jump_dirs[2];
}

bool node::getJumpPointDown()
{
	return jump_dirs[3];
}

bool node::getPointLeft()
{
	return point_dirs[0];
}

bool node::getPointRight()
{
	return point_dirs[1];
}

bool node::getPointUp()
{
	return point_dirs[2];
}

bool node::getPointDown()
{
	return point_dirs[3];
}


