#include <iostream>
#include <vector>
#define INF 1000000

class node
{
	private:
		bool jump_point;
		bool target_jump_point;
		bool isObstacle;

		bool jump_dirs[4];
		bool target_dirs[8];
		bool point_dirs[4];
		int jump_steps[8];

		int cost;
		int dist_to_finish;
		int tot_cost;

	public:
		node();
	
		void setObstacle();

		void setTargetJumpPoint(int dir_);

		void setJumpPointLeft();
		void setJumpPointRight();
		void setJumpPointUp();
		void setJumpPointDown();

		void setPointLeft();
		void setPointRight();
		void setPointUp();
		void setPointDown();

		void setJumpLeft(int dist);
		void setJumpRight(int dist);
		void setJumpUp(int dist);
		void setJumpDown(int dist);
		void setJumpLeftUp(int dist);
		void setJumpLeftDown(int dist);
		void setJumpRightUp(int dist);
		void setJumpRightDown(int dist);

		void setCost(int cost_);
		void setDist(int dist_to_finish_);

		bool getJumpPoint();

		bool getTargetJumpPoint();
		bool getTargetJumpPoint(int dir_);

		bool getObstacle();

		bool getJumpPointLeft();
		bool getJumpPointRight();
		bool getJumpPointUp();
		bool getJumpPointDown();

		bool getPointLeft();
		bool getPointRight();
		bool getPointUp();
		bool getPointDown();

		int getJumpLeft();
		int getJumpRight();
		int getJumpUp();
		int getJumpDown();
		int getJumpLeftUp();
		int getJumpLeftDown();
		int getJumpRightUp();
		int getJumpRightDown();

		int getCost();
		int getDist();
		int getTotalCost();
};

node::node()
{
	jump_point = false;
	target_jump_point = false;
	isObstacle = false;
	cost = 0;
	dist_to_finish = 0;

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

	target_dirs[0] = false;
	target_dirs[1] = false;
	target_dirs[2] = false;
	target_dirs[3] = false;
	target_dirs[4] = false;
	target_dirs[5] = false;
	target_dirs[6] = false;
	target_dirs[7] = false;
}

void node::setObstacle()
{
	isObstacle = true;
}

void node::setCost(int cost_)
{
	cost = cost_;
	tot_cost = cost + dist_to_finish;
}

void node::setDist(int dist_to_finish_)
{
	dist_to_finish = dist_to_finish_;
	tot_cost = cost + dist_to_finish;
}

void node::setTargetJumpPoint(int dir_)
{
	target_jump_point = true;
	target_dirs[dir_] = true;
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

void node::setJumpLeft(int dist)
{
	jump_steps[3] = dist;
}

void node::setJumpRight(int dist)
{
	jump_steps[4] = dist;
}

void node::setJumpUp(int dist)
{
	jump_steps[1] = dist;
}

void node::setJumpDown(int dist)
{
	jump_steps[6] = dist;
}

void node::setJumpLeftUp(int dist)
{
	jump_steps[0] = dist;
}

void node::setJumpLeftDown(int dist)
{
	jump_steps[5] = dist;
}

void node::setJumpRightUp(int dist)
{
	jump_steps[2] = dist;
}

void node::setJumpRightDown(int dist)
{
	jump_steps[7] = dist;
}

bool node::getJumpPoint()
{
	return jump_point;
}

bool node::getTargetJumpPoint()
{
	return target_jump_point;
}

bool node::getTargetJumpPoint(int dir_)
{
	return target_dirs[dir_];
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

int node::getJumpLeft()
{
	return jump_steps[3];
}

int node::getJumpRight()
{
	return jump_steps[4];
}

int node::getJumpUp()
{
	return jump_steps[1];
}

int node::getJumpDown()
{
	return jump_steps[6];
}

int node::getJumpLeftUp()
{
	return jump_steps[0];
}

int node::getJumpLeftDown()
{
	return jump_steps[5];
}

int node::getJumpRightUp()
{
	return jump_steps[2];
}

int node::getJumpRightDown()
{
	return jump_steps[7];
}

int node::getCost()
{
	return cost;
}

int node::getDist()
{
	return dist_to_finish;
}

int node::getTotalCost()
{
	return tot_cost;
}
