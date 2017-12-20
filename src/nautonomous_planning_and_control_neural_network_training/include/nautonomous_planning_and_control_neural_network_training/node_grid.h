#include <iostream>
#include <vector>
#define INF 1000000

class node
{
	private:
		float pos_x;
		float pos_y;
		float dist_to_finish;
		float cost;
		float total_cost;
		int connected_node;
		int node_nr;

	public:
		node();

		void initializeNode(float x_, float y_, float f_x_, float f_y_, bool dist_to_finish_if_INF_, float cost_, int connected_node_, int node_);

		void setPreviousNode(float cost_, int connected_node_);

		float getX();
		float getY();
		float getCost();
		float getTotalCost();
		int getNode();
		int getDistToFinish();
		int getPreviousNode();
};

node::node()
{
	pos_x = 0.0;
	pos_y = 0.0;
	dist_to_finish = 0.0;
	cost = INF;
	total_cost = INF;
	connected_node = 0;
	node_nr = 0;
}

void node::initializeNode(float x_, float y_, float f_x_, float f_y_, bool dist_to_finish_is_INF_, float cost_, int connected_node_, int node_)
{
	pos_x = x_;
	pos_y = y_;
	if (dist_to_finish_is_INF_) {dist_to_finish = INF;}
	else {dist_to_finish = sqrt(pow(pos_x - f_x_,2) + pow(pos_y - f_y_,2));}
	cost = cost_;
	total_cost = cost_ + dist_to_finish;
	connected_node = connected_node_;
	node_nr = node_;
}

void node::setPreviousNode(float cost_, int connected_node_)
{
	cost = cost_;
	total_cost = cost_ + dist_to_finish;
	connected_node= connected_node_;
}

float node::getX()
{
	return pos_x;
}

float node::getY()
{
	return pos_y;
}

float node::getCost()
{
	return cost;
}

float node::getTotalCost()
{
	return total_cost;
}

int node::getNode()
{
	return node_nr;
}

int node::getDistToFinish()
{
	return dist_to_finish;
}
