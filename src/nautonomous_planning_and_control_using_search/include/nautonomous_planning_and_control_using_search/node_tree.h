#include <iostream>
#include <vector>
#define INF 1000000

class node
{
	private:
		float pos_x;
		float pos_y;
		float theta;
		float dist_to_finish;
		float cost;
		float total_cost;
		int connected_node_prev;
		std::vector<int> connected_nodes_next;
		int node_nr;
		bool connected;


	public:
		node();

		void initializeNode(float x_, float y_, float theta_, float dist_to_finish_, float cost_, int connected_node_prev_, int node_, bool connected_);

		void addConnectedNode(int connected_node_nr_);

		void setCost(float cost_, int connected_node_prev_);
		
		void setDistToFinishToINF();
		float getX();
		float getY();
		float getTheta();
		float getCost();
		float getTotalCost();
		int getNode();
		float getDistToFinish();
		bool isConnected();
		int getPreviousNode();
		std::vector<int> getConnectedNodes();

};

node::node()
{
	pos_x = 0.0;
	pos_y = 0.0;
	theta = 0.0;
	dist_to_finish = 0.0;
	cost = 0.0;
	total_cost = 0.0;
	connected_node_prev = 0;
	node_nr = 0;
	connected = false;

}

void node::initializeNode(float x_, float y_, float theta_, float dist_to_finish_, float cost_, int connected_node_prev_, int node_, bool connected_)
{
	pos_x = x_;
	pos_y = y_;
	theta = theta_;
	dist_to_finish = dist_to_finish_;
	cost = cost_;
	total_cost = cost_ + dist_to_finish;
	connected_node_prev = connected_node_prev_;
	node_nr = node_;
	connected = connected_;
}

void node::addConnectedNode(int connected_node_nr_)
{
	connected_nodes_next.push_back(connected_node_nr_);
	connected = true;
}

void node::setDistToFinishToINF()
{
	dist_to_finish = INF;
	total_cost = INF;
}

float node::getX()
{
	return pos_x;
}

float node::getY()
{
	return pos_y;
}

float node::getTheta()
{
	return theta;
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

float node::getDistToFinish()
{
	return dist_to_finish;
}

bool node::isConnected()
{
	return connected;
}

std::vector<int> node::getConnectedNodes()
{
	return connected_nodes_next;
}

int node::getPreviousNode()
{
	return connected_node_prev;
}

void node::setCost(float cost_, int connected_node_prev_)
{
	cost = cost_;
	total_cost = cost_ + dist_to_finish;
	connected_node_prev = connected_node_prev_;
}
