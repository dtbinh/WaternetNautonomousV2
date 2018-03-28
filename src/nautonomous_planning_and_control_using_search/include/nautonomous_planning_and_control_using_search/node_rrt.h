#include <iostream>
#include <vector>
#define INF 1000000

class node
{
	private:
		float pos_x;
		float pos_y;
		float theta;
		float cost;
		int connected_node_prev;
		int time_stamp;
		std::vector<int> connected_nodes_next;
		int node_nr;
		bool connected;


	public:
		node();

		void initializeNode(float x_, float y_, float theta_, float cost_, int connected_node_prev_, int node_, bool connected_, int time_stamp_);

		void addConnectedNode(int connected_node_nr_);

		void setCost(float cost_, int connected_node_prev_);
		
		float getX();
		float getY();
		float getTheta();
		float getCost();
		int getNode();
		int getTimeStamp();
		bool isConnected();
		int getPreviousNode();
		std::vector<int> getConnectedNodes();

};

node::node()
{
	pos_x = 0.0;
	pos_y = 0.0;
	theta = 0.0;
	cost = 0.0;
	connected_node_prev = 0;
	node_nr = 0;
	connected = false;
	time_stamp = 0.0;
}

void node::initializeNode(float x_, float y_, float theta_, float cost_, int connected_node_prev_, int node_, bool connected_, int time_stamp_)
{
	pos_x = x_;
	pos_y = y_;
	theta = theta_;
	cost = cost_;
	connected_node_prev = connected_node_prev_;
	node_nr = node_;
	connected = connected_;
	time_stamp = time_stamp_;
}

void node::addConnectedNode(int connected_node_nr_)
{
	connected_nodes_next.push_back(connected_node_nr_);
	connected = true;
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

int node::getNode()
{
	return node_nr;
}

bool node::isConnected()
{
	return connected;
}

int node::getTimeStamp()
{
	return time_stamp;
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
	connected_node_prev = connected_node_prev_;
}
