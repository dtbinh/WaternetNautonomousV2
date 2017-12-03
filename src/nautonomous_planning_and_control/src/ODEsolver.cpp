#include <nautonomous_planning_and_control/ODEsolver.h>

void Boat_model( const state_type &x , state_type &dxdt , double t )
{
	current_time = ros::Time::now().toSec();
	
	x0 = x[0] - (obstacle_x + obstacle_u * (t + current_time - start_time));
	x1 = x[1] - (obstacle_y + obstacle_v * (t + current_time - start_time));

	l = pow((cos(obstacle_th) * x0 + sin(obstacle_th) * x1)/obstacle_a,2) + pow((-sin(obstacle_th) * x0 + cos(obstacle_th) * x1)/obstacle_b,2) - 1;
	
	he11 = (pow(obstacle_a,2) - pow(obstacle_b,2)) * sin(obstacle_th) * cos(obstacle_th);
	he12 = pow(obstacle_a * cos(obstacle_th),2) + pow(obstacle_b * sin(obstacle_th),2);
	he21 = pow(obstacle_b * cos(obstacle_th),2) + pow(obstacle_a * sin(obstacle_th),2);

	if (t < 5)
	{
		gammaF =  (0.0792 * pow(t,3) - 0.02376  * pow(t,4) + 0.0019 * pow(t,5));	
		k1 =  0.5*(0.0792 * pow(t,3) - 0.02376  * pow(t,4) + 0.0019 * pow(t,5));	
	}
	else
	{
		gammaF = 1;
		k1 = 0.5;
	}

	k2 = k1;

	h1 = gammaF / (obstacle_a * obstacle_b) * (he11 * x0 - he12 * x1);
	h2 = gammaF / (obstacle_a * obstacle_b) * (he21 * x0 - he11 * x1);

    	dxdt[0] = h1 - k1 * x0 * l;
    	dxdt[1] = h2 - k2 * x1 * l;
}

void write_waypoints( const state_type &x , const double t )
{
	waypoints_x.push_back((double)x[0]+ obstacle_x)  ;
	waypoints_y.push_back((double)x[1]+ obstacle_y) ;
	length++;
}

void obstacle_cb( const nautonomous_mpc_msgs::Obstacle::ConstPtr& obstacle_msg )
{
	obstacle = *obstacle_msg;
	obstacle_x = obstacle.state.pose.position.x;
	obstacle_y = obstacle.state.pose.position.y;
	obstacle_a = obstacle.major_semiaxis;
	obstacle_b = obstacle.minor_semiaxis;
	obstacle_th = obstacle.state.pose.orientation.z;
	obstacle_u = obstacle.state.twist.linear.x;
	obstacle_v = obstacle.state.twist.linear.y;
	obstacle_w = obstacle.state.twist.angular.z;
}

void start_cb( const nautonomous_mpc_msgs::StageVariable::ConstPtr& start_msg )
{
	ros::Time begin = ros::Time::now();

	start_state = *start_msg;
	start_x = start_state.x;
	start_y = start_state.y;
	start_theta = start_state.theta;

	// Define initial values:
	// ----------------------
    	state_type x = {{ start_x, start_y }}; // initial conditions
    	integrate( Boat_model , x , t_start , t_end , 2.5 , write_waypoints );

	// Get the results:
	// ---------------	

	waypoint_marker.header.frame_id = "/my_frame";
	waypoint_marker.header.stamp = ros::Time::now();
	waypoint_marker.ns = "points_and_lines";
	waypoint_marker.action = visualization_msgs::Marker::ADD;
	waypoint_marker.pose.orientation.w = 1.0;
    	waypoint_marker.id = 0;
	waypoint_marker.type = visualization_msgs::Marker::POINTS;

	waypoint_marker.scale.x = 0.2;
	waypoint_marker.scale.y = 0.2;
	waypoint_marker.color.b = 1.0;
	obstacle_marker.color.g = 0.25;
	waypoint_marker.color.a = 1.0;

	for (int i = 0; i < length; i++)
	{
		cout << "Waypoint " << i << " : (" << waypoints_x[i]  << ", " << waypoints_y[i] << ")" << endl;
		p.x = waypoints_x[i] - obstacle_x;
      		p.y = waypoints_y[i] - obstacle_y;
	
      		waypoint_marker.points.push_back(p);

		route.waypoints.push_back(p);

    		marker_pub.publish(waypoint_marker);
	}
	
	obstacle_marker.header.frame_id = "/my_frame";
	obstacle_marker.header.stamp = ros::Time::now();
	obstacle_marker.ns = "points_and_lines";
	obstacle_marker.action = visualization_msgs::Marker::ADD;
	obstacle_marker.pose.orientation.w = 1.0;
    	obstacle_marker.id = 0;
	obstacle_marker.type = visualization_msgs::Marker::CYLINDER;

	obstacle_marker.scale.z = 0.5;
	obstacle_marker.color.b = 1.0;
	obstacle_marker.color.a = 1.0;

	obstacle_marker.scale.x = obstacle_a * 2;
	obstacle_marker.scale.y = obstacle_b * 2;

	obstacle_marker.pose.position.x = obstacle_x;
	obstacle_marker.pose.position.y = obstacle_y;

	q = toQuaternion(0,0,obstacle_th);

	obstacle_marker.pose.orientation = q;

	obstacle_marker.points.push_back(p);

    	marker_pub_2.publish(obstacle_marker);
	
	waypoint_pub.publish(route);
	
	waypoints_x.clear();	
	waypoints_y.clear();
	route.waypoints.clear();
	length = 0;

	ros::Time end = ros::Time::now();
	cout << "Elapsed time is: " << end-begin << endl;

}

void start_time_cb(const std_msgs::Float64::ConstPtr& start_time_msg)
{
	start_time = start_time_msg->data;
}

int main(int argc, char **argv)
{
	ros::init (argc, argv,"Route_generator");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");
//	ros::Duration(1).sleep();

	marker_pub = nh_private.advertise<visualization_msgs::Marker>("waypoint_marker", 10);
	marker_pub_2 = nh_private.advertise<visualization_msgs::Marker>("obstacle_marker", 10);
	waypoint_pub = nh_private.advertise<nautonomous_mpc_msgs::Route>("waypoint_route", 10);

	obstacle_sub = nh.subscribe<nautonomous_mpc_msgs::Obstacle>("/mission_coordinator/obstacle",10,obstacle_cb);
	start_sub = nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/mission_coordinator/start",10,start_cb);
	start_time_sub = nh.subscribe<std_msgs::Float64>("/mission_coordinator/start_time",10,start_time_cb);

//	ros::Duration(1).sleep();
	
	cout << "spin" <<endl;
	ros::spin();
	return 0;
}
