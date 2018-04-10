#include <nautonomous_planning_and_control/ODEsolver.h>

double safety_margin = 0.5;

void Boat_model( const state_type &x , state_type &dxdt , double t )
{
	l = pow((cos(obstacle_th) * x[0] + sin(obstacle_th) * x[1])/obstacle_a,2) + pow((-sin(obstacle_th) * x[0] + cos(obstacle_th) * x[1])/obstacle_b,2) - 1;
	
	he11 = (pow(obstacle_a,2) - pow(obstacle_b,2)) * sin(obstacle_th) * cos(obstacle_th);
	he12 = pow(obstacle_a * cos(obstacle_th),2) + pow(obstacle_b * sin(obstacle_th),2);
	he21 = pow(obstacle_b * cos(obstacle_th),2) + pow(obstacle_a * sin(obstacle_th),2);

	if (t < 5)
	{
		gammaF = gamma_max * (0.0792 * pow(t,3) - 0.02376  * pow(t,4) + 0.0019 * pow(t,5));	
		k1 =  k_max*(0.0792 * pow(t,3) - 0.02376  * pow(t,4) + 0.0019 * pow(t,5));	
	}
	else
	{
		gammaF = gamma_max;
		k1 = k_max;
	}

	k2 = k1;

	h1 = gammaF / (obstacle_a * obstacle_b) * (he11 * x[0] - he12 * x[1]);
	h2 = gammaF / (obstacle_a * obstacle_b) * (he21 * x[0] - he11 * x[1]);

    	dxdt[0] = h1 - k1 * x[0] * l;
    	dxdt[1] = h2 - k2 * x[1] * l;
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
	obstacle_x = obstacle.pose.position.x;
	obstacle_y = obstacle.pose.position.y;
	obstacle_a = obstacle.major_semiaxis + safety_margin;
	obstacle_b = obstacle.minor_semiaxis + safety_margin;
	obstacle_th = obstacle.pose.orientation.z;
	obstacle_u = obstacle.twist.linear.x;
	obstacle_v = obstacle.twist.linear.y;
	obstacle_w = obstacle.twist.angular.z;
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
    	state_type x = {{ start_x - obstacle_x , start_y - obstacle_y }}; // initial conditions
    	integrate( Boat_model , x , t_start , t_end , 2.5 , write_waypoints );

	// Get the results:
	// ---------------	

	for (int i = 0; i < length; i++)
	{
		cout << "Waypoint " << i << " : (" << waypoints_x[i]  << ", " << waypoints_y[i] << ")" << endl;
		p.x = waypoints_x[i];
      		p.y = waypoints_y[i];

		route.waypoints.push_back(p);
	}
	
	waypoint_pub.publish(route);
	
	waypoints_x.clear();	
	waypoints_y.clear();
	route.waypoints.clear();
	length = 0;

	ros::Time end = ros::Time::now();
	cout << "Elapsed time is: " << end.toSec()-begin.toSec() << endl;

}


int main(int argc, char **argv)
{
	ros::init (argc, argv,"Route_generator");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");


	nh_private.getParam("k_max", k_max);
	nh_private.getParam("gamma_max", gamma_max);
	nh_private.getParam("final_time", t_end);
	nh_private.getParam("safety_margin", safety_margin);

	waypoint_pub = nh_private.advertise<nautonomous_mpc_msgs::Route>("waypoint_route", 10);

	obstacle_sub = nh.subscribe<nautonomous_mpc_msgs::Obstacle>("/mission_coordinator/obstacle",10,obstacle_cb);
	start_sub = nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/mission_coordinator/start",10,start_cb);
	
	cout << "spin" <<endl;
	ros::spin();
	return 0;
}
