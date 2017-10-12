#include <nautonomous_webserver_logging/logger_server.h>

using namespace ros;
using namespace std;

LoggerServer::LoggerServer()
{
    // Initiate subscribers
    this->authSub = this->nh_.subscribe("/webserver/authentication", 5, &LoggerServer::storeNautonomousAuthentication, this);
    this->gpsMsgSub = this->nh_.subscribe("gps_fix_topic", 5, &LoggerServer::gpsDataCb, this);
    this->imuMsgSub = this->nh_.subscribe("imu_topic", 5, &LoggerServer::imuDataCb, this);
    this->robotPoseEkfOdomMsgSub = this->nh_.subscribe("odom_combined_topic", 5, &LoggerServer::robotposeEkfOdomDataCb, this);
    this->propulsionStatusMsgSub = this->nh_.subscribe("actuation_watchdog_topic", 5, &LoggerServer::propulsionStatusCb, this);

    // Initiate publishers
    this->gpsLogPub = this->nh_.advertise<nautonomous_logging_msgs::LogGPS>("/webserver/logging/gps", 1);
    this->imuLogPub = this->nh_.advertise<nautonomous_logging_msgs::LogIMU>("/webserver/logging/imu", 1);
    this->propulsionStatusLogPub = this->nh_.advertise<nautonomous_logging_msgs::LogPropulsionStatus>("/webserver/logging/propulsion/status", 1);

    //initiate service clients
    this->msgAuthenticationClient = this->nh_.serviceClient<nautonomous_authentication_msgs::Authentication>("msg_auth");
    this->generateMacClient = this->nh_.serviceClient<nautonomous_authentication_msgs::Encryption>("mac_gen");

    // Set booleans for sending log msgs to false
    this->sendLogMsgs = false;
    this->sendGPSMsg = false;
    this->sendIMUMsg = false;
    this->sendPropulsionStatusMsg = false;

    // Set initial coord vars
    this->fill_initial_coord = true;
    this->initial_lat = 0.0;
    this->initial_lon = 0.0;

    // boolean for whether to start timers or not
    this->startGPSTimer = true;
    this->startIMUTimer = true;

    // set initial logging interval
    this->loggingIntervalInSec = 5.0;
}

LoggerServer::~LoggerServer()
{
}

/**
 * \brief Receives auth data from connected clients through ros_bridge, and store it
 * \params nautonomous_webserver_logging::IsAuthConstPtr &msg
 * \return
 */
void LoggerServer::storeNautonomousAuthentication(const nautonomous_webserver_msgs::NautonomousAuthenticationConstPtr &msg)
{
    //this is a map of clientname/mac pairs. First this will always be 1 entry, but this may become more when a fleet of nautonomous with 1 master is deployed.
    if (this->rosInstances.count(msg->nautonomousName.c_str()) == 0)
    {
        this->rosInstances.insert(std::make_pair<std::string, std::string>(msg->nautonomousName.c_str(), msg->acceptedMac.c_str()));
        ROS_INFO("Saving authentication mac for client.");
        this->sendLogMsgs = true;
    }
    else if (this->rosInstances.at(msg->nautonomousName.c_str()) == "")
    {
        this->rosInstances.at(msg->nautonomousName.c_str()) = msg->acceptedMac.c_str();
        ROS_INFO("This client is known, but not authenticated. Updating mac for client.");
        this->sendLogMsgs = true;
    }
    else
    {
        ROS_INFO("Error authenticating client.");
    }
}

/**
 * \brief Receives gps data and processes it to be sent to webserver
 * \params sensor_msgs::NavSatFixConstPtr& fix
 * \return
 */
void LoggerServer::gpsDataCb(const sensor_msgs::NavSatFixConstPtr &msg)
{
    if(this->fill_initial_coord){
        this->initial_lat = msg->latitude;
        this->initial_lon = msg->longitude;
        this->fill_initial_coord = false;
    }

    if ((msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX && (this->initial_lat == msg->latitude && this->initial_lon == msg->longitude)) || msg->header.stamp == ros::Time(0))
    {
        ROS_INFO("Received GPS message which won't be processed.");
        this->sendGPSMsg = false;
        return;
    }
    else
    {
        if (this->sendLogMsgs)
        {
            if (this->startGPSTimer)
            {
                this->gpsStartTimeInSec = ros::Time::now().toSec();
                this->startGPSTimer = false;
            }

            //only log to server every 5 sec
            if ((ros::Time::now().toSec() - this->gpsStartTimeInSec) >= this->loggingIntervalInSec)
            {
                this->gps_msg.header.stamp = msg->header.stamp;
                this->gps_msg.name = "Nautonomous-1";
                // create token from boat name + data to log and store in srv
                this->gps_msg.token = this->gps_msg.name + std::to_string(msg->latitude) + std::to_string(msg->longitude);
                this->mac_generation_srv.request.token = this->gps_msg.token;

                // service call that takes the token and generates a mac from it.
                if(this->generateMacClient.call(this->mac_generation_srv)){
                    this->gps_msg.mac = this->mac_generation_srv.response.mac;
                }

                this->gps_msg.latitude = msg->latitude;
                this->gps_msg.longitude = msg->longitude;

                ROS_INFO("Logging gps msg to server");
                this->sendGPSMsg = true;
                this->startGPSTimer = true;
            }
        }
    }
}

/**
 * \brief Receives IMU data and processes it to be sent to webserver
 * \params sensor_msgs::ImuConstPtr &msg
 * \return
 */
void LoggerServer::imuDataCb(const sensor_msgs::ImuConstPtr &msg)
{
    if (msg->header.stamp == ros::Time(0))
    {
        ROS_INFO("Received IMU message which won't be processed.");
        this->sendIMUMsg = false;
        return;
    }
    else
    {
        if (this->sendLogMsgs)
        {
            // Set starting time in sec when counter = 0
            if (this->startIMUTimer)
            {
                this->imuStartTimeInSec = ros::Time::now().toSec();
                this->startIMUTimer = false;
            }

            //only log to server every 5 sec
            if ((ros::Time::now().toSec() - this->imuStartTimeInSec) >= this->loggingIntervalInSec)
            {
                this->imu_msg.header.stamp = msg->header.stamp;
                this->imu_msg.name = "Nautonomous-1";

                // create token from boat name + data to log and store in srv
                this->imu_msg.token = this->imu_msg.name + std::to_string(msg->orientation.w) + std::to_string(msg->orientation.x) + 
                                      std::to_string(msg->orientation.y) + std::to_string(msg->orientation.z);
                this->mac_generation_srv.request.token = this->imu_msg.token;

                // service call that takes the token and generates a mac from it.
                if(this->generateMacClient.call(this->mac_generation_srv)){
                    this->imu_msg.mac = this->mac_generation_srv.response.mac;
                }

                this->imu_msg.orientation = msg->orientation;
                this->imu_msg.linear_acceleration = msg->linear_acceleration;

                ROS_INFO("Logging IMU msg to server");
                this->sendIMUMsg = true;
                this->startIMUTimer = true;
            }
        }

        // determine the logging interval in seconds, based on the angular velocity Z value. This value represents the amount at which the boat turns.
        // The interval is linear with the velocity Z value. 0-90 degrees is 1-5 sec. 
        double vel_z_degr = msg->angular_velocity.z * 180 / M_PI;

        // vel_z_degr will have a value between -90 and 90 (or -180 and 180), but this indicates the turning direction. The linearity applies to either direction
        if(vel_z_degr < 0){
            vel_z_degr = vel_z_degr * -1;
        }

        // we take 90 as maximum
        if(vel_z_degr > 90){
            vel_z_degr = 90;
        }
        
        // if turning speed is close to 0, a longer interval is used since it is mostly going straight forward.
        this->loggingIntervalInSec = 5 - ((vel_z_degr / 90) * 5) < 1 ? 1 : 5 - ((vel_z_degr / 90) * 5);
    }
}

/**
 * \brief Receives Pose data and processes it to be sent to webserver, this is mainly a test for now
 * \params geometry_msgs::PoseWithCovarianceStampedConstPtr &msg
 * \return
 */
void LoggerServer::robotposeEkfOdomDataCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
    double longitude = 0.0;
    double latitude = 0.0;
    std::string utmZone = "31U";
    gps_common::UTMtoLL(msg->pose.pose.position.y, msg->pose.pose.position.x, utmZone, latitude, longitude);

    //ROS_INFO("Gained LL coords are: longitude - %f, latitude - %f", longitude, latitude);
}

/**
 * \brief Receives the actuation platform's Status and sends it to the webserver
 * \params diagnostic_msgs::DiagnosticStatusConstPtr &msg
 * \return
 */
void LoggerServer::propulsionStatusCb(const diagnostic_msgs::DiagnosticStatusConstPtr &msg)
{

    if (this->sendLogMsgs)
    {
        this->propulsion_status_msg.header.stamp = ros::Time::now();
        this->propulsion_status_msg.name = "Nautonomous-1";
        this->propulsion_status_msg.mac = this->rosInstances.at("Nautonomous-1");
        this->propulsion_status_msg.level = msg->level;
        this->propulsion_status_msg.message = msg->message;
        ROS_INFO("Received watchdog message, logging to server.");

        this->sendPropulsionStatusMsg = true;
    }
}

/**
 * \brief custom spin function that only sends messages if booleans are true
 * \params 
 * \return
 */
void LoggerServer::run()
{
    ros::Rate rate(10);
    while (ros::ok())
    {
        if (this->sendLogMsgs)
        {
            if (this->sendGPSMsg)
            {
                this->gpsLogPub.publish(this->gps_msg);
                this->sendGPSMsg = false;
            }
            if (this->sendIMUMsg)
            {
                this->imuLogPub.publish(this->imu_msg);
                this->sendIMUMsg = false;
            }
            if (this->sendPropulsionStatusMsg)
            {
                this->propulsionStatusLogPub.publish(this->propulsion_status_msg);
                this->sendPropulsionStatusMsg = false;
            }
        }

        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "webserver_logger_node");

    LoggerServer server = LoggerServer();

    server.run();

    ros::shutdown();

    return 0;
}
