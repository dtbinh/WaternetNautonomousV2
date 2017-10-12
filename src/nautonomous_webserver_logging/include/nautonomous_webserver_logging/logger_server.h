#ifndef LOGGER_SERVER_H_
#define LOGGER_SERVER_H_


#include <fstream>
#include <math.h>
#include <stdio.h>
#include <sstream>
#include <string>
#include <openssl/sha.h>
#include <unordered_map>

#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticStatus.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <gps_common/conversions.h>

#include <nautonomous_logging_msgs/LogGPS.h>
#include <nautonomous_logging_msgs/LogIMU.h>
#include <nautonomous_logging_msgs/LogPropulsionStatus.h>
#include <nautonomous_webserver_msgs/NautonomousAuthentication.h>
#include <nautonomous_authentication_msgs/Authentication.h>
#include <nautonomous_authentication_msgs/Encryption.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/Imu.h>

#define SECRET_FILE_PARAM "/logger_server/secret_file_location"

class LoggerServer{
    public:
        LoggerServer();
        ~LoggerServer();
        void run();
        
    private:
        ros::NodeHandle nh_;
        ros::Subscriber authSub;
        ros::Subscriber gpsMsgSub;
        ros::Subscriber imuMsgSub;
        ros::Subscriber robotPoseEkfOdomMsgSub;
        ros::Subscriber propulsionStatusMsgSub;
        ros::Publisher gpsLogPub;
        ros::Publisher imuLogPub;
        ros::Publisher propulsionStatusLogPub;
        ros::ServiceClient msgAuthenticationClient;
        ros::ServiceClient generateMacClient;

        // UTM zone
        std::string utmZone;

        // check if coords 
        bool fill_initial_coord;
        double initial_lat, initial_lon;

        // map with connected rosinstances name/mac pairs
        std::unordered_map<std::string, std::string> rosInstances;

        // declare logmsgs to send
        nautonomous_logging_msgs::LogGPS gps_msg;
        geometry_msgs::Quaternion imu_orientation;
        nautonomous_logging_msgs::LogIMU imu_msg;
        nautonomous_logging_msgs::LogPropulsionStatus propulsion_status_msg;

        //declare service messages
        nautonomous_authentication_msgs::Authentication authentication_srv;
        nautonomous_authentication_msgs::Encryption mac_generation_srv;

        // declare callbacks for each subscriber
        void storeNautonomousAuthentication(const nautonomous_webserver_msgs::NautonomousAuthenticationConstPtr&);
        void gpsDataCb(const sensor_msgs::NavSatFixConstPtr&);
        void imuDataCb(const sensor_msgs::ImuConstPtr&);
        void robotposeEkfOdomDataCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr&);
        void propulsionStatusCb(const diagnostic_msgs::DiagnosticStatusConstPtr&);
        
        // booleans to check before publishing
        bool sendLogMsgs;
        bool sendGPSMsg;
        bool sendIMUMsg;
        bool sendPropulsionStatusMsg;

        // variables for logging intervals
        bool startGPSTimer;
        bool startIMUTimer;
        double gpsStartTimeInSec;
        double imuStartTimeInSec;
        double loggingIntervalInSec;
};


#endif /* LOGGER_SERVER_H_ */
