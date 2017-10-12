// #include <nautonomous_actuation_propulsion/test/sabertooth_test.h>

// TEST(SabertoothBasicSuite, testMessage)
// {
//     uint8_t message[4];
//     uint8_t address = 128;
//     uint8_t command = 0;
//     uint8_t value = 100;
    
//     sabertooth_test_message(address, command, value, message);

//     ASSERT_EQ(message[0], address);
//     ASSERT_EQ(message[1], command);
//     ASSERT_EQ(message[2], value);
// }

// TEST(SabertoothPropulsionSuite, moveStraight)
// {
//     uint8_t straightCommand[4], turnCommand[4];
//     geometry_msgs::Twist* propulsion = new geometry_msgs::Twist();

//     propulsion->linear.x = 1.1; //forward
//     propulsion->angular.z = 0.0; //no turn

//     ros::NodeHandle nh;

//     propulsion_address = (uint8_t) 128;

//     sabertooth_advanced_process_propulsion_twist(&straightCommand[0], &turnCommand[0], geometry_msgs::TwistConstPtr(propulsion));

//     ASSERT_EQ(straightCommand[0], 128);
//     ASSERT_EQ(straightCommand[1], 8); //forward
//     ASSERT_EQ(straightCommand[2], 127);

//     ASSERT_EQ(turnCommand[0], 128);
//     ASSERT_EQ(turnCommand[1], 10); //no turn
//     ASSERT_EQ(turnCommand[2], 0);
// }

// TEST(SabertoothPropulsionSuite, moveBackwards)
// {
//     uint8_t straightCommand[4], turnCommand[4];
//     geometry_msgs::Twist* propulsion = new geometry_msgs::Twist();

//     propulsion->linear.x = -1.1; //backwards
//     propulsion->angular.z = 0.0; //no turn

//     ros::NodeHandle nh;

//     propulsion_address = (uint8_t) 128;

//     sabertooth_advanced_process_propulsion_twist(&straightCommand[0], &turnCommand[0], geometry_msgs::TwistConstPtr(propulsion));

//     ASSERT_EQ(straightCommand[0], 128);
//     ASSERT_EQ(straightCommand[1], 9); //backwards
//     ASSERT_EQ(straightCommand[2], 127);

//     ASSERT_EQ(turnCommand[0], 128);
//     ASSERT_EQ(turnCommand[1], 10); //no turn
//     ASSERT_EQ(turnCommand[2], 0);
// }

// TEST(SabertoothPropulsionSuite, turnLeft)
// {
//     uint8_t straightCommand[4], turnCommand[4];
//     geometry_msgs::Twist* propulsion = new geometry_msgs::Twist();

//     propulsion->linear.x = 0.0; //no propulsion
//     propulsion->angular.z = 0.6; //left

//     ros::NodeHandle nh;

//     propulsion_address = (uint8_t) 128;

//     sabertooth_advanced_process_propulsion_twist(&straightCommand[0], &turnCommand[0], geometry_msgs::TwistConstPtr(propulsion));

//     ASSERT_EQ(straightCommand[0], 128);
//     ASSERT_EQ(straightCommand[1], 9); //no propulsion
//     ASSERT_EQ(straightCommand[2], 0);

//     ASSERT_EQ(turnCommand[0], 128);
//     ASSERT_EQ(turnCommand[1], 11); //left
//     ASSERT_EQ(turnCommand[2], 127);
// }

// TEST(SabertoothPropulsionSuite, turnRight)
// {
//     uint8_t straightCommand[4], turnCommand[4];
//     geometry_msgs::Twist* propulsion = new geometry_msgs::Twist();

//     propulsion->linear.x = 0.0; //no propulsion
//     propulsion->angular.z = -0.6; //left

//     ros::NodeHandle nh;

//     propulsion_address = (uint8_t) 128;

//     sabertooth_advanced_process_propulsion_twist(&straightCommand[0], &turnCommand[0], geometry_msgs::TwistConstPtr(propulsion));

//     ASSERT_EQ(straightCommand[0], 128);
//     ASSERT_EQ(straightCommand[1], 9); //no propulsion
//     ASSERT_EQ(straightCommand[2], 0);

//     ASSERT_EQ(turnCommand[0], 128);
//     ASSERT_EQ(turnCommand[1], 10); //left
//     ASSERT_EQ(turnCommand[2], 127);
// }

// int main(int argc, char **argv){
//     ros::init(argc, argv, "sabertooth_test_node");
//     testing::InitGoogleTest(&argc, argv);
//     return RUN_ALL_TESTS();
// }
