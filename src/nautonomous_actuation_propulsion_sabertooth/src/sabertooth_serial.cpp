/**
    Sabertooth Serial
    sabertooth_serial.cpp
    Purpose: High level serial module for the sabertooth motor.

    @author Daan Zeeuwe
    @version 1.0 8/7/17 
*/

#include <nautonomous_actuation_propulsion_sabertooth/sabertooth_serial.h>

SabertoothSerial::SabertoothSerial(ros::NodeHandle private_node_handle)
{
    private_node_handle.param("serial_port", serial_port_, std::string("/dev/nautonomous/actuation"));
    private_node_handle.param("serial_baud", serial_baud_, 115200);
    private_node_handle.param("serial_timeout", serial_timeout_, 250);
    private_node_handle.param("actuation_serial_available_param", serial_available_, true);

    initialize();
}

SabertoothSerial::~SabertoothSerial()
{
    if(propulsion_serial_)
	{
		propulsion_serial_->close();
		delete propulsion_serial_;
		propulsion_serial_ = nullptr;
	}
}

/**
 *\brief  Initialized the serial connection, set the used port and bautrate.
 *\params void
 *\return if the actuation serial is open or not
 */
bool SabertoothSerial::initSerial() 
{   
        bool port_open = false;

        if (serial_available_)
        {
            try
            {
		        propulsion_serial_ = new serial::Serial(serial_port_, serial_baud_,
		            serial::Timeout::simpleTimeout(serial_timeout_));

                port_open = propulsion_serial_->isOpen();
            }
            catch(serial::IOException ex)
            {
                ROS_ERROR("IOException opening serial port: %s", ex.what());
                port_open = false;
                propulsion_serial_ = nullptr;
            }
        } 
        else 
        {
            propulsion_serial_ = nullptr;
        }

		return port_open;
}

/**
 * Init the serial port for the motors
 * Returns true if the serial connection was opened.
 */
bool SabertoothSerial::initialize()
{

    if(initSerial())
    {
        return 1;
    } 
    else if(serial_available_)
    {
        ROS_ERROR("Could not init Actuation");
        exit(0);
    } 
    else
    {
        ROS_WARN("Serial not used"); // TODO use visualization when serial is not available.
    }

	return 0;
}

/**
Check if the propulsion serial attribute exists, if so try to open the serial port.
 */
bool SabertoothSerial::isOpen() 
{
    if(propulsion_serial_){
        return propulsion_serial_->isOpen();
    }
	return false;
}

/**
 Read the status of actuation platform by reading one byte from it.
 */
std::string SabertoothSerial::readStatus()
{
    std::string response;

    if(isOpen())
    {
        propulsion_serial_->read(response, 1); //status mesage only has 1 byte
    }

    return response;
}


/**
 If the actuation platform is connected and open, write the packet given to the serial port.
 */
void SabertoothSerial::writePacket(SabertoothPacket packet)
{
    if(isOpen())
    {
        propulsion_serial_->write(&packet.getPacket()[0], packet.size()); // Packet has 4 arguments.
    }
}

/**
 If the serial port is open flush the input, so we can read the next input next time.
 */
void SabertoothSerial::flushInput()
{
    if(isOpen())
    {
        propulsion_serial_->flushInput();
    }
}
