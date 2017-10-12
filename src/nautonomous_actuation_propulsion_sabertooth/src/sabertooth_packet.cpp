/**
    Sabertooth Packet
    sabertooth_packet.cpp
    Purpose: Common functions for handling packets for the sabertooth motor serial packets

    @author Daan Zeeuwe
    @version 1.0 8/7/17 
*/

#include <nautonomous_actuation_propulsion_sabertooth/sabertooth_packet.h>

/**
 * Constructor
 */
SabertoothPacket::SabertoothPacket()
{
	
}

/**
 * Create test message for sabertooth
 * @param uint8_t command
 * @param uint8_t value
 * @param uint8_t* packet
 */
void SabertoothPacket::fillPacket(uint8_t command, uint8_t value)
{
	packet_[command_index_] = command;
	packet_[value_index_] = value;

	setChecksum();
}

/**
 * Set the checksum of the packet by combining the fields
 * 0b01111111 checksum constant to prevent overflow to the 8th bit.
 * @param uint8_t* packet
 */
void SabertoothPacket::setChecksum()
{
    packet_[checksum_index_] = (packet_[address_index_] + packet_[command_index_] + packet_[value_index_]) & 0b01111111; 
}

/**
 Set the address of the packet
**/
void SabertoothPacket::setAddress(uint8_t address)
{
    packet_[address_index_] = address;
}

/**
 Get the address of the packet
**/
uint8_t SabertoothPacket::getAddress()
{
    return packet_[address_index_];
}

/**
 Set the command value of the packet
**/
uint8_t SabertoothPacket::getCommand()
{
    return packet_[command_index_];
}

/**
 Get the command value of the packet
**/
uint8_t SabertoothPacket::getValue()
{
    return packet_[value_index_];
}

/**
 Set the checksum of the packet
**/
uint8_t SabertoothPacket::getChecksum()
{
    return packet_[checksum_index_];
}

/**
 Get the entire packet.
**/
uint8_t* SabertoothPacket::getPacket()
{
    return packet_;
}

/**
 Get the size of the packet.
**/
uint8_t SabertoothPacket::size()
{
    return packet_size_;
}