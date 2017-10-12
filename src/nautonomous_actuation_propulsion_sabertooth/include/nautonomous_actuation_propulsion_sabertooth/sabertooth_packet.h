/**
    Sabertooth Packet
    sabertooth_packet.h
    Purpose: Common functions for handling packets for the sabertooth motor serial packets

    /author Daan Zeeuwe
    /version 1.0 8/7/17 
*/

#ifndef SABERTOOTHPACKET_H_
#define SABERTOOTHPACKET_H_

#include <stdint.h>

class SabertoothPacket
{
    private:

        const uint8_t address_index_        = 0;
        const uint8_t command_index_        = 1;
        const uint8_t value_index_          = 2;
        const uint8_t checksum_index_       = 3;

        const static uint8_t packet_size_   = 4;

        uint8_t packet_[packet_size_];

        /**
        * Set the checksum of the packet by combining the fields
        * 0b01111111 checksum constant to prevent overflow to the 8th bit.
        */
        void setChecksum();

    public:
        
        enum SabertoothCommand
        {
            Motor1Forward = 0,
            Motor1Backward = 1,

            Motor2Forward = 4,
            Motor2Backward = 5,

            MixedForward = 8,
            MixedBackward = 9,

            MixedRight = 10,
            MixedLeft = 11,

            SerialTimeout = 14,
            Ramping = 16
        };

        SabertoothPacket();
        
        /**
        * Create test message for sabertooth
        * /param uint8_t command
        * /param uint8_t value
        */
        void fillPacket(uint8_t command, uint8_t value);

        void setAddress(uint8_t address);

        uint8_t getAddress();
        uint8_t getCommand();
        uint8_t getValue();
        uint8_t getChecksum();

        uint8_t* getPacket();

        uint8_t size();

};

#endif // SABERTOOTHPACKET_H_
