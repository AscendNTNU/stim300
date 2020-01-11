#include "driver_arduino.h"
#include <sstream>
#include <ros/ros.h>

DriverArduino::DriverArduino(SerialDriver& serial_driver) : serial_driver_(serial_driver) {
    serial_driver_.open(SerialDriver::BAUDRATE::BAUD_115200);
}

DriverArduino::~DriverArduino()
{
    serial_driver_.close();
}

void DriverArduino::sendCurrentTimeNSec(ros::Time current_time)
{
    uint64_t current_time_ns = current_time.toNSec();
    uint8_t* current_time_ns_ptr = (uint8_t*)&current_time_ns;
    serial_driver_.writeBytes(current_time_ns_ptr, 8);
}

void DriverArduino::processPacket()
{

    uint8_t byte = 0;
    while (!serial_driver_.readByte(byte));
    while (serial_driver_.readByte(byte))
    {
        ROS_ERROR("%i", byte);
        // buffer_.push_back(byte);
    }
}

void DriverArduino::testArduinoCom()
{
    // uint8_t byte[sizeof(uint64_t)];
    uint32_t big = 4e+9;
    // uint32_t top = (big >> 32) & 0xFFFFFFFF;
    // uint32_t bottom = big & 0xFFFFFFFF;
    // uint8_t* byte = (uint8_t*)&big;
    // serial_driver_.writeBytes(byte, 8);
    // ROS_ERROR_STREAM("PC sent: " << big); //((big >> 32) & 0xFFFFFFFF) << big);
    // std::stringstream ss;
    uint64_t big_number = 10000000000050000001ULL;
    ROS_WARN_STREAM("big_number is " << big_number);
    for (uint8_t i = 0; i < 8; i++) {
        serial_driver_.writeByte((big_number) & 0xFF);
        big_number = (big_number >> 8);
        //serial_driver_.writeByte(14);

    }
    // ss << big_number;
    // for (char c : ss.str()) {
    //     serial_driver_.writeByte(c);
    // }
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // uint32_t ret;
    uint8_t test = 0;
    uint64_t buffer = 0;
    int i = 0;

    while(serial_driver_.readByte(test)) {
        ROS_ERROR_STREAM("Arduino returned: " << (int)test);

        buffer |= (uint64_t) test << ((uint64_t) (i*8));
        // buffer += test;
        i++;
    }
        ROS_ERROR_STREAM("Arduino returned total: " << buffer);
    // processPacket();
    // serial_driver_.readBytes((uint8_t&)ret, 8);
    // uint32_t top_ret = (ret >> 32) & 0xFFFFFFFF;
    // uint32_t bottom_ret = ret & 0xFFFFFFFF;
}