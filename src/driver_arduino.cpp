#include "driver_arduino.h"

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
    serial_driver_.writeBytes(current_time_ns, 8);
}

void DriverArduino::processPacket()
{

    uint8_t byte;
    while (serial_driver_.readByte(byte))
    {
        ROS_ERROR("%i", byte);
        // buffer_.push_back(byte);
    }
}

void DriverArduino::testArduinoCom()
{
    uint8_t byte = 2;
    serial_driver_.writeByte(char(byte));
    ROS_ERROR("PC sent: %d", byte);

    uint8_t ret;
    serial_driver_.readByte(ret);
    ROS_ERROR("Arduino returned: %d", ret);
}