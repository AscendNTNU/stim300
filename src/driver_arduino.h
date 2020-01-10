#ifndef DRIVER_STIM300_DRIVER_ARDUINO_H
#define DRIVER_STIM300_DRIVER_ARDUINO_H

#include "ros/ros.h"

#include "../src/serial_driver.h"

#include <vector>
#include <ostream>


class DriverArduino
{
public:
    DriverArduino(SerialDriver& serial_driver);
    ~DriverArduino();
    void sendCurrentTimeNSec(ros::Time current_time);
    void processPacket();

    void testArduinoCom();

private:
    SerialDriver& serial_driver_;
    std::vector<uint8_t> buffer_;

};

#endif // DRIVER_STIM300_DRIVER_ARDUINO_H