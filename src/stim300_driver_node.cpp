#include "driver_stim300.h"
#include "driver_arduino.h"
#include "serial_unix.h"
#include <serial/serial.h>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"


constexpr int defaultSampleRate{ 125 };
constexpr double averageAllanVarianceOfGyro{ 0.0001 * 2 * 0.00046};
constexpr double averageAllanVarianceOfAcc{ 100 * 2 * 0.0052};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stim300_driver_node");

  ros::NodeHandle node;

  std::string stim_port;
  std::string arduino_port;
  double stanardDeivationOfGyro{ 0 };
  double stanardDeviationOfAcc{ 0 };
  double varianceOfGyro{ 0 };
  double varianceOfAcc{ 0 };
  int sampleRate{ 0 };

  // node.param<std::string>("stim_port", stim_port, "/dev/ttyUSB0");
  // SerialUnix serial_stim(stim_port);
  // DriverStim300 driver_stim300(serial_stim);

  node.param<std::string>("arduino_port", arduino_port, "/dev/ttyUSB1");
  SerialUnix serial_arduino(arduino_port); // THIS MAY NOT BE THE ACTUAL PORT
  DriverArduino driver_arduino(serial_arduino);

  serial::Serial ser;
  // ser.setPort("/dev/ttyACM0");
  // ser.setBaudrate(115200);
  // serial::Timeout to = serial::Timeout::simpleTimeout(1500);
  // ser.setTimeout(to);
  // ser.open();


  node.param("stanard_deviation_of_gyro", stanardDeivationOfGyro, averageAllanVarianceOfGyro);
  node.param("stanard_deviation_of_acc", stanardDeviationOfAcc, averageAllanVarianceOfAcc);
  node.param("sample_rate", sampleRate, defaultSampleRate);
  varianceOfGyro = sampleRate * pow(stanardDeivationOfGyro, 2);
  varianceOfAcc = sampleRate * pow(stanardDeviationOfAcc, 2);


  sensor_msgs::Imu imu_msg_template{};
  imu_msg_template.orientation_covariance[0] = -1;
  imu_msg_template.angular_velocity_covariance[0] = varianceOfGyro;
  imu_msg_template.angular_velocity_covariance[4] = varianceOfGyro;
  imu_msg_template.angular_velocity_covariance[8] = varianceOfGyro;
  imu_msg_template.linear_acceleration_covariance[0] = varianceOfAcc;
  imu_msg_template.linear_acceleration_covariance[4] = varianceOfAcc;
  imu_msg_template.linear_acceleration_covariance[8] = varianceOfAcc;
  imu_msg_template.orientation.x = 0;
  imu_msg_template.orientation.y = 0;
  imu_msg_template.orientation.z = 0;
  imu_msg_template.header.frame_id = "imu_0";

  ros::Publisher imuSensorPublisher = node.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);

  ros::Rate loop_rate(250);


  ROS_INFO("STIM300 IMU initialized successfully");driver_arduino.sendCurrentTimeNSec(ros::Time::now());
  while (ros::ok())
  {
    driver_arduino.testArduinoCom();
    loop_rate.sleep();
  }

  // std::string read;
  // std::string port{ "/dev/ttyACM0" };
  // std::string input;

  // while (ros::ok())
  // {
  // try
  //   {
  //     ROS_WARN("Tries...");
  //     if (ser.isOpen())
  //     {
  //       ROS_WARN("ser is open");
  //       // read string from serial device
  //       if(ser.available())
  //       {
  //         read = ser.read(ser.available());
  //         ROS_DEBUG("read %i new characters from serial port, adding to %i characters of old input.", (int)read.size(), (int)input.size());
  //         input += read;
  //         break;
  //       //   while (input.length() >= 36) // while there might be a complete package in input
  //       //   {
  //       // }
  //     }
  //     else
  //     {
  //       // try and open the serial port
  //       try
  //       {
  //         ser.close();
  //         ser.setPort(port);
  //         ser.setBaudrate(115200);
  //         serial::Timeout to = serial::Timeout::simpleTimeout(1500);
  //         ser.setTimeout(to);
  //         ser.open();
  //       }
  //       catch (serial::IOException& e)
  //       {
  //         ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << ". Trying again in 5 seconds.");
  //         ros::Duration(5).sleep();
  //       }

  //       if(ser.isOpen())
  //       {
  //         ROS_DEBUG_STREAM("Serial port " << ser.getPort() << " initialized and opened.");
  //       }
  //     }
  //     }
  //   }
  //   catch (serial::IOException& e)
  //   {
  //     ROS_ERROR_STREAM("Error reading from the serial port " << ser.getPort() << ". Closing connection.");
  //     ser.close();
  //   }

  // while (ros::ok())
  // {
  //   driver_arduino.processPacket();
  // //   sensor_msgs::Imu stim300msg = imu_msg_template;

  // //   stim300msg.header.stamp = ros::Time::now();

  // //   if (driver_stim300.processPacket())
  // //   {
  // //     if (!driver_stim300.isChecksumGood())
  // //     {
  // //       ROS_WARN("stim300 CRC error ");
  // //       continue;
  // //     }

  // //     if (!driver_stim300.isSensorStatusGood())
  // //     {
  // //       ROS_WARN("STIM300: Internal hardware error");
  // //       continue;
  // //     }

  // //     stim300msg.linear_acceleration.x = driver_stim300.getAccX() + 0.0023;
  // //     stim300msg.linear_acceleration.y = driver_stim300.getAccY() + 0.05;
  // //     stim300msg.linear_acceleration.z = driver_stim300.getAccZ() + 0.027;
  // //     stim300msg.angular_velocity.x = driver_stim300.getGyroX();
  // //     stim300msg.angular_velocity.y = driver_stim300.getGyroY();
  // //     stim300msg.angular_velocity.z = driver_stim300.getGyroZ();
  // //     imuSensorPublisher.publish(stim300msg);

  // //   }

  //    loop_rate.sleep();

  //    ros::spinOnce();
  // }
  //}
  // ROS_WARN_STREAM("Received " << input << " from Arduino.");
  return 0;
}