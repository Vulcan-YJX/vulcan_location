#ifndef GPS_COORDINATE_HPP__
#define GPS_COORDINATE_HPP__

#include <iostream>
#include <memory>
#include <string>

#include "SerialPort.hpp"
#include "gps_protocol.hpp"

class gps_coordinate
{
private:
  std::shared_ptr<VulcanSerial::SerialPort> SerialPortPtr;
  const std::string gps_port = "/dev/ttyUSB0";
  VulcanSerial::BaudRate rate = VulcanSerial::BaudRate::B_9600;
  /* data */
public:
  gps_coordinate(/* args */);
  ~gps_coordinate();
};

#endif /*GPS_COORDINATE_HPP__*/
