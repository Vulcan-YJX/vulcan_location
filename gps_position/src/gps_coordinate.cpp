#include "gps_coordinate.hpp"

gps_coordinate::gps_coordinate(/* args */)
{
  SerialPortPtr = std::make_shared<VulcanSerial::SerialPort>();
  SerialPortPtr->SetDevice(gps_port.c_str());
  SerialPortPtr->SetBaudRate(rate);
  SerialPortPtr->SetNumDataBits(VulcanSerial::NumDataBits::EIGHT);
  SerialPortPtr->SetNumStopBits(VulcanSerial::NumStopBits::ONE);
  SerialPortPtr->Open();
}

gps_coordinate::~gps_coordinate() { SerialPortPtr->Close(); }
