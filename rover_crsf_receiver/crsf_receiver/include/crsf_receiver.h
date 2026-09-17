#ifndef CRSF_RECEIVER_HPP
#define CRSF_RECEIVER_HPP

#include <vector>


#include "rclcpp/rclcpp.hpp"

// #include "termios2_redefines.h"
#include "serial_driver/serial_driver.hpp"
#include "crsf_parser.h"
#include "crsf_receiver_msg/msg/crsf_channels16.hpp"
#include "crsf_receiver_msg/msg/crsf_link_info.hpp"


using drivers::serial_driver::SerialDriver;
using drivers::serial_driver::SerialPortConfig;
using drivers::serial_driver::FlowControl;
using drivers::serial_driver::Parity;
using drivers::serial_driver::StopBits;
using drivers::common::IoContext;

using namespace std::chrono_literals;
using namespace crsf_receiver_msg::msg;


class CrsfReceiverNode: public rclcpp::Node
{
public:
  explicit CrsfReceiverNode();
  ~CrsfReceiverNode();
  
private:
  std::unique_ptr<IoContext>     serial_io_context;
  std::unique_ptr<SerialDriver>  serial_driver;

  CrsfParser parser;

  std::string device;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<CRSFChannels16>::SharedPtr channels_publisher;
  rclcpp::Publisher<CRSFLinkInfo>::SharedPtr link_publisher;

  void main_timer_callback();
  void receive_callback(const std::vector<uint8_t> & buffer, const size_t & bytes_transferred);
};


#endif 