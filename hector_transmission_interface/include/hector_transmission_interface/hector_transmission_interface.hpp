#ifndef HECTOR_TRANSMISSION_INTERFACE_HECTOR_TRANSMISSION_INTERFACE_HPP
#define HECTOR_TRANSMISSION_INTERFACE_HECTOR_TRANSMISSION_INTERFACE_HPP

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

namespace hector_transmission_interface
{

class HectorTransmissionInterface : public rclcpp::Node
{
public:
  HectorTransmissionInterface();

private:
  //! @brief Sets up subscribers, publishers, etc. to configure the node
  void setup();

private:
};

} // namespace hector_transmission_interface

#endif // HECTOR_TRANSMISSION_INTERFACE_HECTOR_TRANSMISSION_INTERFACE_HPP
