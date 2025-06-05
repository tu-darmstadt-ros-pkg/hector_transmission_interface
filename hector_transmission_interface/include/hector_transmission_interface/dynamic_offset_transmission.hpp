//
// Created by aljoscha-schmidt on 6/4/25.
//

#ifndef DYNAMIC_OFFSET_TRANSMISSION_HPP
#define DYNAMIC_OFFSET_TRANSMISSION_HPP

#include <hector_transmission_interface_msgs/srv/set_transmission_offset.hpp>
#include <transmission_interface/simple_transmission.hpp>

namespace hector_transmission_interface
{
class DynamicOffsetTransmission : public transmission_interface::SimpleTransmission
{
  explicit DynamicOffsetTransmission( const rclcpp::Node::SharedPtr node,
                                      const std::string &storage_path,
                                      const double joint_to_actuator_reduction,
                                      const double joint_offset = 0.0 );
  bool setTransmissionOffset(
      const hector_transmission_interface_msgs::srv::SetTransmissionOffset::Request::SharedPtr request,
      hector_transmission_interface_msgs::srv::SetTransmissionOffset::Response::SharedPtr response );
};
} // namespace hector_transmission_interface
#endif // DYNAMIC_OFFSET_TRANSMISSION_HPP
