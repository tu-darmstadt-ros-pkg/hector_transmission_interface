#ifndef HECTOR_TRANSMISSION_INTERFACE__ADJUSTABLE_OFFSET_MANAGER_HPP_
#define HECTOR_TRANSMISSION_INTERFACE__ADJUSTABLE_OFFSET_MANAGER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hector_transmission_interface/adjustable_offset_transmission.hpp"
#include "hector_transmission_interface_msgs/srv/adjust_transmission_offsets.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace hector_transmission_interface
{
class AdjustableOffsetManager
{
public:
  using PositionGetter = std::function<double()>;

  struct ManagedJoint {
    std::shared_ptr<AdjustableOffsetTransmission> state_transmission;
    std::shared_ptr<AdjustableOffsetTransmission> command_transmission;
    PositionGetter position_getter;
  };

  /**
   * @brief Construct a new Adjustable Offset Manager
   * @param node Shared pointer to a node for service creation and logging
   */
  explicit AdjustableOffsetManager( rclcpp::Node::SharedPtr node );

  /**
   * @brief Adds a joint to the manager if it uses AdjustableOffsetTransmissions
   * @param name Name of the joint
   * @param state_tx The state transmission instance
   * @param command_tx The command transmission instance
   * @param position_getter A functional callback returning the current joint position
   */
  void add_joint( const std::string &name,
                  std::shared_ptr<transmission_interface::Transmission> state_tx,
                  std::shared_ptr<transmission_interface::Transmission> command_tx,
                  PositionGetter position_getter );

private:
  void handle_service(
      const std::shared_ptr<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets::Request>
          request,
      const std::shared_ptr<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets::Response>
          response );

  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets>::SharedPtr service_;
  std::unordered_map<std::string, ManagedJoint> managed_joints_;
};

} // namespace hector_transmission_interface

#endif // HECTOR_TRANSMISSION_INTERFACE__ADJUSTABLE_OFFSET_MANAGER_HPP_
