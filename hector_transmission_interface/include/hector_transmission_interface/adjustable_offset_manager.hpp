#ifndef HECTOR_TRANSMISSION_INTERFACE__ADJUSTABLE_OFFSET_MANAGER_HPP_
#define HECTOR_TRANSMISSION_INTERFACE__ADJUSTABLE_OFFSET_MANAGER_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "hector_transmission_interface/adjustable_offset_transmission.hpp"
#include "hector_transmission_interface_msgs/srv/adjust_transmission_offsets.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace hector_transmission_interface
{
/**
 * @brief Interface to allow mocking of offset adjustments regardless of
 * concrete transmission implementation.
 */
class IAdjustableOffset
{
public:
  virtual ~IAdjustableOffset() = default;
  virtual void adjustOffset( double offset ) = 0;
  virtual double getOffset() const = 0;
};

class AdjustableOffsetManager
{
public:
  using PositionGetter = std::function<double()>;
  using ServiceCallback = std::function<bool()>;

  struct ManagedJoint {
    std::shared_ptr<IAdjustableOffset> state_handle;
    std::shared_ptr<IAdjustableOffset> command_handle;
    PositionGetter position_getter;
  };

  /**
   * @brief Construct the manager
   * @param node The node to host the service
   * @param comm_mutex Optional reference to the hardware communication mutex
   * @param pre_callback Optional callback invoked before service processing (aborts if returns false)
   * @param post_callback Optional callback invoked after service processing
   */
  explicit AdjustableOffsetManager(
      rclcpp::Node::SharedPtr node,
      std::optional<std::reference_wrapper<std::mutex>> comm_mutex = std::nullopt,
      std::optional<ServiceCallback> pre_callback = std::nullopt,
      std::optional<ServiceCallback> post_callback = std::nullopt );

  /**
   * @brief Adds a joint. Internally wraps the transmission in an adapter.
   */
  void add_joint( const std::string &name,
                  std::shared_ptr<transmission_interface::Transmission> state_tx,
                  std::shared_ptr<transmission_interface::Transmission> command_tx,
                  PositionGetter position_getter );

  /**
   * @brief Direct injection for testing purposes.
   */
  void add_managed_joint( const std::string &name, ManagedJoint mj );

private:
  void handle_service(
      const std::shared_ptr<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets::Request>
          request,
      const std::shared_ptr<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets::Response>
          response );

  rclcpp::Node::SharedPtr node_;
  // Optional reference wrapper: safe, non-owning, and clearly expresses intent
  std::optional<std::reference_wrapper<std::mutex>> comm_mutex_;
  std::optional<ServiceCallback> pre_callback_;
  std::optional<ServiceCallback> post_callback_;
  rclcpp::Service<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets>::SharedPtr service_;
  std::unordered_map<std::string, ManagedJoint> managed_joints_;
};

} // namespace hector_transmission_interface

#endif
