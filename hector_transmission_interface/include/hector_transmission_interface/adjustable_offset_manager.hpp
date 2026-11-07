#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "hector_transmission_interface/adjustable_offset_transmission.hpp"
#include "hector_transmission_interface_msgs/srv/adjust_transmission_offsets.hpp"
#include "hector_transmission_interface_msgs/srv/flip_by_pi.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hector_transmission_interface
{

class IAdjustableOffset
{
public:
  virtual ~IAdjustableOffset() = default;
  virtual void adjustOffset( double offset ) = 0;
  virtual double getOffset() const = 0;
};

/**
 * @brief A lock that may or may not own anything, decided once at construction.
 *
 * The hardware mutex is optional - a caller that owns no hardware need not supply one - and that
 * used to be expressed as a default-constructed unique_lock move-assigned into behind an if.
 * Same behaviour, but acquisition and release then sat in different places. Here they are one
 * object: whether it owns a lock is settled by the constructor and never changes.
 */
class OptionalCommLock
{
public:
  explicit OptionalCommLock( const std::optional<std::reference_wrapper<std::mutex>> &mutex )
  {
    if ( mutex )
      lock_.emplace( mutex->get() );
  }

  OptionalCommLock( const OptionalCommLock & ) = delete;
  OptionalCommLock &operator=( const OptionalCommLock & ) = delete;

  [[nodiscard]] bool owns_lock() const noexcept { return lock_.has_value(); }

private:
  std::optional<std::lock_guard<std::mutex>> lock_;
};

class AdjustableOffsetManager
{
public:
  using PositionGetter = std::function<double()>;
  using ServiceCallback = std::function<bool()>;
  /// Why a joint's offset may not change now, or an empty string if it may.
  using MayAdjust = std::function<std::string( const std::string &joint )>;
  /// Runs an adjustment where the hardware's control loop runs, and waits for it. Returns false
  /// only if the adjustment never ran and never will, because it refers to the caller's stack.
  using RunInControlLoop = std::function<bool( const std::function<void()> &adjustment )>;

  /// A joint has one offset, however many transmissions read it.
  struct ManagedJoint {
    std::shared_ptr<IAdjustableOffset> handle;
    PositionGetter position_getter;
  };

  explicit AdjustableOffsetManager(
      rclcpp::Node::SharedPtr node,
      std::optional<std::reference_wrapper<std::mutex>> comm_mutex = std::nullopt,
      std::optional<ServiceCallback> pre_callback = std::nullopt,
      std::optional<ServiceCallback> post_callback = std::nullopt );

  /**
   * @brief Manage a joint's offset through its transmissions.
   *
   * Ignored unless the state transmission is an AdjustableOffsetTransmission. The command
   * transmission shares the same JointOffset, but keeps its own joint goal, computed under the
   * offset that was current when the controller wrote it. Pass it so an adjustment re-derives that
   * goal; leaving it out means the next write moves the actuator by the offset change.
   * @param name Joint name
   * @param state_tx State transmission
   * @param position_getter Function to get current position
   * @param command_tx Command transmission, if the joint has one
   */
  void add_joint_state_interface(
      const std::string &name, std::shared_ptr<transmission_interface::Transmission> state_tx,
      PositionGetter position_getter,
      std::shared_ptr<transmission_interface::Transmission> command_tx = nullptr );

  void add_managed_joint( const std::string &name, ManagedJoint mj );

  /// Refuse the joints @p may_adjust names a reason for. Asked where the adjustment runs.
  void set_may_adjust( MayAdjust may_adjust ) { may_adjust_ = std::move( may_adjust ); }

  /**
   * @brief Run every adjustment through @p run, in place of taking the hardware mutex.
   *
   * read() and write() map with the offsets, so a hardware that runs the adjustment inside its
   * control loop needs no lock against them. The post-callback and the position getters run there
   * too, as do a mutex, logging and allocations, which is acceptable for a rare calibration.
   */
  void set_run_in_control_loop( RunInControlLoop run ) { run_in_control_loop_ = std::move( run ); }

private:
  /// Runs @p adjustment under the hardware mutex, or through the control loop if one is set.
  bool runAdjustment( const std::function<void()> &adjustment );
  /// The reason @p joint may not be adjusted, or an empty string.
  std::string refusal( const std::string &joint ) const;

  void handle_service(
      const std::shared_ptr<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets::Request>
          request,
      const std::shared_ptr<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets::Response>
          response );

  void handle_flip_by_pi(
      const std::shared_ptr<hector_transmission_interface_msgs::srv::FlipByPi::Request> request,
      const std::shared_ptr<hector_transmission_interface_msgs::srv::FlipByPi::Response> response );

  rclcpp::Node::SharedPtr node_;
  std::optional<std::reference_wrapper<std::mutex>> comm_mutex_;

  /// Runs BEFORE the hardware mutex is taken. It is expected to talk to the controller manager,
  /// which needs the hardware free - so it must not assume any hardware state is stable, and it
  /// may block.
  std::optional<ServiceCallback> pre_callback_unlocked_;

  /// Runs WHILE the hardware mutex is held. It may therefore touch hardware state directly, and
  /// must not take that mutex again or call anything that does. Same rule for
  /// ManagedJoint::position_getter, which is invoked from inside the same critical section.
  std::optional<ServiceCallback> post_callback_locked_;
  /// Registration runs on the controller manager's thread while the services are already up, so a
  /// call landing during a re-export would iterate a map being inserted into. The hardware mutex
  /// does not cover this: it serialises the handlers against read and write, not against
  /// registration.
  mutable std::mutex managed_joints_mutex_;
  std::unordered_map<std::string, ManagedJoint> managed_joints_;
  MayAdjust may_adjust_;
  RunInControlLoop run_in_control_loop_;
  rclcpp::Service<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets>::SharedPtr service_;
  rclcpp::Service<hector_transmission_interface_msgs::srv::FlipByPi>::SharedPtr flip_by_pi_service_;
};

} // namespace hector_transmission_interface
