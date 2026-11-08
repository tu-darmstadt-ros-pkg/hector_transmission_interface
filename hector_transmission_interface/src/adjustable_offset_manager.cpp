#include "hector_transmission_interface/adjustable_offset_manager.hpp"

#include <cmath>
#include <set>
#include <vector>

namespace hector_transmission_interface
{
namespace
{
// Filled where the bus is held and logged after it is released, since logging has no time bound.
struct DeferredLog {
  std::vector<std::string> info, warn, error;

  void flush( const rclcpp::Logger &logger ) const
  {
    for ( const auto &message : info ) RCLCPP_INFO( logger, "%s", message.c_str() );
    for ( const auto &message : warn ) RCLCPP_WARN( logger, "%s", message.c_str() );
    for ( const auto &message : error ) RCLCPP_ERROR( logger, "%s", message.c_str() );
  }
};
} // namespace

/**
 * @brief Concrete adapter mapping the interface to the non-virtual transmission methods.
 */
class AdjustableTransmissionAdapter : public IAdjustableOffset
{
public:
  explicit AdjustableTransmissionAdapter( std::shared_ptr<AdjustableOffsetTransmission> state_tx,
                                          std::shared_ptr<AdjustableOffsetTransmission> command_tx )
      : state_tx_( std::move( state_tx ) ), command_tx_( std::move( command_tx ) )
  {
  }

  void adjustOffset( double offset ) override
  {
    state_tx_->adjustTransmissionOffset( offset );
    // The two sides share one JointOffset, so the command side already sees the new offset. What
    // it does not see is its own cached joint goal, which was computed under the old one: mapping
    // that down again moves the actuator by the offset change. Re-deriving the joint goal from the
    // actuator goal keeps the actuator where it is.
    if ( command_tx_ )
      command_tx_->adjustTransmissionOffset( offset );
  }
  double getOffset() const override { return state_tx_->get_joint_offset(); }

private:
  std::shared_ptr<AdjustableOffsetTransmission> state_tx_;
  std::shared_ptr<AdjustableOffsetTransmission> command_tx_;
};

AdjustableOffsetManager::AdjustableOffsetManager(
    rclcpp::Node::SharedPtr node, std::optional<std::reference_wrapper<std::mutex>> comm_mutex,
    std::optional<ServiceCallback> pre_callback, std::optional<ServiceCallback> post_callback )
    : node_( node ), comm_mutex_( comm_mutex ), pre_callback_unlocked_( pre_callback ),
      post_callback_locked_( post_callback )
{
  service_ = node_->create_service<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets>(
      "~/adjust_transmission_offsets", std::bind( &AdjustableOffsetManager::handle_service, this,
                                                  std::placeholders::_1, std::placeholders::_2 ) );
  flip_by_pi_service_ = node_->create_service<hector_transmission_interface_msgs::srv::FlipByPi>(
      "~/flip_by_pi", std::bind( &AdjustableOffsetManager::handle_flip_by_pi, this,
                                 std::placeholders::_1, std::placeholders::_2 ) );
}

void AdjustableOffsetManager::add_joint_state_interface(
    const std::string &name, std::shared_ptr<transmission_interface::Transmission> state_tx,
    PositionGetter position_getter, std::shared_ptr<transmission_interface::Transmission> command_tx )
{
  auto adj_state = std::dynamic_pointer_cast<AdjustableOffsetTransmission>( state_tx );
  if ( !adj_state )
    return;

  auto adj_command = std::dynamic_pointer_cast<AdjustableOffsetTransmission>( command_tx );
  if ( command_tx && !adj_command ) {
    RCLCPP_WARN( node_->get_logger(),
                 "Joint '%s' has a command transmission that does not carry an adjustable offset. "
                 "Its goal will be mapped with the new offset after an adjustment, which moves the "
                 "actuator.",
                 name.c_str() );
  }

  {
    std::lock_guard<std::mutex> lock( managed_joints_mutex_ );
    managed_joints_[name] = ManagedJoint{
        std::make_shared<AdjustableTransmissionAdapter>( adj_state, adj_command ), position_getter };
  }
  RCLCPP_INFO( node_->get_logger(), "Managing the offset of joint '%s'.", name.c_str() );
}

void AdjustableOffsetManager::add_managed_joint( const std::string &name, ManagedJoint mj )
{
  std::lock_guard<std::mutex> lock( managed_joints_mutex_ );
  managed_joints_[name] = mj;
}

void AdjustableOffsetManager::handle_service(
    const std::shared_ptr<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets::Request> request,
    const std::shared_ptr<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets::Response>
        response )
{
  // --- PRE CALLBACK ---
  if ( pre_callback_unlocked_.has_value() ) {
    if ( !pre_callback_unlocked_.value()() ) {
      RCLCPP_WARN( node_->get_logger(), "Pre-callback returned false, aborting service call." );
      response->success = false;
      response->message = "Pre-callback failed";
      return;
    }
  }

  response->success = false;

  // A JointState whose name and position arrays disagree carries no usable measurement: the
  // positions cannot be attributed to names. Truncating to the shorter one calibrates against
  // whatever happens to line up, so a malformed request is refused whole.
  if ( request->external_joint_measurements.name.size() !=
       request->external_joint_measurements.position.size() ) {
    response->message =
        "external_joint_measurements has " +
        std::to_string( request->external_joint_measurements.name.size() ) + " names but " +
        std::to_string( request->external_joint_measurements.position.size() ) + " positions";
    RCLCPP_ERROR( node_->get_logger(), "Refusing offset adjustment: %s", response->message.c_str() );
    return;
  }
  if ( request->original_joint_state.name.size() != request->original_joint_state.position.size() ) {
    response->message =
        "original_joint_state has " + std::to_string( request->original_joint_state.name.size() ) +
        " names but " + std::to_string( request->original_joint_state.position.size() ) +
        " positions";
    RCLCPP_ERROR( node_->get_logger(), "Refusing offset adjustment: %s", response->message.c_str() );
    return;
  }

  const size_t num_measurements = request->external_joint_measurements.name.size();
  std::vector<std::string> refused;
  DeferredLog log;

  // Under the hardware mutex or in the control loop, including the post-callback and every
  // position_getter call
  const bool ran = runAdjustment( [&] {
    // Separate from the hardware mutex, which does not cover the registration map.
    std::lock_guard<std::mutex> joints_lock( managed_joints_mutex_ );
    for ( size_t i = 0; i < num_measurements; ++i ) {
      const std::string &joint_name = request->external_joint_measurements.name[i];
      double external_pos = request->external_joint_measurements.position[i];

      if ( !std::isfinite( external_pos ) ) {
        log.warn.push_back( "Received NaN for joint " + joint_name + ", skipping." );
        continue;
      }

      auto it = managed_joints_.find( joint_name );
      if ( it == managed_joints_.end() )
        continue;

      auto &mj = it->second;
      if ( !mj.handle ) {
        log.warn.push_back( "Joint '" + joint_name + "' has no offset handle registered, skipping." );
        continue;
      }
      if ( const std::string reason = refusal( joint_name ); !reason.empty() ) {
        refused.push_back( joint_name + " (" + reason + ")" );
        continue;
      }

      double internal_pos = 0.0;
      bool found_internal = false;

      if ( !request->original_joint_state.name.empty() ) {
        for ( size_t j = 0; j < request->original_joint_state.name.size(); ++j ) {
          if ( request->original_joint_state.name[j] == joint_name ) {
            internal_pos = request->original_joint_state.position[j];
            found_internal = true;
            break;
          }
        }
      }

      if ( !found_internal )
        internal_pos = mj.position_getter();

      double current_offset = mj.handle->getOffset();
      // Everything the new offset is computed from has to be a number. The fallback baseline is
      // whatever the hardware last read, which is not a position at all before the first read
      // succeeds, and an offset derived from one is written to disk and poisons every later reading.
      if ( !std::isfinite( internal_pos ) || !std::isfinite( current_offset ) ) {
        log.error.push_back( "Joint '" + joint_name +
                             "' has no usable position to calibrate against (baseline " +
                             std::to_string( internal_pos ) + ", current offset " +
                             std::to_string( current_offset ) + "). Skipping it." );
        continue;
      }
      double corrected_offset =
          request->external_joint_measurements.position[i] - internal_pos + current_offset;

      mj.handle->adjustOffset( corrected_offset );

      response->offset_changes.push_back( corrected_offset - current_offset );
      // Named, because the changes are fewer than the request whenever a joint is skipped and
      // nothing else says which ones they belong to.
      response->joint_names.push_back( joint_name );
    }
    if ( post_callback_locked_.has_value() && !post_callback_locked_.value()() ) {
      log.warn.push_back( "Post-callback returned false." );
    }
  } );
  log.flush( node_->get_logger() );
  if ( !ran ) {
    response->offset_changes.clear();
    response->joint_names.clear();
    response->message = "The hardware could not run the adjustment, nothing was changed";
    RCLCPP_ERROR( node_->get_logger(), "Refusing offset adjustment: %s", response->message.c_str() );
    return;
  }
  // An empty request adjusted everything it was asked to, which is nothing. Answering success for
  // that tells a calibration client its measurements were applied when it supplied none.
  const bool all_adjusted =
      num_measurements > 0 && response->offset_changes.size() == num_measurements;
  response->success = all_adjusted;
  if ( num_measurements == 0 ) {
    response->message = "No joints were requested";
  } else if ( all_adjusted ) {
    response->message = "Success";
  } else if ( response->offset_changes.empty() ) {
    response->message = "No joints adjusted";
  } else {
    response->message = "Adjusted " + std::to_string( response->offset_changes.size() ) + " of " +
                        std::to_string( num_measurements ) +
                        " joints; joint_names lists the ones that changed";
  }

  if ( !refused.empty() ) {
    std::string list;
    for ( const auto &entry : refused ) { list += ( list.empty() ? "" : ", " ) + entry; }
    response->message += "; refused: " + list;
    RCLCPP_ERROR( node_->get_logger(), "Offset adjustment refused for %s", list.c_str() );
  }
}
void AdjustableOffsetManager::handle_flip_by_pi(
    const std::shared_ptr<hector_transmission_interface_msgs::srv::FlipByPi::Request> request,
    const std::shared_ptr<hector_transmission_interface_msgs::srv::FlipByPi::Response> response )
{
  // --- PRE CALLBACK ---
  if ( pre_callback_unlocked_.has_value() ) {
    if ( !pre_callback_unlocked_.value()() ) {
      RCLCPP_WARN( node_->get_logger(),
                   "Pre-callback returned false, aborting flip_by_pi service call." );
      response->success = false;
      response->message = "Pre-callback failed";
      return;
    }
  }

  size_t flipped_count = 0;
  std::vector<std::string> refused;
  DeferredLog log;

  // A name repeated in one request would otherwise be flipped twice, which is a full turn of the
  // joint rather than a half one.
  const std::set<std::string> requested( request->joint_names.begin(), request->joint_names.end() );
  const bool ran = runAdjustment( [&] {
    // Separate from the hardware mutex, which does not cover the registration map.
    std::lock_guard<std::mutex> joints_lock( managed_joints_mutex_ );
    for ( const auto &joint_name : requested ) {
      auto it = managed_joints_.find( joint_name );
      if ( it == managed_joints_.end() ) {
        log.warn.push_back( "Joint '" + joint_name + "' not found, skipping." );
        continue;
      }

      auto &mj = it->second;
      if ( !mj.handle ) {
        log.warn.push_back( "Joint '" + joint_name + "' has no offset handle registered, skipping." );
        continue;
      }
      if ( const std::string reason = refusal( joint_name ); !reason.empty() ) {
        refused.push_back( joint_name + " (" + reason + ")" );
        continue;
      }

      mj.handle->adjustOffset( mj.handle->getOffset() + M_PI );

      log.info.push_back( "Flipped joint '" + joint_name + "' by pi." );
      ++flipped_count;
    }
    if ( post_callback_locked_.has_value() && !post_callback_locked_.value()() ) {
      log.warn.push_back( "Post-callback returned false." );
    }
  } );
  log.flush( node_->get_logger() );
  if ( !ran ) {
    response->success = false;
    response->message = "The hardware could not run the flip, nothing was changed";
    RCLCPP_ERROR( node_->get_logger(), "Refusing flip_by_pi: %s", response->message.c_str() );
    return;
  }

  // Every named joint, not just one of them: a half-flipped pair leaves the robot in a pose
  // nobody asked for, and the response has no room to say which half.
  response->success = !requested.empty() && flipped_count == requested.size();
  response->message = "Flipped " + std::to_string( flipped_count ) + " of " +
                      std::to_string( requested.size() ) + " requested joint(s) by pi";
  if ( !refused.empty() ) {
    std::string list;
    for ( const auto &entry : refused ) { list += ( list.empty() ? "" : ", " ) + entry; }
    response->message += "; refused: " + list;
    RCLCPP_ERROR( node_->get_logger(), "flip_by_pi refused for %s", list.c_str() );
  }
}

bool AdjustableOffsetManager::runAdjustment( const std::function<void()> &adjustment )
{
  if ( run_in_control_loop_ ) {
    return run_in_control_loop_( adjustment );
  }
  const OptionalCommLock lock( comm_mutex_ );
  adjustment();
  return true;
}

std::string AdjustableOffsetManager::refusal( const std::string &joint ) const
{
  return may_adjust_ ? may_adjust_( joint ) : std::string();
}

} // namespace hector_transmission_interface
