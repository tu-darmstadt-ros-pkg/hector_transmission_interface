#include "hector_transmission_interface/adjustable_offset_manager.hpp"

#include <cmath>

namespace hector_transmission_interface
{
/**
 * @brief Concrete adapter mapping the interface to the non-virtual transmission methods.
 */
class AdjustableTransmissionAdapter : public IAdjustableOffset
{
public:
  explicit AdjustableTransmissionAdapter( std::shared_ptr<AdjustableOffsetTransmission> tx )
      : tx_( tx )
  {
  }

  void adjustOffset( double offset ) override { tx_->adjustTransmissionOffset( offset ); }
  double getOffset() const override { return tx_->get_joint_offset(); }

private:
  std::shared_ptr<AdjustableOffsetTransmission> tx_;
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
    PositionGetter position_getter )
{
  auto adj_state = std::dynamic_pointer_cast<AdjustableOffsetTransmission>( state_tx );
  if ( !adj_state )
    return;

  managed_joints_[name] =
      ManagedJoint{ std::make_shared<AdjustableTransmissionAdapter>( adj_state ), position_getter };
  RCLCPP_INFO( node_->get_logger(), "Managing the offset of joint '%s'.", name.c_str() );
}

void AdjustableOffsetManager::add_managed_joint( const std::string &name, ManagedJoint mj )
{
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

  // --- THREAD SAFETY ---
  // Held for the rest of this function, including the post-callback and every position_getter call.
  const OptionalCommLock lock( comm_mutex_ );

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

  for ( size_t i = 0; i < num_measurements; ++i ) {
    const std::string &joint_name = request->external_joint_measurements.name[i];
    double external_pos = request->external_joint_measurements.position[i];

    if ( !std::isfinite( external_pos ) ) {
      RCLCPP_WARN( node_->get_logger(), "Received NaN for joint %s, skipping.", joint_name.c_str() );
      continue;
    }

    auto it = managed_joints_.find( joint_name );
    if ( it == managed_joints_.end() )
      continue;

    auto &mj = it->second;
    if ( !mj.handle ) {
      RCLCPP_WARN( node_->get_logger(), "Joint '%s' has no offset handle registered, skipping.",
                   joint_name.c_str() );
      continue;
    }

    double internal_pos = 0.0;
    bool found_internal = false;

    if ( !request->original_joint_state.name.empty() ) {
      for ( size_t j = 0; j < request->original_joint_state.name.size(); ++j ) {
        if ( request->original_joint_state.name[j] == joint_name ) {
          internal_pos = request->original_joint_state.position[j];
          found_internal = std::isfinite( internal_pos );
          break;
        }
      }
    }

    if ( !found_internal )
      internal_pos = mj.position_getter();

    double current_offset = mj.handle->getOffset();
    double corrected_offset =
        request->external_joint_measurements.position[i] - internal_pos + current_offset;

    mj.handle->adjustOffset( corrected_offset );

    response->offset_changes.push_back( corrected_offset - current_offset );
  }
  response->message = response->offset_changes.empty() ? "No joints adjusted" : "Success";
  response->success = !response->offset_changes.empty();

  // --- POST CALLBACK ---
  if ( post_callback_locked_.has_value() ) {
    if ( !post_callback_locked_.value()() ) {
      RCLCPP_WARN( node_->get_logger(), "Post-callback returned false." );
    }
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

  // --- THREAD SAFETY ---
  // Held for the rest of this function, including the post-callback.
  const OptionalCommLock lock( comm_mutex_ );

  size_t flipped_count = 0;

  for ( const auto &joint_name : request->joint_names ) {
    auto it = managed_joints_.find( joint_name );
    if ( it == managed_joints_.end() ) {
      RCLCPP_WARN( node_->get_logger(), "Joint '%s' not found, skipping.", joint_name.c_str() );
      continue;
    }

    auto &mj = it->second;
    if ( !mj.handle ) {
      RCLCPP_WARN( node_->get_logger(), "Joint '%s' has no offset handle registered, skipping.",
                   joint_name.c_str() );
      continue;
    }

    mj.handle->adjustOffset( mj.handle->getOffset() + M_PI );

    RCLCPP_INFO( node_->get_logger(), "Flipped joint '%s' by pi.", joint_name.c_str() );
    ++flipped_count;
  }

  response->success = flipped_count > 0;
  response->message = flipped_count > 0
                          ? "Flipped " + std::to_string( flipped_count ) + " joint(s) by pi"
                          : "No joints flipped";

  // --- POST CALLBACK ---
  if ( post_callback_locked_.has_value() ) {
    if ( !post_callback_locked_.value()() ) {
      RCLCPP_WARN( node_->get_logger(), "Post-callback returned false." );
    }
  }
}

} // namespace hector_transmission_interface
