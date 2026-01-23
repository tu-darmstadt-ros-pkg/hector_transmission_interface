#include "hector_transmission_interface/adjustable_offset_manager.hpp"

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
    rclcpp::Node::SharedPtr node, std::optional<std::reference_wrapper<std::mutex>> comm_mutex )
    : node_( node ), comm_mutex_( comm_mutex )
{
  service_ = node_->create_service<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets>(
      "~/adjust_transmission_offsets", std::bind( &AdjustableOffsetManager::handle_service, this,
                                                  std::placeholders::_1, std::placeholders::_2 ) );
}

void AdjustableOffsetManager::add_joint(
    const std::string &name, std::shared_ptr<transmission_interface::Transmission> state_tx,
    std::shared_ptr<transmission_interface::Transmission> command_tx, PositionGetter position_getter )
{
  auto adj_state = std::dynamic_pointer_cast<AdjustableOffsetTransmission>( state_tx );
  auto adj_cmd = std::dynamic_pointer_cast<AdjustableOffsetTransmission>( command_tx );

  if ( adj_state && adj_cmd ) {
    ManagedJoint mj;
    mj.state_handle = std::make_shared<AdjustableTransmissionAdapter>( adj_state );
    mj.command_handle = std::make_shared<AdjustableTransmissionAdapter>( adj_cmd );
    mj.position_getter = position_getter;
    add_managed_joint( name, mj );
    RCLCPP_INFO( node_->get_logger(), "Registered joint '%s' via adapter.", name.c_str() );
  }
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
  // --- THREAD SAFETY ---
  // Create a conditional lock. If the optional is set, we lock the referenced mutex.
  std::unique_lock<std::mutex> lock;
  if ( comm_mutex_.has_value() ) {
    lock = std::unique_lock<std::mutex>( comm_mutex_->get() );
    RCLCPP_DEBUG( node_->get_logger(), "Offset adjustment service locked hardware mutex." );
  }

  response->success = false;

  // 1. Safety check for array bounds
  size_t num_measurements = std::min( request->external_joint_measurements.name.size(),
                                      request->external_joint_measurements.position.size() );

  for ( size_t i = 0; i < num_measurements; ++i ) {
    const std::string &joint_name = request->external_joint_measurements.name[i];
    double external_pos = request->external_joint_measurements.position[i];

    // 2. Skip NaNs
    if ( !std::isfinite( external_pos ) ) {
      RCLCPP_WARN( node_->get_logger(), "Received NaN for joint %s, skipping.", joint_name.c_str() );
      continue;
    }

    auto it = managed_joints_.find( joint_name );
    if ( it == managed_joints_.end() )
      continue;

    auto &mj = it->second;
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

    double current_offset = mj.state_handle->getOffset();
    double corrected_offset =
        request->external_joint_measurements.position[i] - internal_pos + current_offset;

    mj.state_handle->adjustOffset( corrected_offset );
    mj.command_handle->adjustOffset( corrected_offset );

    response->adjusted_offsets.push_back( corrected_offset );
  }
  response->message = response->adjusted_offsets.empty() ? "No joints adjusted" : "Success";
  response->success = !response->adjusted_offsets.empty();
}

} // namespace hector_transmission_interface
