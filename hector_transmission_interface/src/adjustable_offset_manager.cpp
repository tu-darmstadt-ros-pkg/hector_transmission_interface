#include "hector_transmission_interface/adjustable_offset_manager.hpp"

namespace hector_transmission_interface
{
AdjustableOffsetManager::AdjustableOffsetManager( rclcpp::Node::SharedPtr node ) : node_( node )
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
    mj.state_transmission = adj_state;
    mj.command_transmission = adj_cmd;
    mj.position_getter = position_getter;
    managed_joints_[name] = mj;
    RCLCPP_INFO( node_->get_logger(), "Registered joint '%s' for adjustable offset calibration.",
                 name.c_str() );
  }
}

void AdjustableOffsetManager::handle_service(
    const std::shared_ptr<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets::Request> request,
    const std::shared_ptr<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets::Response>
        response )
{
  RCLCPP_INFO( node_->get_logger(), "Request to adjust transmission offsets received." );
  response->success = true;

  for ( size_t i = 0; i < request->external_joint_measurements.name.size(); ++i ) {
    const std::string &joint_name = request->external_joint_measurements.name[i];

    auto it = managed_joints_.find( joint_name );
    if ( it == managed_joints_.end() ) {
      RCLCPP_WARN( node_->get_logger(), "Joint '%s' is not managed or not adjustable.",
                   joint_name.c_str() );
      continue;
    }

    auto &mj = it->second;
    double external_pos = request->external_joint_measurements.position[i];
    double internal_pos = 0.0;
    bool found_internal = false;

    // Check if optional original joint state is provided
    if ( !request->original_joint_state.name.empty() ) {
      for ( size_t j = 0; j < request->original_joint_state.name.size(); ++j ) {
        if ( request->original_joint_state.name[j] == joint_name ) {
          internal_pos = request->original_joint_state.position[j];
          found_internal = true;
          break;
        }
      }
    }

    // Fallback to current state via the functional getter
    if ( !found_internal ) {
      if ( mj.position_getter ) {
        internal_pos = mj.position_getter();
      } else {
        RCLCPP_ERROR( node_->get_logger(), "No position getter for joint '%s'", joint_name.c_str() );
        continue;
      }
    }

    // Logic: New Offset = ExternalPos - (InternalPos - OldOffset)
    double current_offset = mj.state_transmission->get_joint_offset();
    double corrected_offset = external_pos - internal_pos + current_offset;

    mj.state_transmission->adjustTransmissionOffset( corrected_offset );
    mj.command_transmission->adjustTransmissionOffset( corrected_offset );

    response->adjusted_offsets.push_back( corrected_offset );
    RCLCPP_INFO( node_->get_logger(), "Adjusted offset for joint '%s' to %f", joint_name.c_str(),
                 corrected_offset );
  }

  if ( response->adjusted_offsets.empty() ) {
    response->success = false;
    response->message = "No offsets were adjusted. Check joint names.";
  } else {
    response->message = "Offsets adjusted successfully";
  }
}

} // namespace hector_transmission_interface
