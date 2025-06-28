
#include "transmission_interface/simple_transmission_loader.hpp"

#include <memory>

#include "hardware_interface/hardware_info.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "transmission_interface/simple_transmission.hpp"
#include <hector_transmission_interface/dynamic_offset_transmission_loader.hpp>

namespace hector_transmission_interface
{
DynamicOffsetTransmissionLoader::DynamicOffsetTransmissionLoader( const rclcpp::Node::SharedPtr &node )
    : node_( node )
{
}
std::shared_ptr<transmission_interface::Transmission>
DynamicOffsetTransmissionLoader::load( const hardware_interface::TransmissionInfo &transmission_info )
{
  try {
    const auto mechanical_reduction = transmission_info.joints.at( 0 ).mechanical_reduction;
    const auto offset = transmission_info.joints.at( 0 ).offset;
    const auto joint_name = transmission_info.joints.at( 0 ).name;
    std::shared_ptr<transmission_interface::Transmission> transmission(
        new DynamicOffsetTransmission( node_, joint_name, mechanical_reduction, offset ) );
    return transmission;
  } catch ( const std::exception &ex ) {
    RCLCPP_ERROR( rclcpp::get_logger( "simple_transmission_loader" ),
                  "Failed to construct transmission '%s'", ex.what() );
    return std::shared_ptr<transmission_interface::Transmission>();
  }
}

} // namespace hector_transmission_interface

PLUGINLIB_EXPORT_CLASS( hector_transmission_interface::DynamicOffsetTransmissionLoader,
                        transmission_interface::TransmissionLoader )