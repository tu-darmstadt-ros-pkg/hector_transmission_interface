
#include "transmission_interface/simple_transmission_loader.hpp"

#include <memory>

#include "hardware_interface/hardware_info.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "transmission_interface/simple_transmission.hpp"
#include <hector_transmission_interface/adjustable_offset_transmission_loader.hpp>

namespace hector_transmission_interface
{

std::shared_ptr<transmission_interface::Transmission> AdjustableOffsetTransmissionLoader::load(
    const hardware_interface::TransmissionInfo &transmission_info )
{
  try {
    const auto mechanical_reduction = transmission_info.joints.at( 0 ).mechanical_reduction;
    const auto offset = transmission_info.joints.at( 0 ).offset;
    const auto joint_name = transmission_info.joints.at( 0 ).name;
    std::shared_ptr<transmission_interface::Transmission> transmission(
        new AdjustableOffsetTransmission( joint_name, mechanical_reduction, offset ) );
    return transmission;
  } catch ( const std::exception &ex ) {
    RCLCPP_ERROR( rclcpp::get_logger( "simple_transmission_loader" ),
                  "Failed to construct transmission '%s'", ex.what() );
    return {};
  }
}

} // namespace hector_transmission_interface

PLUGINLIB_EXPORT_CLASS( hector_transmission_interface::AdjustableOffsetTransmissionLoader,
                        transmission_interface::TransmissionLoader )
