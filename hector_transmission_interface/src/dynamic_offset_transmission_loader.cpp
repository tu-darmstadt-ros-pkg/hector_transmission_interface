#include "hector_transmission_interface/dynamic_offset_transmission.hpp"
#include "hector_transmission_interface/hector_transmission_interface.hpp"
#include <functional>

namespace hector_transmission_interface
{

HectorTransmissionInterface::HectorTransmissionInterface() : Node( "hector_transmission_interface" )
{
  setup();
}

void HectorTransmissionInterface::setup() { }

} // namespace hector_transmission_interface

int main( int argc, char *argv[] )
{

  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<hector_transmission_interface::HectorTransmissionInterface>() );
  rclcpp::shutdown();

  return 0;
}
