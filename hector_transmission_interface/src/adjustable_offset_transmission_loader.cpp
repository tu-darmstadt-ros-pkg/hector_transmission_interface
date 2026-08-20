
#include "transmission_interface/simple_transmission_loader.hpp"

#include <algorithm>
#include <cctype>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "transmission_interface/simple_transmission.hpp"
#include <hector_transmission_interface/adjustable_offset_transmission_loader.hpp>

namespace hector_transmission_interface
{

namespace
{
bool parseBoolParam( const std::string &value )
{
  std::string lower;
  lower.reserve( value.size() );
  std::transform( value.begin(), value.end(), std::back_inserter( lower ),
                  []( unsigned char c ) { return std::tolower( c ); } );
  return lower == "true" || lower == "1" || lower == "yes" || lower == "on";
}

std::vector<std::string> parseInterfaceList( const std::string &value )
{
  // Split on commas or whitespace; trim each entry; drop empties.
  std::vector<std::string> result;
  std::string current;
  const auto flush = [&]() {
    auto begin = current.find_first_not_of( " \t" );
    auto end = current.find_last_not_of( " \t" );
    if ( begin != std::string::npos ) {
      result.emplace_back( current.substr( begin, end - begin + 1 ) );
    }
    current.clear();
  };
  for ( const char c : value ) {
    if ( c == ',' || c == ' ' || c == '\t' ) {
      flush();
    } else {
      current.push_back( c );
    }
  }
  flush();
  return result;
}
} // namespace

namespace
{
struct ParsedInfo {
  std::string joint_name;
  double reduction;
  double offset;
  bool pass_through_effort = false;
  std::vector<std::string> pass_through_interfaces;
};

ParsedInfo parse( const hardware_interface::TransmissionInfo &transmission_info )
{
  const auto &joint_info = transmission_info.joints.at( 0 );
  ParsedInfo parsed{ joint_info.name, joint_info.mechanical_reduction, joint_info.offset };
  const auto pte_it = transmission_info.parameters.find( "pass_through_effort" );
  if ( pte_it != transmission_info.parameters.end() ) {
    parsed.pass_through_effort = parseBoolParam( pte_it->second );
  }
  const auto pti_it = transmission_info.parameters.find( "pass_through_interfaces" );
  if ( pti_it != transmission_info.parameters.end() ) {
    parsed.pass_through_interfaces = parseInterfaceList( pti_it->second );
  }
  return parsed;
}

void logFailure( const std::exception &ex )
{
  RCLCPP_ERROR( rclcpp::get_logger( "adjustable_offset_transmission_loader" ),
                "Failed to construct transmission '%s'", ex.what() );
}
} // namespace

std::shared_ptr<transmission_interface::Transmission> AdjustableOffsetTransmissionLoader::load(
    const hardware_interface::TransmissionInfo &transmission_info )
{
  try {
    auto info = parse( transmission_info );
    return std::make_shared<AdjustableOffsetTransmission>(
        info.joint_name, info.reduction, info.offset, info.pass_through_effort,
        std::move( info.pass_through_interfaces ) );
  } catch ( const std::exception &ex ) {
    logFailure( ex );
    return {};
  }
}

AdjustableOffsetTransmissionLoader::Pair AdjustableOffsetTransmissionLoader::loadPair(
    const hardware_interface::TransmissionInfo &transmission_info )
{
  try {
    const auto info = parse( transmission_info );
    auto offset = std::make_shared<JointOffset>( info.joint_name, info.offset );
    using Side = AdjustableOffsetTransmission::Side;
    return { std::make_shared<AdjustableOffsetTransmission>( offset, Side::State, info.reduction,
                                                             info.pass_through_effort,
                                                             info.pass_through_interfaces ),
             std::make_shared<AdjustableOffsetTransmission>( offset, Side::Command, info.reduction,
                                                             info.pass_through_effort,
                                                             info.pass_through_interfaces ) };
  } catch ( const std::exception &ex ) {
    logFailure( ex );
    return {};
  }
}

} // namespace hector_transmission_interface

PLUGINLIB_EXPORT_CLASS( hector_transmission_interface::AdjustableOffsetTransmissionLoader,
                        transmission_interface::TransmissionLoader )
