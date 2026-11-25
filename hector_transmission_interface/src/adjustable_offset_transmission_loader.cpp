
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
bool parseBoolParam( const std::string &name, const std::string &value )
{
  std::string lower;
  lower.reserve( value.size() );
  std::transform( value.begin(), value.end(), std::back_inserter( lower ),
                  []( unsigned char c ) { return std::tolower( c ); } );
  // Trimmed, because a URDF parser is not obliged to hand the text over without the surrounding
  // whitespace, and " true" is not a different answer from "true".
  const auto first = lower.find_first_not_of( " \t\n\r" );
  const auto last = lower.find_last_not_of( " \t\n\r" );
  if ( first != std::string::npos )
    lower = lower.substr( first, last - first + 1 );

  if ( lower == "true" || lower == "1" || lower == "yes" || lower == "on" )
    return true;
  if ( lower == "false" || lower == "0" || lower == "no" || lower == "off" )
    return false;
  // Anything else is a typo. Reading it as false configures the transmission the opposite way to
  // what was written down, silently.
  throw transmission_interface::Exception( "Parameter '" + name + "' has the value '" + value +
                                           "', which is not a boolean." );
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
  // This transmission maps one joint to one actuator. Reading only the first of several, which is
  // what at(0) does on its own, loads a configuration nobody wrote and leaves the rest unmapped.
  if ( transmission_info.joints.size() != 1 ) {
    throw transmission_interface::Exception( "Transmission '" + transmission_info.name + "' has " +
                                             std::to_string( transmission_info.joints.size() ) +
                                             " joints. Exactly one is supported." );
  }
  if ( transmission_info.actuators.size() != 1 ) {
    throw transmission_interface::Exception( "Transmission '" + transmission_info.name + "' has " +
                                             std::to_string( transmission_info.actuators.size() ) +
                                             " actuators. Exactly one is supported." );
  }
  const auto &joint_info = transmission_info.joints.at( 0 );
  ParsedInfo parsed{ joint_info.name, joint_info.mechanical_reduction, joint_info.offset };
  const auto pte_it = transmission_info.parameters.find( "pass_through_effort" );
  if ( pte_it != transmission_info.parameters.end() ) {
    parsed.pass_through_effort = parseBoolParam( "pass_through_effort", pte_it->second );
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
