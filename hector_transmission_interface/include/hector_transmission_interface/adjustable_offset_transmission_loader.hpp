#ifndef HECTOR_TRANSMISSION_INTERFACE__DYNAMIC_OFFSET_TRANSMISSION_LOADER_HPP_
#define HECTOR_TRANSMISSION_INTERFACE__DYNAMIC_OFFSET_TRANSMISSION_LOADER_HPP_

#include <memory>

#include <hector_transmission_interface/adjustable_offset_transmission.hpp>
#include <transmission_interface/transmission.hpp>
#include <transmission_interface/transmission_loader.hpp>
namespace hector_transmission_interface
{
/**
 * \brief Class for loading a simple transmission instance from configuration data.
 */
class AdjustableOffsetTransmissionLoader : public transmission_interface::TransmissionLoader
{
public:
  /// A lone transmission owning its joint's offset (the pluginlib entry point).
  std::shared_ptr<transmission_interface::Transmission>
  load( const hardware_interface::TransmissionInfo &transmission_info ) override;

  struct Pair {
    std::shared_ptr<transmission_interface::Transmission> state;
    std::shared_ptr<transmission_interface::Transmission> command;
    explicit operator bool() const { return state && command; }
  };

  /**
   * rief The two transmissions a hardware interface needs for one joint - one for its present
   * values, one for its goals - sharing a single offset. Both null on failure.
   */
  Pair loadPair( const hardware_interface::TransmissionInfo &transmission_info );
};

} // namespace hector_transmission_interface

#endif // HECTOR_TRANSMISSION_INTERFACE__DYNAMIC_OFFSET_TRANSMISSION_LOADER_HPP_
