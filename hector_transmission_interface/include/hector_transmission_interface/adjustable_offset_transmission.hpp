//
// Created by aljoscha-schmidt on 6/4/25.
//

#ifndef DYNAMIC_OFFSET_TRANSMISSION_HPP
#define DYNAMIC_OFFSET_TRANSMISSION_HPP

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <transmission_interface/simple_transmission.hpp>
#include <utility>
#include <vector>

namespace hector_transmission_interface
{
/**
 * \brief The offset of one joint, and the state of the 2pi jump detector that maintains it.
 *
 * A joint is served by two transmissions - one bound to its present values, one to its goals -
 * and they must agree on the offset at every instant, so the offset lives here and both hold a
 * pointer to it. Persisted to ~/.ros/dynamic_offset_transmissions/<joint>.txt.
 */
class JointOffset
{
public:
  explicit JointOffset( const std::string &joint_name, double initial_offset );

  double value() const { return offset_; }
  void set( double offset );

  /**
   * \brief Feed one present actuator position to the jump detector.
   *
   * When a Dynamixel actuator loses power it can come back with its position shifted by a
   * multiple of 2pi. Such a shift since the previous reading is absorbed into the offset, so the
   * joint position stays where it was.
   * @return the number of 2pi turns corrected (0 if nothing happened)
   */
  int observeActuatorPosition( double actuator_position, double reduction );

  /// Restart the detector from this position: a deliberate offset change is not a jump.
  void rebaseDetector( double actuator_position ) { prev_actuator_position_ = actuator_position; }

  int correctionCount() const { return correction_count_; }
  const std::string &jointName() const { return joint_name_; }

private:
  bool existsTransmissionOffsetFile() const;
  void loadTransmissionOffset();
  void saveTransmissionOffset() const;

  std::string joint_name_;
  std::filesystem::path transmission_file_path_;
  double offset_;
  std::optional<double> prev_actuator_position_;
  int correction_count_{ 0 };

  static constexpr double TWO_PI = 2.0 * M_PI;
  // Tolerance for detecting a 2pi jump (allow ~10% margin)
  static constexpr double JUMP_TOLERANCE = 0.2 * M_PI;
};

class AdjustableOffsetTransmission : public transmission_interface::SimpleTransmission
{
public:
  /**
   * Which of a joint's two transmissions this is. Only the one bound to the PRESENT values sees
   * real actuator readings, so only it feeds the jump detector; the one bound to the goals just
   * maps with whatever offset the joint has.
   */
  enum class Side { State, Command };

  /// A transmission owning a fresh JointOffset for the joint: the state side of a lone instance.
  explicit AdjustableOffsetTransmission( const std::string &joint_name,
                                         double joint_to_actuator_reduction,
                                         double joint_offset = 0.0, bool pass_through_effort = false,
                                         std::vector<std::string> pass_through_interfaces = {} );

  /// One of the two transmissions sharing a joint's offset.
  AdjustableOffsetTransmission( std::shared_ptr<JointOffset> offset, Side side,
                                double joint_to_actuator_reduction, bool pass_through_effort = false,
                                std::vector<std::string> pass_through_interfaces = {} );

  /***
   * \brief
   * Adjusts the transmission offset for the joint.
   * This will also save the new offset to the file system.
   * WARNING: MAKE SURE TO DEACTIVATE THE CONTROLLER BEFORE CHANGING THE OFFSET!
   * @param offset The new offset to set for the joint.
   */
  void adjustTransmissionOffset( double offset );

  /// The joint's offset. Hides SimpleTransmission's copy, which is only a cache of this.
  double get_joint_offset() const { return offset_->value(); }
  const std::shared_ptr<JointOffset> &jointOffset() const { return offset_; }
  Side side() const { return side_; }

  /**
   * \brief Override configure to optionally route effort/torque/force and arbitrary
   * named interfaces through identity passthrough instead of letting the base class
   * scale them by the reduction.
   *
   * Handles whose interface name is in the passthrough set are removed from the lists
   * passed to SimpleTransmission::configure (so the base scaling never sees them) and
   * stored in passthrough_pairs_ for direct identity copy in actuator_to_joint /
   * joint_to_actuator.
   */
  void configure( const std::vector<transmission_interface::JointHandle> &joint_handles,
                  const std::vector<transmission_interface::ActuatorHandle> &actuator_handles ) override;

  /**
   * \brief Map actuator to joint values with the joint's offset.
   *
   * On the state side the actuator position is first shown to the jump detector, which may
   * change the offset. Also identity-copies any passthrough interfaces from actuator to joint.
   */
  void actuator_to_joint() override;

  /**
   * \brief Override joint_to_actuator to identity-copy any passthrough interfaces
   * from joint to actuator side after the base scaling.
   */
  void joint_to_actuator() override;

  /**
   * \brief Returns the number of 2pi corrections applied to this joint's offset (for diagnostics).
   */
  int getCorrectionCount() const { return offset_->correctionCount(); }

private:
  /// SimpleTransmission computes with its own jnt_offset_; refresh it from the shared value.
  void syncBaseOffset() { jnt_offset_ = offset_->value(); }

  std::shared_ptr<JointOffset> offset_;
  Side side_;

  // Passthrough configuration
  bool pass_through_effort_;
  std::vector<std::string> pass_through_interface_names_;
  // Pairs of (joint_handle, actuator_handle) sharing the same interface name that
  // should be identity-copied in both directions instead of being scaled by reduction.
  std::vector<std::pair<transmission_interface::JointHandle, transmission_interface::ActuatorHandle>>
      passthrough_pairs_;
};

// ---------------------------------------------------------------------------------------------
// JointOffset

inline JointOffset::JointOffset( const std::string &joint_name, double initial_offset )
    : joint_name_( joint_name ), offset_( initial_offset )
{
  transmission_file_path_ = std::filesystem::path( std::getenv( "HOME" ) ) / ".ros" /
                            "dynamic_offset_transmissions" / ( joint_name + ".txt" );
  if ( existsTransmissionOffsetFile() ) {
    loadTransmissionOffset();
  } else {
    saveTransmissionOffset(); // save the initial offset to file
  }
  if ( initial_offset != offset_ ) {
    RCLCPP_INFO( rclcpp::get_logger( "AdjustableOffsetTransmission" ),
                 "Loaded transmission offset for joint '%s': %f from file %s", joint_name_.c_str(),
                 offset_, transmission_file_path_.c_str() );
  }
}

inline void JointOffset::set( double offset )
{
  offset_ = offset;
  saveTransmissionOffset();
}

inline int JointOffset::observeActuatorPosition( double actuator_position, double reduction )
{
  int n_jumps = 0;
  if ( prev_actuator_position_.has_value() ) {
    const double delta = actuator_position - prev_actuator_position_.value();
    // Round to the nearest integer multiple of 2pi
    n_jumps = static_cast<int>( std::round( delta / TWO_PI ) );
    if ( n_jumps != 0 && std::abs( delta - n_jumps * TWO_PI ) < JUMP_TOLERANCE ) {
      // The jump in joint space is n_jumps * 2pi / reduction; absorb it into the offset.
      offset_ -= n_jumps * TWO_PI / reduction;
      correction_count_ += std::abs( n_jumps );
      saveTransmissionOffset();
      RCLCPP_WARN( rclcpp::get_logger( "AdjustableOffsetTransmission" ),
                   "Detected %d x 2pi actuator position jump on joint '%s' "
                   "(actuator delta: %.4f rad). Corrected offset to %.6f.",
                   n_jumps, joint_name_.c_str(), delta, offset_ );
    } else {
      n_jumps = 0;
    }
  }
  prev_actuator_position_ = actuator_position;
  return n_jumps;
}

inline bool JointOffset::existsTransmissionOffsetFile() const
{
  return std::filesystem::exists( transmission_file_path_ );
}

inline void JointOffset::loadTransmissionOffset()
{
  // Load the transmission offset from the specified storage path
  if ( std::filesystem::exists( transmission_file_path_ ) ) {
    std::ifstream file( transmission_file_path_ );
    if ( file.is_open() ) {
      double offset;
      file >> offset;
      if ( file.fail() || !file.eof() ) {
        RCLCPP_ERROR( rclcpp::get_logger( "AdjustableOffsetTransmission" ),
                      "Failed to read a valid offset from file: %s",
                      transmission_file_path_.string().c_str() );
      } else {
        offset_ = offset;
      }
      file.close();
    } else {
      RCLCPP_ERROR( rclcpp::get_logger( "AdjustableOffsetTransmission" ),
                    "Failed to open transmission offset file: %s",
                    transmission_file_path_.string().c_str() );
    }
  } else {
    RCLCPP_WARN( rclcpp::get_logger( "AdjustableOffsetTransmission" ),
                 "Transmission offset file does not exist: %s",
                 transmission_file_path_.string().c_str() );
  }
}

inline void JointOffset::saveTransmissionOffset() const
{
  // Save the current transmission offset to the specified storage path
  std::filesystem::create_directories( transmission_file_path_.parent_path() );
  std::ofstream file( transmission_file_path_ );
  if ( file.is_open() ) {
    file << offset_;
    if ( file.fail() ) {
      RCLCPP_ERROR( rclcpp::get_logger( "AdjustableOffsetTransmission" ),
                    "Failed to write offset to file: %s", transmission_file_path_.string().c_str() );
    }
    file.close();
  } else {
    RCLCPP_ERROR( rclcpp::get_logger( "AdjustableOffsetTransmission" ),
                  "Failed to open transmission offset file for writing: %s",
                  transmission_file_path_.string().c_str() );
  }
}

// ---------------------------------------------------------------------------------------------
// AdjustableOffsetTransmission

inline AdjustableOffsetTransmission::AdjustableOffsetTransmission(
    const std::string &joint_name, double joint_to_actuator_reduction, double joint_offset,
    bool pass_through_effort, std::vector<std::string> pass_through_interfaces )
    : AdjustableOffsetTransmission( std::make_shared<JointOffset>( joint_name, joint_offset ),
                                    Side::State, joint_to_actuator_reduction, pass_through_effort,
                                    std::move( pass_through_interfaces ) )
{
}

inline AdjustableOffsetTransmission::AdjustableOffsetTransmission(
    std::shared_ptr<JointOffset> offset, Side side, double joint_to_actuator_reduction,
    bool pass_through_effort, std::vector<std::string> pass_through_interfaces )
    : SimpleTransmission( joint_to_actuator_reduction, offset->value() ),
      offset_( std::move( offset ) ), side_( side ), pass_through_effort_( pass_through_effort ),
      pass_through_interface_names_( std::move( pass_through_interfaces ) )
{
}

inline void AdjustableOffsetTransmission::actuator_to_joint()
{
  if ( side_ == Side::State && actuator_position_ ) {
    offset_->observeActuatorPosition( actuator_position_.get_value(), reduction_ );
  }
  syncBaseOffset();
  SimpleTransmission::actuator_to_joint();

  // Identity-copy any passthrough interfaces from actuator to joint side.
  for ( auto &[joint_handle, actuator_handle] : passthrough_pairs_ ) {
    if ( joint_handle && actuator_handle ) {
      joint_handle.set_value( actuator_handle.get_value() );
    }
  }
}

inline void AdjustableOffsetTransmission::joint_to_actuator()
{
  syncBaseOffset();
  SimpleTransmission::joint_to_actuator();

  // Identity-copy any passthrough interfaces from joint to actuator side.
  for ( auto &[joint_handle, actuator_handle] : passthrough_pairs_ ) {
    if ( joint_handle && actuator_handle ) {
      actuator_handle.set_value( joint_handle.get_value() );
    }
  }
}

inline void AdjustableOffsetTransmission::configure(
    const std::vector<transmission_interface::JointHandle> &joint_handles,
    const std::vector<transmission_interface::ActuatorHandle> &actuator_handles )
{
  passthrough_pairs_.clear();

  // Decide which interface names should bypass the base scaling.
  const auto is_passthrough_name = [this]( const std::string &name ) {
    if ( pass_through_effort_ &&
         ( name == hardware_interface::HW_IF_EFFORT || name == hardware_interface::HW_IF_TORQUE ||
           name == hardware_interface::HW_IF_FORCE ) ) {
      return true;
    }
    return std::find( pass_through_interface_names_.begin(), pass_through_interface_names_.end(),
                      name ) != pass_through_interface_names_.end();
  };

  std::vector<transmission_interface::JointHandle> filtered_joints;
  std::vector<transmission_interface::ActuatorHandle> filtered_actuators;
  std::vector<transmission_interface::JointHandle> joint_passthrough;
  std::vector<transmission_interface::ActuatorHandle> actuator_passthrough;

  filtered_joints.reserve( joint_handles.size() );
  filtered_actuators.reserve( actuator_handles.size() );

  for ( const auto &h : joint_handles ) {
    if ( is_passthrough_name( h.get_interface_name() ) ) {
      joint_passthrough.push_back( h );
    } else {
      filtered_joints.push_back( h );
    }
  }
  for ( const auto &h : actuator_handles ) {
    if ( is_passthrough_name( h.get_interface_name() ) ) {
      actuator_passthrough.push_back( h );
    } else {
      filtered_actuators.push_back( h );
    }
  }

  SimpleTransmission::configure( filtered_joints, filtered_actuators );

  // Pair passthrough handles by interface name. Drop orphans (one side missing).
  for ( const auto &joint_handle : joint_passthrough ) {
    const auto &name = joint_handle.get_interface_name();
    auto it = std::find_if( actuator_passthrough.begin(), actuator_passthrough.end(),
                            [&name]( const transmission_interface::ActuatorHandle &a ) {
                              return a.get_interface_name() == name;
                            } );
    if ( it == actuator_passthrough.end() ) {
      RCLCPP_DEBUG( rclcpp::get_logger( "AdjustableOffsetTransmission" ),
                    "Passthrough joint handle '%s' on joint '%s' has no matching actuator handle; "
                    "dropping.",
                    name.c_str(), offset_->jointName().c_str() );
      continue;
    }
    if ( !joint_handle || !*it ) {
      RCLCPP_DEBUG( rclcpp::get_logger( "AdjustableOffsetTransmission" ),
                    "Passthrough handle pair '%s' on joint '%s' has nullptr backing; dropping.",
                    name.c_str(), offset_->jointName().c_str() );
      continue;
    }
    passthrough_pairs_.emplace_back( joint_handle, *it );
  }
}

inline void AdjustableOffsetTransmission::adjustTransmissionOffset( double offset )
{
  offset_->set( offset );
  // A deliberate change is not a jump: whatever the state side reads next is the new baseline.
  if ( side_ == Side::State && actuator_position_ ) {
    offset_->rebaseDetector( actuator_position_.get_value() );
  }
  actuator_to_joint(); // update the joint side with the new offset
}
} // namespace hector_transmission_interface
#endif // DYNAMIC_OFFSET_TRANSMISSION_HPP
