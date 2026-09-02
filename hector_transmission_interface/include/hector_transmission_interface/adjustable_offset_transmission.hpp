//
// Created by aljoscha-schmidt on 6/4/25.
//

#ifndef DYNAMIC_OFFSET_TRANSMISSION_HPP
#define DYNAMIC_OFFSET_TRANSMISSION_HPP

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
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
   * When a Dynamixel actuator loses power it comes back reporting its absolute position within a
   * single turn, i.e. the position it had shifted by a multiple of 2pi. Such a shift since the
   * previous reading is absorbed into the offset, so the joint position stays where it was.
   *
   * With a reset window set, only a shift that lands the reading inside the window counts: any
   * other multiple of 2pi is a real move and is reported, not corrected. Without a window every
   * multiple of 2pi is taken for a reset.
   * @return the number of 2pi turns corrected (0 if nothing happened)
   */
  int observeActuatorPosition( double actuator_position, double reduction );

  /**
   * \brief The interval a power cycle can leave this actuator's position in, in actuator radians.
   *
   * A window narrower than 2pi admits at most one of the candidate shifts, one of 3pi at most two.
   * The caller decides how tightly it can bound the interval; anything under 4pi rejects some
   * candidate and is worth setting.
   */
  void setResetWindow( double lower, double upper )
  {
    if ( !( lower < upper ) ) {
      // An empty window admits no candidate, so every correction is silently dropped while the
      // detector goes on warning about each jump it found and could not act on.
      throw transmission_interface::Exception(
          "The boot position window of joint '" + joint_name_ + "' is empty (lower " +
          std::to_string( lower ) + " is not below upper " + std::to_string( upper ) + ")." );
    }
    reset_window_ = std::make_pair( lower, upper );
  }
  const std::optional<std::pair<double, double>> &resetWindow() const { return reset_window_; }

  /// Restart the detector from this position: a deliberate offset change is not a jump.
  /// A non-finite position is not recorded, keeping the baseline finite for the jump arithmetic.
  void rebaseDetector( double actuator_position )
  {
    if ( std::isfinite( actuator_position ) ) {
      prev_actuator_position_ = actuator_position;
    }
  }

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
  std::optional<std::pair<double, double>> reset_window_;
  int correction_count_{ 0 };

  /// The number of turns a power cycle would have shifted the previous reading by, if any.
  std::optional<int> turnsOfAReset( double previous, double current ) const;

  static constexpr double TWO_PI = 2.0 * M_PI;
  // Tolerance for detecting a 2pi jump (allow ~10% margin)
  static constexpr double JUMP_TOLERANCE = 0.2 * M_PI;
  /// Largest turn count a power cycle can plausibly shift a reading by.
  static constexpr double MAX_RESET_TURNS = 1e6;
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
  const char *home = std::getenv( "HOME" );
  if ( home == nullptr ) {
    // Constructing a path from nullptr is undefined, and a robot brought up by systemd routinely
    // has no HOME. Refusing here is the only place that can still say why.
    throw transmission_interface::Exception(
        "HOME is not set, so the offset of joint '" +
        joint_name + "' has nowhere to be stored. An offset that cannot be persisted is lost on every restart." );
  }
  transmission_file_path_ = std::filesystem::path( home ) / ".ros" /
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
  if ( !std::isfinite( offset ) ) {
    // Every joint position is computed from this, and it would be persisted for the next boot too.
    RCLCPP_ERROR( rclcpp::get_logger( "AdjustableOffsetTransmission" ),
                  "Refusing a non-finite offset for joint '%s'.", joint_name_.c_str() );
    return;
  }
  offset_ = offset;
  saveTransmissionOffset();
}

inline std::optional<int> JointOffset::turnsOfAReset( double previous, double current ) const
{
  const double delta = current - previous;
  const double turns = std::round( delta / TWO_PI );
  // A turn count that does not fit an int is undefined to convert, and no actuator reset is that
  // far away. MAX_RESET_TURNS is far above any real one and far below where the cast breaks.
  if ( std::abs( turns ) > MAX_RESET_TURNS ) {
    return std::nullopt;
  }
  const int n_turns = static_cast<int>( turns );
  if ( n_turns == 0 || std::abs( delta - n_turns * TWO_PI ) >= JUMP_TOLERANCE ) {
    return std::nullopt; // not a whole number of turns apart
  }
  if ( !reset_window_ ) {
    return n_turns;
  }
  // A power cycle reports the previous position wrapped into the window, so the readings it could
  // produce are the ones congruent to previous that lie there: one value for a window a single turn
  // wide, two for the union of a signed and an unsigned turn. Asking only whether current falls in
  // the window would also admit everything between those values, which on a wider window is most of
  // a turn of real movement. The tolerance is the slop the delta test above already allowed, so a
  // candidate is not missed for measurement noise.
  const auto &[lower, upper] = *reset_window_;
  for ( double candidate = previous - TWO_PI * std::floor( ( previous - lower ) / TWO_PI );
        candidate < upper; candidate += TWO_PI ) {
    if ( std::abs( current - candidate ) < JUMP_TOLERANCE ) {
      return n_turns;
    }
  }
  RCLCPP_WARN( rclcpp::get_logger( "AdjustableOffsetTransmission" ),
               "Actuator of joint '%s' moved %d x 2pi between two readings (%.4f -> %.4f rad) "
               "without landing where a power cycle puts it ([%.4f, %.4f)). Treated as a real "
               "move, not corrected.",
               joint_name_.c_str(), n_turns, previous, current, lower, upper );
  return std::nullopt;
}

inline int JointOffset::observeActuatorPosition( double actuator_position, double reduction )
{
  if ( !std::isfinite( actuator_position ) || !std::isfinite( reduction ) || reduction == 0.0 ) {
    // Rounding a non-finite delta to a turn count is undefined, and the jump test cannot reject it
    // either: every comparison against NaN is false. A reduction that is not a real number turns
    // any correction it does compute into a non-finite offset. Nothing is recorded, so the next
    // real reading becomes the baseline.
    return 0;
  }
  int n_jumps = 0;
  if ( prev_actuator_position_.has_value() ) {
    if ( const auto turns = turnsOfAReset( *prev_actuator_position_, actuator_position ) ) {
      n_jumps = *turns;
      // The jump in joint space is n_jumps * 2pi / reduction; absorb it into the offset.
      offset_ -= n_jumps * TWO_PI / reduction;
      correction_count_ += std::abs( n_jumps );
      saveTransmissionOffset();
      RCLCPP_WARN( rclcpp::get_logger( "AdjustableOffsetTransmission" ),
                   "Detected %d x 2pi actuator position jump on joint '%s' "
                   "(actuator delta: %.4f rad). Corrected offset to %.6f.",
                   n_jumps, joint_name_.c_str(), actuator_position - *prev_actuator_position_,
                   offset_ );
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
  const auto logger = rclcpp::get_logger( "AdjustableOffsetTransmission" );
  if ( !std::filesystem::exists( transmission_file_path_ ) ) {
    RCLCPP_WARN( logger, "Transmission offset file does not exist: %s",
                 transmission_file_path_.string().c_str() );
    return;
  }
  std::ifstream file( transmission_file_path_ );
  if ( !file.is_open() ) {
    RCLCPP_ERROR( logger, "Failed to open transmission offset file: %s",
                  transmission_file_path_.string().c_str() );
    return;
  }

  double offset;
  file >> offset;
  // Tested before the scan below, which drives the stream to eof: an extraction that failed on a
  // truncated or empty file also ends there, so failbit only tells the two apart now.
  if ( file.fail() ) {
    RCLCPP_ERROR( logger, "Transmission offset file holds no number: %s",
                  transmission_file_path_.string().c_str() );
    return;
  }
  // Extraction stops at the newline without setting eofbit, so requiring eof rejects every file
  // that ends in one - which is every file written by an editor or by echo. What must be rejected
  // is trailing content, so the check is for anything that is not whitespace.
  bool trailing_content = false;
  for ( char c = 0; file.get( c ); ) {
    if ( std::isspace( static_cast<unsigned char>( c ) ) == 0 ) {
      trailing_content = true;
      break;
    }
  }
  if ( file.bad() || trailing_content || !std::isfinite( offset ) ) {
    RCLCPP_ERROR( logger, "Failed to read a valid offset from file: %s",
                  transmission_file_path_.string().c_str() );
    return;
  }
  offset_ = offset;
}

inline void JointOffset::saveTransmissionOffset() const
{
  const auto logger = rclcpp::get_logger( "AdjustableOffsetTransmission" );
  std::error_code error;
  std::filesystem::create_directories( transmission_file_path_.parent_path(), error );
  if ( error ) {
    // Reported rather than thrown: this runs from the control loop and from service callbacks,
    // and an exception out of either takes the process with it.
    RCLCPP_ERROR( logger, "Failed to create the directory for %s: %s",
                  transmission_file_path_.string().c_str(), error.message().c_str() );
    return;
  }

  // Written beside the target and renamed onto it. A crash or a full disk partway through leaves
  // the previous offset in place instead of the empty or half-written file that a direct write
  // would leave, which the loader cannot tell from a good one.
  std::filesystem::path temp_path = transmission_file_path_;
  temp_path += ".tmp";
  {
    std::ofstream file( temp_path );
    if ( !file.is_open() ) {
      RCLCPP_ERROR( logger, "Failed to open transmission offset file for writing: %s",
                    temp_path.string().c_str() );
      return;
    }
    // Enough digits that loading gives back the offset that was saved. The stream default is six
    // significant digits, which loses roughly 1e-5 rad per save/load cycle on a multi-turn offset.
    file << std::setprecision( std::numeric_limits<double>::max_digits10 ) << offset_;
    file.close();
    // After close(), which is where a deferred flush failure (a full disk) surfaces.
    if ( file.fail() ) {
      RCLCPP_ERROR( logger, "Failed to write offset to file: %s", temp_path.string().c_str() );
      std::filesystem::remove( temp_path, error );
      return;
    }
  }

  std::filesystem::rename( temp_path, transmission_file_path_, error );
  if ( error ) {
    RCLCPP_ERROR( logger, "Failed to move the new offset onto %s: %s",
                  transmission_file_path_.string().c_str(), error.message().c_str() );
    std::filesystem::remove( temp_path, error );
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
