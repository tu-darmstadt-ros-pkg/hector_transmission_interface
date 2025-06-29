//
// Created by aljoscha-schmidt on 6/4/25.
//

#ifndef DYNAMIC_OFFSET_TRANSMISSION_HPP
#define DYNAMIC_OFFSET_TRANSMISSION_HPP

#include <controller_orchestrator/controller_orchestrator.hpp>
#include <filesystem>
#include <fstream>
#include <hector_transmission_interface_msgs/srv/set_transmission_offset.hpp>
#include <transmission_interface/simple_transmission.hpp>
namespace hector_transmission_interface
{
class AdjustableOffsetTransmission : public transmission_interface::SimpleTransmission
{
public:
  explicit AdjustableOffsetTransmission( const std::string &joint_name,
                                         double joint_to_actuator_reduction,
                                         double joint_offset = 0.0 );
  /***
   * \brief
   * Adjusts the transmission offset for the joint.
   * This will also save the new offset to the file system.
   * WARNING: MAKE SURE TO DEACTIVATE THE CONTROLLER BEFORE CHANGING THE OFFSET!
   * @param offset The new offset to set for the joint.
   */
  void adjustTransmissionOffset( double offset );

private:
  void loadTransmissionOffset();
  void saveTransmissionOffset() const;

  std::string joint_name_;
  std::filesystem::path transmission_file_path_;
};

inline AdjustableOffsetTransmission::AdjustableOffsetTransmission( const std::string &joint_name,
                                                                   double joint_to_actuator_reduction,
                                                                   double joint_offset )
    : SimpleTransmission( joint_to_actuator_reduction, joint_offset ), joint_name_( joint_name )
{
  // path is ~/.ros/dynamic_offset_transmissions/ + joint_name + ".yaml"
  transmission_file_path_ = std::filesystem::path( std::getenv( "HOME" ) ) / ".ros" /
                            "dynamic_offset_transmissions" / ( joint_name + ".txt" );
  loadTransmissionOffset(); // if file exists the offset is read and set
}

inline void AdjustableOffsetTransmission::loadTransmissionOffset()
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
        jnt_offset_ = offset;
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

inline void AdjustableOffsetTransmission::saveTransmissionOffset() const
{
  // Save the current transmission offset to the specified storage path
  std::filesystem::create_directories( transmission_file_path_.parent_path() );
  std::ofstream file( transmission_file_path_ );
  if ( file.is_open() ) {
    file << jnt_offset_;
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

inline void AdjustableOffsetTransmission::adjustTransmissionOffset( double offset )
{
  jnt_offset_ = offset;
  saveTransmissionOffset();
}
} // namespace hector_transmission_interface
#endif // DYNAMIC_OFFSET_TRANSMISSION_HPP
