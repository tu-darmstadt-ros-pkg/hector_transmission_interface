//
// Created by aljoscha-schmidt on 6/4/25.
//

#ifndef DYNAMIC_OFFSET_TRANSMISSION_HPP
#define DYNAMIC_OFFSET_TRANSMISSION_HPP

#include <filesystem>
#include <fstream>
#include <hector_transmission_interface_msgs/srv/set_transmission_offset.hpp>
#include <transmission_interface/simple_transmission.hpp>

namespace hector_transmission_interface
{
class DynamicOffsetTransmission : public transmission_interface::SimpleTransmission
{
public:
  explicit DynamicOffsetTransmission( const rclcpp::Node::SharedPtr node,
                                      const std::string &joint_name,
                                      const double joint_to_actuator_reduction,
                                      const double joint_offset = 0.0 );
  bool setTransmissionOffset(
      const hector_transmission_interface_msgs::srv::SetTransmissionOffset::Request::SharedPtr request,
      hector_transmission_interface_msgs::srv::SetTransmissionOffset::Response::SharedPtr response );

private:
  void loadTransmissionOffset();
  void saveTransmissionOffset() const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<hector_transmission_interface_msgs::srv::SetTransmissionOffset> set_offset_service_;
  std::string joint_name_;
  std::filesystem::path transmission_file_path_;
};

inline DynamicOffsetTransmission::DynamicOffsetTransmission( const rclcpp::Node::SharedPtr node,
                                                             const std::string &joint_name,
                                                             const double joint_to_actuator_reduction,
                                                             const double joint_offset )
    : SimpleTransmission( joint_to_actuator_reduction, joint_offset ), node_( node ),
      set_offset_service_(), joint_name_( joint_name )
{
  // path is ~/.ros/dynamic_offset_transmissions/ + joint_name + ".yaml"
  transmission_file_path_ = std::filesystem::path( std::getenv( "HOME" ) ) / ".ros" /
                            "dynamic_offset_transmissions" / ( joint_name + ".txt" );
}

inline void DynamicOffsetTransmission::loadTransmissionOffset()
{
  // Load the transmission offset from the specified storage path
  if ( std::filesystem::exists( transmission_file_path_ ) ) {
    std::ifstream file( transmission_file_path_ );
    if ( file.is_open() ) {
      double offset;
      file >> offset;
      if ( file.fail() || !file.eof() ) {
        RCLCPP_ERROR( node_->get_logger(), "Failed to read a valid offset from file: %s",
                      transmission_file_path_.string().c_str() );
      } else {
        jnt_offset_ = offset;
      }
      file.close();
    } else {
      RCLCPP_ERROR( node_->get_logger(), "Failed to open transmission offset file: %s",
                    transmission_file_path_.string().c_str() );
    }
  } else {
    RCLCPP_WARN( node_->get_logger(), "Transmission offset file does not exist: %s",
                 transmission_file_path_.string().c_str() );
  }
}

inline void DynamicOffsetTransmission::saveTransmissionOffset() const
{
  // Save the current transmission offset to the specified storage path
  std::filesystem::create_directories( transmission_file_path_.parent_path() );
  std::ofstream file( transmission_file_path_ );
  if ( file.is_open() ) {
    file << jnt_offset_;
    if ( file.fail() ) {
      RCLCPP_ERROR( node_->get_logger(), "Failed to write offset to file: %s",
                    transmission_file_path_.string().c_str() );
    }
    file.close();
  } else {
    RCLCPP_ERROR( node_->get_logger(), "Failed to open transmission offset file for writing: %s",
                  transmission_file_path_.string().c_str() );
  }
}

inline bool DynamicOffsetTransmission::setTransmissionOffset(
    const hector_transmission_interface_msgs::srv::SetTransmissionOffset::Request::SharedPtr request,
    hector_transmission_interface_msgs::srv::SetTransmissionOffset::Response::SharedPtr response )
{
  jnt_offset_ = request->offset;
  saveTransmissionOffset();
  response->success = true;
  return true;
}
} // namespace hector_transmission_interface
#endif // DYNAMIC_OFFSET_TRANSMISSION_HPP
