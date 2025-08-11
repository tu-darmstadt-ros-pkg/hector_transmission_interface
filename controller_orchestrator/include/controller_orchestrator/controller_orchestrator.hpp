#ifndef CONTROLLER_ORCHESTRATOR_CONTROLLER_ORCHESTRATOR_HPP
#define CONTROLLER_ORCHESTRATOR_CONTROLLER_ORCHESTRATOR_HPP

#include <memory>

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/list_hardware_components.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>

namespace controller_orchestrator
{

class ControllerOrchestrator
{
  using ListControllers = controller_manager_msgs::srv::ListControllers;
  using SwitchController = controller_manager_msgs::srv::SwitchController;
  using ListHardwareComponents = controller_manager_msgs::srv::ListHardwareComponents;

public:
  explicit ControllerOrchestrator(
      const rclcpp::Node::SharedPtr &node,
      const std::string &controller_manager_name = "controller_manager" );

  /**
   * Tries to activate the given controllers, deactivating all conflicting controllers (all
   * controllers that are currently active and claim the same resources).
   *
   * Can be replaced with strictness FORCE_AUTO, after implementing
   * @param activate_controllers list of controllers to activate
   * @param timeout_s timeout in seconds for the operation, defaults to 2.0
   * of the controllers to be activated is already active.
   * @return
   */
  bool smartSwitchController( std::vector<std::string> &activate_controllers,
                              int timeout_s = 2 ) const;

  void smartSwitchControllerAsync(
      const std::vector<std::string> &activate_controllers,
      const std::function<void( bool success, const std::string &message )> &callback ) const;

  std::vector<std::string> getActiveControllerOfHardwareInterface( const std::string &hardware_interface,
                                                                   int timeout_s = 2 ) const;

  bool deactivateControllers( const std::vector<std::string> &controllers_to_deactivate,
                              int timeout_s = 2 ) const;

  bool activateControllers( const std::vector<std::string> &controllers_to_activate,
                            int timeout_s = 2 ) const;

  /*bool activateControllersOfHardwareInterface( const std::string &hardware_interface,
                                               const std::vector<std::string>
     &controllers_to_activate, int timeout_s = 2 );*/
  bool unloadControllersOfJoint( const std::string &joint_name, int timeout_s = 2 );

private:
  bool smartSwitchControllerAnalysis(
      std::vector<std::string> &to_activate, std::vector<std::string> &to_deactivate,
      const controller_manager_msgs::srv::ListControllers_Response &req ) const;
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::string controller_manager_name_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListHardwareComponents>::SharedPtr
      list_hardware_components_client_;
};

} // namespace controller_orchestrator

#endif // CONTROLLER_ORCHESTRATOR_CONTROLLER_ORCHESTRATOR_HPP
