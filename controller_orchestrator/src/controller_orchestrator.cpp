#include "controller_orchestrator/controller_orchestrator.hpp"
#include <chrono>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <unordered_set>

namespace controller_orchestrator
{

using ListControllers = controller_manager_msgs::srv::ListControllers;
using SwitchController = controller_manager_msgs::srv::SwitchController;

ControllerOrchestrator::ControllerOrchestrator( rclcpp::Node::SharedPtr node,
                                                const std::string &controller_manager_name )
    : node_( node ), controller_manager_name_( controller_manager_name )
{
  callback_group_ = node_->create_callback_group( rclcpp::CallbackGroupType::Reentrant );
  list_controllers_client_ =
      node_->create_client<ListControllers>( controller_manager_name_ + "/list_controllers",
                                             rmw_qos_profile_services_default, callback_group_ );
  switch_controller_client_ =
      node_->create_client<SwitchController>( controller_manager_name_ + "/switch_controller",
                                              rmw_qos_profile_services_default, callback_group_ );
}

bool ControllerOrchestrator::smartSwitchController( std::vector<std::string> &activate_controllers,
                                                    int timeout_s )
{
  // 1) Wait for and call "list_controllers" to retrieve all controllers’ states
  if ( !list_controllers_client_->wait_for_service( std::chrono::seconds( timeout_s ) ) ) {
    RCLCPP_ERROR( node_->get_logger(), "list_controllers service not available" );
    return false;
  }

  auto list_req = std::make_shared<ListControllers::Request>();
  RCLCPP_INFO( node_->get_logger(),
               "Calling list_controllers service to retrieve controller states..." );
  auto list_future = list_controllers_client_->async_send_request( list_req );

  // Block until either the future is ready or we exceed timeout_s
  auto status = list_future.wait_for( std::chrono::seconds( timeout_s ) );
  if ( status == std::future_status::timeout ) {
    RCLCPP_ERROR( node_->get_logger(), "list_controllers service call timed out (>%d s)", timeout_s );
    return false;
  }
  auto list_resp = list_future.get();
  if ( !list_resp ) {
    RCLCPP_ERROR( node_->get_logger(), "list_controllers service call failed or returned null" );
    return false;
  }
  RCLCPP_INFO( node_->get_logger(), "Received list_controllers response with [%zu] controllers.",
               list_resp->controller.size() );

  // Build a map: controller name → vector of claimed interfaces
  std::unordered_map<std::string, std::vector<std::string>> controller_to_resources;
  // Keep track of which controllers are currently active
  std::unordered_set<std::string> currently_active;

  for ( const auto &ctrl_state : list_resp->controller ) {
    const std::string &name = ctrl_state.name;
    if ( ctrl_state.state == "active" ) {
      currently_active.insert( name );
    }
    // claimed_interfaces is already a flat string[]
    controller_to_resources[name] = { ctrl_state.required_command_interfaces.begin(),
                                      ctrl_state.required_command_interfaces.end() };
  }

  // 2) Skip any controllers in activate_controllers that are already active
  std::vector<std::string> to_activate;
  to_activate.reserve( activate_controllers.size() );
  for ( const auto &name : activate_controllers ) {
    if ( currently_active.count( name ) ) {
      RCLCPP_INFO( node_->get_logger(), "Controller '%s' is already active → skipping activation.",
                   name.c_str() );
    } else {
      if ( controller_to_resources.find( name ) == controller_to_resources.end() ) {
        RCLCPP_ERROR( node_->get_logger(),
                      "Controller '%s' was not found in list_controllers response", name.c_str() );
        return false;
      }
      to_activate.push_back( name );
    }
  }

  if ( to_activate.empty() ) {
    RCLCPP_INFO( node_->get_logger(), "No new controllers to activate; all are already active." );
    return true;
  }

  // 3) Build a set of all interfaces needed by the controllers we will activate
  std::unordered_set<std::string> needed_resources;
  needed_resources.reserve( to_activate.size() * 4 );

  for ( const auto &name : to_activate ) {
    const auto &resources = controller_to_resources.at( name );
    for ( const auto &res : resources ) {
      if ( needed_resources.count( res ) ) {
        RCLCPP_ERROR( node_->get_logger(),
                      "Resource conflict among requested controllers: "
                      "interface '%s' requested by more than one controller.",
                      res.c_str() );
        return false;
      }
      needed_resources.insert( res );
    }
  }

  // 4) Compare needed_resources against interfaces held by currently active controllers
  std::vector<std::string> to_deactivate;
  to_deactivate.reserve( currently_active.size() );

  for ( const auto &active_name : currently_active ) {
    const auto &active_resources = controller_to_resources.at( active_name );
    bool conflict = false;
    for ( const auto &res : active_resources ) {
      if ( needed_resources.count( res ) ) {
        conflict = true;
        break;
      }
    }
    if ( conflict ) {
      RCLCPP_WARN( node_->get_logger(),
                   "Active controller '%s' will be stopped due to resource conflict.",
                   active_name.c_str() );
      to_deactivate.push_back( active_name );
    }
  }

  // 5) Finally, call switch_controller with start=to_activate, stop=to_deactivate
  if ( !switch_controller_client_->wait_for_service( std::chrono::seconds( timeout_s ) ) ) {
    RCLCPP_ERROR( node_->get_logger(), "switch_controller service not available" );
    return false;
  }

  auto switch_req = std::make_shared<SwitchController::Request>();
  switch_req->activate_controllers = to_activate;
  switch_req->deactivate_controllers = to_deactivate;
  switch_req->strictness = SwitchController::Request::BEST_EFFORT;
  switch_req->activate_asap = false;
  switch_req->timeout.sec = 0; // zero = infinite
  switch_req->timeout.nanosec = 0u;

  auto switch_future = switch_controller_client_->async_send_request( switch_req );

  // Block until either the future is ready or we exceed timeout_s
  auto switch_status = switch_future.wait_for( std::chrono::seconds( timeout_s ) );
  if ( switch_status == std::future_status::timeout ) {
    RCLCPP_ERROR( node_->get_logger(), "switch_controller service call timed out (>%d s)", timeout_s );
    return false;
  }
  auto switch_resp = switch_future.get();
  if ( !switch_resp || !switch_resp->ok ) {
    RCLCPP_ERROR( node_->get_logger(), "switch_controller service returned ok=false%s",
                  ( switch_resp ? std::string( ": " + switch_resp->message ) : std::string( "" ) ) );
    return false;
  }

  RCLCPP_INFO( node_->get_logger(),
               "Successfully switched controllers. Activated [%zu], deactivated [%zu].",
               to_activate.size(), to_deactivate.size() );
  return true;
}

} // namespace controller_orchestrator
