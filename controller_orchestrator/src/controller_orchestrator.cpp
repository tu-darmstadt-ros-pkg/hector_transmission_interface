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

ControllerOrchestrator::ControllerOrchestrator( const rclcpp::Node::SharedPtr &node,
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
  list_hardware_components_client_ =
      node_->create_client<controller_manager_msgs::srv::ListHardwareComponents>(
          controller_manager_name_ + "/list_hardware_components", rmw_qos_profile_services_default,
          callback_group_ );
}

void ControllerOrchestrator::smartSwitchControllerAsync(
    std::vector<std::string> activate_controllers,
    std::function<void( bool success, const std::string &message )> callback ) const
{
  if ( activate_controllers.empty() ) {
    callback( true, "No controllers requested" );
    return;
  }

  if ( !list_controllers_client_->service_is_ready() ) {
    callback( false, "list_controllers service not ready" );
    return;
  }

  auto list_req = std::make_shared<ListControllers::Request>();
  list_controllers_client_->async_send_request(
      list_req, [=]( rclcpp::Client<ListControllers>::SharedFuture list_future ) {
        const auto list_resp = list_future.get();
        if ( !list_resp ) {
          callback( false, "list_controllers returned null response" );
          return;
        }

        std::unordered_map<std::string, std::vector<std::string>> controller_to_resources;
        std::unordered_set<std::string> currently_active;
        for ( const auto &ctrl : list_resp->controller ) {
          if ( ctrl.state == "active" )
            currently_active.insert( ctrl.name );
          controller_to_resources[ctrl.name] = { ctrl.required_command_interfaces.begin(),
                                                 ctrl.required_command_interfaces.end() };
        }

        std::vector<std::string> to_activate;
        for ( const auto &name : activate_controllers ) {
          if ( currently_active.count( name ) )
            continue;
          if ( controller_to_resources.count( name ) == 0 ) {
            callback( false, "Controller '" + name + "' not found in list_controllers response" );
            return;
          }
          to_activate.push_back( name );
        }

        if ( to_activate.empty() ) {
          callback( true, "All controllers already active" );
          return;
        }

        std::unordered_set<std::string> needed_resources;
        for ( const auto &name : to_activate ) {
          for ( const auto &res : controller_to_resources.at( name ) ) {
            if ( !needed_resources.insert( res ).second ) {
              callback( false, "Resource conflict: interface '" + res +
                                   "' claimed by multiple controllers" );
              return;
            }
          }
        }

        std::vector<std::string> to_deactivate;
        for ( const auto &active_name : currently_active ) {
          const auto &resources = controller_to_resources.at( active_name );
          for ( const auto &res : resources ) {
            if ( needed_resources.count( res ) ) {
              to_deactivate.push_back( active_name );
              break;
            }
          }
        }

        if ( !switch_controller_client_->service_is_ready() ) {
          callback( false, "switch_controller service not ready" );
          return;
        }

        auto switch_req = std::make_shared<SwitchController::Request>();
        switch_req->activate_controllers = to_activate;
        switch_req->deactivate_controllers = to_deactivate;
        switch_req->strictness = SwitchController::Request::BEST_EFFORT;
        switch_req->activate_asap = false;
        switch_req->timeout.sec = 0;
        switch_req->timeout.nanosec = 0;

        switch_controller_client_->async_send_request(
            switch_req, [=]( rclcpp::Client<SwitchController>::SharedFuture switch_future ) {
              const auto switch_resp = switch_future.get();
              if ( !switch_resp || !switch_resp->ok ) {
                std::string msg = switch_resp ? switch_resp->message : "null response";
                callback( false, "Switching failed: " + msg );
                return;
              }
              callback( true, "Switched controllers successfully" );
            } );
      } );
}
bool ControllerOrchestrator::smartSwitchController( std::vector<std::string> &activate_controllers,
                                                    int timeout_s ) const
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
    RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "switch_controller service returned ok=false%s"
            << ( switch_resp ? std::string( ": " + switch_resp->message ) : std::string( "" ) ) );
    return false;
  }

  RCLCPP_INFO( node_->get_logger(),
               "Successfully switched controllers. Activated [%zu], deactivated [%zu].",
               to_activate.size(), to_deactivate.size() );
  return true;
}

std::vector<std::string> ControllerOrchestrator::getActiveControllerOfHardwareInterface(
    const std::string &hardware_interface, int timeout_s ) const
{
  // 1) Wait for and call "list_hardware_components"
  if ( !list_hardware_components_client_->wait_for_service( std::chrono::seconds( timeout_s ) ) ) {
    RCLCPP_ERROR(
        node_->get_logger(), "getActiveControllerOfHardwareInterface: list_hardware_components service not available after %d s",
        timeout_s );
    return {};
  }
  auto hw_req = std::make_shared<ListHardwareComponents::Request>();
  RCLCPP_INFO( node_->get_logger(),
               "getActiveControllerOfHardwareInterface: calling %s/list_hardware_components...",
               controller_manager_name_.c_str() );
  auto hw_future = list_hardware_components_client_->async_send_request( hw_req );

  if ( hw_future.wait_for( std::chrono::seconds( timeout_s ) ) == std::future_status::timeout ) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "getActiveControllerOfHardwareInterface: list_hardware_components call timed out (>%d s)",
        timeout_s );
    return {};
  }
  auto hw_resp = hw_future.get();
  if ( !hw_resp ) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "getActiveControllerOfHardwareInterface: list_hardware_components returned null" );
    return {};
  }

  // 2) Find the component matching hardware_interface and collect its command interfaces
  std::unordered_set<std::string> target_command_interfaces;
  bool found_component = false;
  for ( const auto &comp : hw_resp->component ) {
    if ( comp.name == hardware_interface ) {
      found_component = true;
      for ( const auto &hw_if : comp.command_interfaces ) {
        target_command_interfaces.insert( hw_if.name );
      }
      break;
    }
  }
  if ( !found_component ) {
    RCLCPP_WARN( node_->get_logger(),
                 "getActiveControllerOfHardwareInterface: hardware component '%s' not found",
                 hardware_interface.c_str() );
    return {};
  }
  if ( target_command_interfaces.empty() ) {
    RCLCPP_INFO( node_->get_logger(),
                 "getActiveControllerOfHardwareInterface: hardware '%s' has no command interfaces",
                 hardware_interface.c_str() );
    return {};
  }

  // 3) Wait for and call "list_controllers"
  if ( !list_controllers_client_->wait_for_service( std::chrono::seconds( timeout_s ) ) ) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "getActiveControllerOfHardwareInterface: list_controllers service not available after %d s",
        timeout_s );
    return {};
  }
  auto list_req = std::make_shared<ListControllers::Request>();
  RCLCPP_INFO( node_->get_logger(),
               "getActiveControllerOfHardwareInterface: calling %s/list_controllers...",
               controller_manager_name_.c_str() );
  auto list_future = list_controllers_client_->async_send_request( list_req );

  if ( list_future.wait_for( std::chrono::seconds( timeout_s ) ) == std::future_status::timeout ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "getActiveControllerOfHardwareInterface: list_controllers call timed out (>%d s)",
                  timeout_s );
    return {};
  }
  auto list_resp = list_future.get();
  if ( !list_resp ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "getActiveControllerOfHardwareInterface: list_controllers returned null" );
    return {};
  }

  // 4) Iterate through each controller; if active and any claimed_interface ∈ target_command_interfaces,
  //    add its name to the result.
  std::vector<std::string> active_controllers;
  for ( const auto &ctrl : list_resp->controller ) {
    if ( ctrl.state != "active" ) {
      continue;
    }
    for ( const auto &claimed_if : ctrl.claimed_interfaces ) {
      if ( target_command_interfaces.count( claimed_if ) > 0 ) {
        active_controllers.push_back( ctrl.name );
        break;
      }
    }
  }

  return active_controllers;
}
bool ControllerOrchestrator::deactivateControllers(
    const std::vector<std::string> &controllers_to_deactivate, int timeout_s ) const
{
  auto switch_req = std::make_shared<SwitchController::Request>();
  switch_req->deactivate_controllers = controllers_to_deactivate;
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
    RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "switch_controller service returned ok=false%s"
            << ( switch_resp ? std::string( ": " + switch_resp->message ) : std::string( "" ) ) );
    return false;
  }
  return true;
}

bool ControllerOrchestrator::activateControllers(
    const std::vector<std::string> &controllers_to_activate, int timeout_s ) const
{
  auto switch_req = std::make_shared<SwitchController::Request>();
  switch_req->activate_controllers = controllers_to_activate;
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
    RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "switch_controller service returned ok=false%s"
            << ( switch_resp ? std::string( ": " + switch_resp->message ) : std::string( "" ) ) );
    return false;
  }
  return true;
}

bool ControllerOrchestrator::unloadControllersOfJoint( const std::string &joint_name, int timeout_s )
{
  // Step 1: Request list of controllers
  auto list_request = std::make_shared<ListControllers::Request>();
  auto list_future = list_controllers_client_->async_send_request( list_request );
  auto list_status = list_future.wait_for( std::chrono::seconds( timeout_s ) );

  if ( list_status == std::future_status::timeout ) {
    RCLCPP_ERROR( node_->get_logger(), "list_controllers service call timed out (>%d s)", timeout_s );
    return false;
  }

  auto list_resp = list_future.get();
  std::vector<std::string> controllers_to_deactivate;

  // Step 2: Iterate through active controllers and check if they claim the joint
  for ( const auto &ctrl : list_resp->controller ) {
    if ( ctrl.state == "active" ) {
      for ( const auto &claimed_if : ctrl.claimed_interfaces ) {
        // Example claimed interface: "joint1/position", "joint2/velocity"
        const auto sep_pos = claimed_if.find( '/' );
        if ( sep_pos != std::string::npos ) {
          std::string claimed_joint = claimed_if.substr( 0, sep_pos );
          if ( claimed_joint == joint_name ) {
            controllers_to_deactivate.push_back( ctrl.name );
            break; // No need to check more interfaces for this controller
          }
        }
      }
    }
  }

  // Step 3: If none to deactivate, return early
  if ( controllers_to_deactivate.empty() ) {
    RCLCPP_INFO( node_->get_logger(), "No active controllers claiming joint '%s'",
                 joint_name.c_str() );
    return true;
  }

  // Step 4: Deactivate the identified controllers
  return deactivateControllers( controllers_to_deactivate, timeout_s );
}

} // namespace controller_orchestrator
