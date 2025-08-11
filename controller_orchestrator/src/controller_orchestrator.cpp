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
  list_controllers_client_ = node_->create_client<ListControllers>(
      controller_manager_name_ + "/list_controllers", rclcpp::QoS( 10 ), callback_group_ );
  switch_controller_client_ = node_->create_client<SwitchController>(
      controller_manager_name_ + "/switch_controller", rclcpp::QoS( 10 ), callback_group_ );
  list_hardware_components_client_ =
      node_->create_client<controller_manager_msgs::srv::ListHardwareComponents>(
          controller_manager_name_ + "/list_hardware_components", rclcpp::QoS( 10 ), callback_group_ );
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
bool ControllerOrchestrator::smartSwitchControllerAnalysis(
    std::vector<std::string> &to_activate, std::vector<std::string> &to_deactivate,
    const controller_manager_msgs::srv::ListControllers_Response &res ) const
{
  // Build a map: controller name → vector of claimed interfaces
  std::unordered_map<std::string, std::vector<std::string>> controller_to_resources;
  std::unordered_map<std::string, std::vector<std::string>> chain_connections;
  std::vector<std::string> currently_active;

  // remove controller from to activate that do not exist
  for ( auto it = to_activate.begin(); it != to_activate.end(); ) {
    if ( std::find_if( res.controller.begin(), res.controller.end(), [&it]( const auto &ctrl ) {
           return ctrl.name == *it;
         } ) == res.controller.end() ) {
      RCLCPP_WARN(
          node_->get_logger(),
          "[ControllerOrchestrator] Controller '%s' not found in list_controllers response; "
          "removing from activation list.",
          it->c_str() );
      it = to_activate.erase( it );
    } else {
      ++it;
    }
  }

  // Remove controllers from the activation list that are already active
  for ( const auto &ctrl_state : res.controller ) {
    const std::string &name = ctrl_state.name;
    if ( ctrl_state.state == "active" ) {
      currently_active.push_back( name );
      if ( std::find( to_activate.begin(), to_activate.end(), name ) != to_activate.end() ) {
        RCLCPP_DEBUG(
            node_->get_logger(), "[ControllerOrchestrator] Controller '%s' is already active; removing from activation list.",
            name.c_str() );
        to_activate.erase( std::remove( to_activate.begin(), to_activate.end(), name ),
                           to_activate.end() );
      }
    }
    // claimed_interfaces is already a flat string[]
    controller_to_resources[name] = { ctrl_state.required_command_interfaces.begin(),
                                      ctrl_state.required_command_interfaces.end() };
    for ( const auto &chain : ctrl_state.chain_connections ) {
      chain_connections[name].push_back( chain.name );
    }
  }

  if ( to_activate.empty() ) {
    RCLCPP_DEBUG(
        node_->get_logger(),
        "[ControllerOrchestrator] No new controllers to activate; all are already active." );
    return true;
  }

  // Recursively, add all controllers of a group with at least one to_activate controller
  bool added = true;
  while ( added ) {
    added = false;
    std::vector<std::string> new_members;
    for ( const auto &name : to_activate ) {
      for ( const auto &group_member : chain_connections[name] ) {
        if ( std::find( to_activate.begin(), to_activate.end(), group_member ) == to_activate.end() &&
             std::find( new_members.begin(), new_members.end(), group_member ) == new_members.end() ) {
          new_members.push_back( group_member );
        }
      }
    }
    if ( !new_members.empty() ) {
      to_activate.insert( to_activate.end(), new_members.begin(), new_members.end() );
      added = true;
    }
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

  // Recursively, add all active controllers that depend on a controller in to_deactivate
  // they cannot remain active if one of their chain connections is to be deactivated
  added = true;
  while ( added ) {
    added = false;
    std::vector<std::string> new_members;
    for ( const auto &name : currently_active ) {
      for ( const auto &chain_connection : chain_connections[name] ) {
        if ( std::find( to_deactivate.begin(), to_deactivate.end(), chain_connection ) ==
                 to_deactivate.end() &&
             std::find( new_members.begin(), new_members.end(), chain_connection ) ==
                 new_members.end() ) {
          new_members.push_back( chain_connection );
        }
      }
    }
    if ( !new_members.empty() ) {
      to_deactivate.insert( to_deactivate.end(), new_members.begin(), new_members.end() );
      added = true;
    }
  }

  return true;
}

bool ControllerOrchestrator::smartSwitchController( std::vector<std::string> &activate_controllers,
                                                    int timeout_s ) const
{
  // 1) Get controller information from controller manager (blocking call)
  if ( !list_controllers_client_->wait_for_service( std::chrono::seconds( timeout_s ) ) ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "[ControllerOrchestrator] list_controllers service not available" );
    return false;
  }

  auto list_req = std::make_shared<ListControllers::Request>();
  auto list_future = list_controllers_client_->async_send_request( list_req );

  // Block until either the future is ready or we exceed timeout_s
  const auto status = list_future.wait_for( std::chrono::seconds( timeout_s ) );
  if ( status == std::future_status::timeout ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "[ControllerOrchestrator] list_controllers service call timed out (>%d s)",
                  timeout_s );
    return false;
  }
  auto list_resp = list_future.get();
  if ( !list_resp ) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "[ControllerOrchestrator] list_controllers service call failed or returned null" );
    return false;
  }

  // 2) Analyze which controllers to activate and which to deactivate
  std::vector<std::string> to_deactivate;
  if ( !smartSwitchControllerAnalysis( activate_controllers, to_deactivate, *list_resp ) ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "[ControllerOrchestrator] smartSwitchControllerAnalysis failed" );
    return false;
  }
  if ( activate_controllers.empty() && to_deactivate.empty() ) {
    RCLCPP_DEBUG( node_->get_logger(),
                  "[ControllerOrchestrator] No controllers to activate or deactivate." );
    return true;
  }
  // 3) Finally, call switch_controller with start=to_activate, stop=to_deactivate (blocking call)
  if ( !switch_controller_client_->wait_for_service( std::chrono::seconds( timeout_s ) ) ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "[ControllerOrchestrator] switch_controller service not available" );
    return false;
  }
  const auto switch_req = std::make_shared<SwitchController::Request>();
  switch_req->activate_controllers = activate_controllers;
  switch_req->deactivate_controllers = to_deactivate;
  switch_req->strictness = SwitchController::Request::BEST_EFFORT;
  switch_req->activate_asap = false;
  switch_req->timeout.sec = 0; // zero = infinite
  switch_req->timeout.nanosec = 0u;

  auto switch_future = switch_controller_client_->async_send_request( switch_req );

  // Block until either the future is ready or we exceed timeout_s
  auto switch_status = switch_future.wait_for( std::chrono::seconds( timeout_s ) );
  if ( switch_status == std::future_status::timeout ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "[ControllerOrchestrator] switch_controller service call timed out (>%d s)",
                  timeout_s );
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
               activate_controllers.size(), to_deactivate.size() );
  return true;
}

std::vector<std::string> ControllerOrchestrator::getActiveControllerOfHardwareInterface(
    const std::string &hardware_interface, int timeout_s ) const
{
  // 1) Wait for and call "list_hardware_components"
  if ( !list_hardware_components_client_->wait_for_service( std::chrono::seconds( timeout_s ) ) ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "[ControllerOrchestrator] getActiveControllerOfHardwareInterface: "
                  "list_hardware_components service not available after %d s",
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
    RCLCPP_ERROR( node_->get_logger(),
                  "[ControllerOrchestrator] switch_controller service call timed out (>%d s)",
                  timeout_s );
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
    RCLCPP_ERROR( node_->get_logger(),
                  "[ControllerOrchestrator] switch_controller service call timed out (>%d s)",
                  timeout_s );
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
    RCLCPP_ERROR( node_->get_logger(),
                  "[ControllerOrchestrator] list_controllers service call timed out (>%d s)",
                  timeout_s );
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
    RCLCPP_INFO( node_->get_logger(),
                 "[ControllerOrchestrator] No active controllers claiming joint '%s'",
                 joint_name.c_str() );
    return true;
  }

  // Step 4: Deactivate the identified controllers
  return deactivateControllers( controllers_to_deactivate, timeout_s );
}

} // namespace controller_orchestrator
