#include <memory>
#include <string>
#include <vector>

#include "controller_orchestrator/controller_orchestrator.hpp"
#include <rclcpp/rclcpp.hpp>

int main( int argc, char **argv )
{
  // 1) Initialize ROS 2
  rclcpp::init( argc, argv );

  // 2) Create a ROS 2 node
  auto node = rclcpp::Node::make_shared( "test_smart_switch_node" );

  // 3) Instantiate the ControllerOrchestrator, pointing to your controller_manager
  //    (replace "/athena/controller_manager" if your namespace is different)
  auto orchestrator = std::make_shared<controller_orchestrator::ControllerOrchestrator>(
      node, "/athena/controller_manager" );

  // 4) Set up a one‐shot timer to invoke smartSwitchController after ~1 second
  rclcpp::TimerBase::SharedPtr timer =
      node->create_wall_timer( std::chrono::seconds( 1 ), [orchestrator, node]() {
        static bool called = false;
        if ( called ) {
          return; // Only run once
        }
        called = true;

        // Replace these names with controllers that actually exist on your system
        std::vector<std::string> controllers_to_activate = {
            "moveit_twist_controller",
            //"arm_trajectory_controller"
        };

        RCLCPP_INFO( node->get_logger(), "[Test] Calling smartSwitchController(...) with [%s, %s]",
                     controllers_to_activate[0].c_str(), controllers_to_activate[0].c_str() );

        // Wait up to 10 seconds for each service call
        bool success = orchestrator->smartSwitchController( controllers_to_activate, 10 );
        if ( success ) {
          RCLCPP_INFO( node->get_logger(),
                       "[Test] smartSwitchController returned true (switch succeeded)" );
        } else {
          RCLCPP_ERROR( node->get_logger(),
                        "[Test] smartSwitchController returned false (switch failed)" );
        }
      } );

  // 5) Create a MultiThreadedExecutor with at least 2 threads.
  //    This lets one thread block on future.wait_for(...) while another
  //    thread handles incoming service callbacks.
  rclcpp::executors::MultiThreadedExecutor executor( rclcpp::ExecutorOptions(),
                                                     2 // two threads minimum
  );
  executor.add_node( node );

  // 6) Spin until shutdown (Ctrl+C). The timer will fire once after ~1 second.
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
