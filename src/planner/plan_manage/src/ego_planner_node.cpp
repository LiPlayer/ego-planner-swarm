#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <iostream>

#include <ego_planner/ego_replan_fsm.h>

using namespace ego_planner;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ego_planner_node");

  EGOReplanFSM rebo_replan;

  rebo_replan.init(node);

  // Ensure the node is not already marked as attached to an executor (can happen if spin_some was used earlier).
  auto base = node->get_node_base_interface();
  bool already_associated = base->get_associated_with_executor_atomic().exchange(false);
  if (already_associated)
  {
    RCLCPP_WARN(node->get_logger(), "Resetting stale executor association before spinning.");
  }

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}

// #include <ros/ros.h>
// #include <csignal>
// #include <visualization_msgs/Marker.h>

// #include <plan_manage/ego_replan_fsm.h>

// using namespace ego_planner;

// void SignalHandler(int signal) {
//   if(ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()){
//     ros::shutdown();
//   }
// }

// int main(int argc, char **argv) {

//   signal(SIGINT, SignalHandler);
//   signal(SIGTERM,SignalHandler);

//   ros::init(argc, argv, "ego_planner_node", ros::init_options::NoSigintHandler);
//   ros::NodeHandle nh("~");

//   EGOReplanFSM rebo_replan;

//   rebo_replan.init(nh);

//   // ros::Duration(1.0).sleep();
//   ros::AsyncSpinner async_spinner(4);
//   async_spinner.start();
//   ros::waitForShutdown();

//   return 0;
// }
