#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/empty.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <vector>

#include "bspline_opt/bspline_optimizer.h"
#include "ego_planner/planner_manager.h"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "plan_env/grid_map.h"
#include "traj_utils/msg/bspline.hpp"
#include "traj_utils/msg/data_disp.hpp"
#include "traj_utils/msg/multi_bsplines.hpp"
#include "traj_utils/planning_visualization.h"

using std::vector;

namespace ego_planner {

class EGOReplanFSM {

private:
  /* ---------- flag ---------- */
  enum FSM_EXEC_STATE {
    INIT,
    WAIT_TARGET,
    GEN_NEW_TRAJ,
    REPLAN_TRAJ,
    EXEC_TRAJ,
    EMERGENCY_STOP,
    SEQUENTIAL_START
  };
  enum TARGET_TYPE {
    SINGLE_TARGET = 1, // single goal fed from /ego_planner/single_target
    WAYPOINT_TARGET =
        2, // preset waypoint list after /ego_planner/waypoints_target
    REFENCE_PATH = 3
  };

  /* planning utils */
  EGOPlannerManager::Ptr planner_manager_;
  PlanningVisualization::Ptr visualization_;
  traj_utils::msg::DataDisp data_disp_;
  traj_utils::msg::MultiBsplines multi_bspline_msgs_buf_;

  /* parameters */
  // Current mode: 1 SINGLE_TARGET, 2 WAYPOINT_TARGET (initialized from param,
  // updated on latest request)
  int target_mode_;
  double no_replan_thresh_, replan_thresh_;
  double planning_horizen_, planning_horizen_time_;
  double emergency_time_;
  bool flag_realworld_experiment_;
  bool enable_fail_safe_;

  /* planning data */
  bool waypoint_triggered_, have_target_, have_odom_, have_new_target_,
      have_recv_pre_agent_;
  FSM_EXEC_STATE exec_state_;
  int continously_called_times_{0};

  Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
  Eigen::Quaterniond odom_orient_;
  rclcpp::Time odom_timestamp_; // timestamp of the last odometry message

  Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_,
      start_yaw_;                                      // start state
  Eigen::Vector3d end_pt_, end_vel_;                   // goal state
  Eigen::Vector3d local_target_pt_, local_target_vel_; // local target state
  std::vector<Eigen::Vector3d> waypoints_;             // active waypoint list
  int next_wp_index_{0};

  bool flag_escape_emergency_;

  /* ROS utils */
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      single_target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<traj_utils::msg::MultiBsplines>::SharedPtr
      swarm_trajs_sub_;
  rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr
      broadcast_bspline_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr
      waypoint_trigger_sub_;

  // rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr replan_pub_;
  // rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr new_pub_;
  rclcpp::Publisher<traj_utils::msg::Bspline>::SharedPtr bspline_pub_;
  rclcpp::Publisher<traj_utils::msg::DataDisp>::SharedPtr data_disp_pub_;
  rclcpp::Publisher<traj_utils::msg::MultiBsplines>::SharedPtr swarm_trajs_pub_;
  rclcpp::Publisher<traj_utils::msg::Bspline>::SharedPtr broadcast_bspline_pub_;

  /* helper functions */
  bool
  callReboundReplan(bool flag_use_poly_init,
                    bool flag_randomPolyTraj); // front-end and back-end method
  bool
  callEmergencyStop(Eigen::Vector3d stop_pos); // front-end and back-end method
  bool planFromGlobalTraj(const int trial_times = 1);
  bool planFromCurrentTraj(const int trial_times = 1);

  /* return value: std::pair< Times of the same state be continuously called,
   * current continuously called state > */
  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
  void printFSMExecState();

  void planNextWaypoint(const Eigen::Vector3d next_wp);
  void getLocalTarget();

  /* ROS functions */
  void execFSMCallback();
  void checkCollisionCallback();
  void onSingleTarget(
      const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg);
  void onWaypointTarget(
      const std::shared_ptr<const geometry_msgs::msg::PoseArray> &msg);
  void
  odometryCallback(const std::shared_ptr<const nav_msgs::msg::Odometry> &msg);
  void swarmTrajsCallback(
      const std::shared_ptr<const traj_utils::msg::MultiBsplines> &msg);
  void BroadcastBsplineCallback(
      const std::shared_ptr<const traj_utils::msg::Bspline> &msg);

  bool checkCollision();
  void publishSwarmTrajs(bool startup_pub);

public:
  EGOReplanFSM(/* args */) {}
  ~EGOReplanFSM() {}

  void init(rclcpp::Node::SharedPtr &node);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace ego_planner

#endif
