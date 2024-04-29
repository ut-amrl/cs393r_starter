//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/msg/ackermann_curvature_drive_msg.hpp"
#include "amrl_msgs/msg/pose2_df.hpp"
#include "amrl_msgs/msg/visualization_msg.hpp"
#include "glog/logging.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "ros2_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"

#include <geometry_msgs/msg/twist.hpp>

#include <irobot_create_msgs/action/drive_distance.hpp>
#include <irobot_create_msgs/action/drive_arc.hpp>
#include <irobot_create_msgs/action/rotate_angle.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

using Eigen::Vector2f;
using amrl_msgs::msg::AckermannCurvatureDriveMsg;
using amrl_msgs::msg::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

namespace {
rclcpp::Publisher<VisualizationMsg>::SharedPtr viz_pub_;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

string GetMapFileFromName(const string &map) {
  string maps_dir_ = ament_index_cpp::get_package_share_directory("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

Navigation::Navigation(const string &map_name, const std::shared_ptr<rclcpp::Node> &node) :
    node_(node),
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  map_.Load(GetMapFileFromName(map_name));
  LOG(INFO) << "Loaded map file: " << GetMapFileFromName(map_name);
  viz_pub_ = node->create_publisher<VisualizationMsg>("visualization", 1);
  twist_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("/ut/cmd_vel", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");

  drive_distance_client_ = rclcpp_action::create_client<irobot_create_msgs::action::DriveDistance>(
        node_,
        "/ut/drive_distance");
  drive_arc_client_ = rclcpp_action::create_client<irobot_create_msgs::action::DriveArc>(
        node_,
        "/ut/drive_arc");
  rotate_angle_client_ = rclcpp_action::create_client<irobot_create_msgs::action::RotateAngle>(
        node_,
        "/ut/rotate_angle");
}

void Navigation::SetNavGoal(const Vector2f &loc, float angle) {
}

bool Navigation::StartDriveDistanceAction(const double &distance, const double &max_translation_speed,
                                          const std::chrono::duration<int64_t, std::milli> &wait_for_server_timeout) {
  if (anyGoalsInProgress()) {
      LOG(ERROR)
          << "Tried to initiate new action when one is already in progress. Make sure that one is not executing and reset variables appropriately";
      return false;
  }
  using namespace std::placeholders;
  if (!drive_distance_client_->wait_for_action_server(wait_for_server_timeout)) {
      LOG(INFO) << "Timed out waiting for server";
      return false;
  }

  if (!drive_distance_pending_feedback_msgs_.empty()) {
      LOG(WARNING) << "Have unprocessed feedback for drive distance. Clearing as we issue new goal";
      drive_distance_pending_feedback_msgs_.clear();
  }

  if (distance < 0) {
      LOG(WARNING) << "Turtlebot will only back up a couple cm, because it lacks cliff sensors on the back";
  }

  auto drive_distance_goal = irobot_create_msgs::action::DriveDistance::Goal();
  drive_distance_goal.distance = distance;
  drive_distance_goal.max_translation_speed = max_translation_speed;

  auto send_goal_options = rclcpp_action::Client<irobot_create_msgs::action::DriveDistance>::SendGoalOptions();
  send_goal_options.goal_response_callback =
        std::bind(&Navigation::DriveDistanceGoalResponseCallback, this, _1);
  send_goal_options.feedback_callback =
        std::bind(&Navigation::DriveDistanceFeedbackCallback, this, _1, _2);
  send_goal_options.result_callback =
        std::bind(&Navigation::DriveDistanceResultCallback, this, _1);
  drive_distance_goal_status_ = PENDING_ACCEPTANCE;
  drive_distance_goal_issued_time_ = node_->get_clock()->now();
  drive_distance_client_->async_send_goal(drive_distance_goal, send_goal_options);
  return true;
}

bool Navigation::StartDriveArcAction(const int &translate_direction, const double &arc_radius, const double &arc_angle, const double &max_translation_speed,
                         const std::chrono::duration<int64_t, std::milli> &wait_for_server_timeout) {
  if (anyGoalsInProgress()) {
      LOG(ERROR)
          << "Tried to initiate new action when one is already in progress. Make sure that one is not executing and reset variables appropriately";
      return false;
  }
  using namespace std::placeholders;
  if (!drive_arc_client_->wait_for_action_server(wait_for_server_timeout)) {
      LOG(INFO) << "Timed out waiting for server";
      return false;
  }

  if (!drive_arc_pending_feedback_msgs_.empty()) {
      LOG(WARNING) << "Have unprocessed feedback for drive arc. Clearing as we issue new goal";
      drive_arc_pending_feedback_msgs_.clear();
  }

  if (arc_angle < 0) {
      LOG(WARNING) << "Turtlebot will only back up a couple cm, because it lacks cliff sensors on the back";
  }

  auto drive_arc_goal = irobot_create_msgs::action::DriveArc::Goal();
  drive_arc_goal.translate_direction = translate_direction;
  drive_arc_goal.angle = arc_angle;
  drive_arc_goal.radius = arc_radius;
  drive_arc_goal.max_translation_speed = max_translation_speed;

  auto send_goal_options = rclcpp_action::Client<irobot_create_msgs::action::DriveArc>::SendGoalOptions();
  send_goal_options.goal_response_callback =
        std::bind(&Navigation::DriveArcGoalResponseCallback, this, _1);
  send_goal_options.feedback_callback =
        std::bind(&Navigation::DriveArcFeedbackCallback, this, _1, _2);
  send_goal_options.result_callback =
        std::bind(&Navigation::DriveArcResultCallback, this, _1);
  drive_arc_goal_status_ = PENDING_ACCEPTANCE;
  drive_arc_goal_issued_time_ = node_->get_clock()->now();
  drive_arc_client_->async_send_goal(drive_arc_goal, send_goal_options);
  return true;
}

bool Navigation::StartRotateAngleAction(const double &angle_rad, const double &max_rotation_speed,
                                        const std::chrono::duration<int64_t, std::milli> &wait_for_server_timeout) {
  if (anyGoalsInProgress()) {
      LOG(ERROR)
          << "Tried to initiate new action when one is already in progress. Make sure that one is not executing and reset variables appropriately";
      return false;
  }
  using namespace std::placeholders;
  if (!rotate_angle_client_->wait_for_action_server(wait_for_server_timeout)) {
      LOG(INFO) << "Timed out waiting for server";
      return false;
  }

  if (!rotate_angle_pending_feedback_msgs_.empty()) {
      LOG(WARNING) << "Have unprocessed feedback for rotate_angle. Clearing as we issue new goal";
      rotate_angle_pending_feedback_msgs_.clear();
  }

  auto rotate_angle_goal = irobot_create_msgs::action::RotateAngle::Goal();
  rotate_angle_goal.angle = angle_rad;
  rotate_angle_goal.max_rotation_speed = max_rotation_speed;

  auto send_goal_options = rclcpp_action::Client<irobot_create_msgs::action::RotateAngle>::SendGoalOptions();
  send_goal_options.goal_response_callback =
        std::bind(&Navigation::RotateAngleGoalResponseCallback, this, _1);
  send_goal_options.feedback_callback =
        std::bind(&Navigation::RotateAngleFeedbackCallback, this, _1, _2);

  send_goal_options.result_callback =
        std::bind(&Navigation::RotateAngleResultCallback, this, _1);
  rotate_angle_goal_status_ = PENDING_ACCEPTANCE;
  rotate_angle_goal_issued_time_ = node_->get_clock()->now();
  rotate_angle_client_->async_send_goal(rotate_angle_goal, send_goal_options);
  return true;
}

void Navigation::UpdateLocation(const Eigen::Vector2f &loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f &loc,
                                float angle,
                                const Vector2f &vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;
}

void Navigation::ObservePointCloud(const vector<Vector2f> &cloud,
                                   double time) {
  point_cloud_ = cloud;
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.

  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // The control iteration goes here.
  // Feel free to make helper functions to structure the control appropriately.

  // The latest observed point cloud is accessible via "point_cloud_"

  // Eventually, you will have to set the control values to issue drive commands:
  // You can call StartDriveDistanceAction, StartDriveArcAction, or StartRotateAngleAction
  // and then wait for the status

  // You should also check that it's not stuck due to ros communication errors by keeping track of how much time
  // has elapsed since your request

  // Make sure to reset the status to NONE once you've processed the completion of an action request

  geometry_msgs::msg::Twist twist;
  twist.linear.x = 1;
  twist_pub_->publish(twist);

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = node_->get_clock()->now();
  global_viz_msg_.header.stamp = node_->get_clock()->now();
  // Publish messages.
  viz_pub_->publish(local_viz_msg_);
  viz_pub_->publish(global_viz_msg_);
}

}  // namespace navigation
