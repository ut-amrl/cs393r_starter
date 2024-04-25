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
\file    slam-main.cc
\brief   Main entry point for slam
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <termios.h>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/msg/localization2_d_msg.hpp"
#include "amrl_msgs/msg/visualization_msg.hpp"
#include "gflags/gflags.h"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "ros2_helpers.h"
#include "config_reader/config_reader.h"
#include "shared/math/math_util.h"
#include "shared/math/line2d.h"
#include "shared/util/timer.h"

#include "slam.h"
#include "vector_map/vector_map.h"
#include "visualization/visualization.h"

using amrl_msgs::msg::VisualizationMsg;
using geometry::line2f;
using geometry::Line;
using math_util::DegToRad;
using math_util::RadToDeg;
using std::string;
using std::vector;
using Eigen::Vector2f;
using visualization::ClearVisualizationMsg;
using visualization::DrawArc;
using visualization::DrawPoint;
using visualization::DrawLine;
using visualization::DrawParticle;

// Create command line arguements
DEFINE_string(laser_topic, "/scan", "Name of ROS topic for LIDAR data");
DEFINE_string(odom_topic, "/odom", "Name of ROS topic for odometry data");

DECLARE_int32(v);

bool run_ = true;
slam::SLAM slam_;
rclcpp::Publisher<amrl_msgs::msg::VisualizationMsg>::SharedPtr visualization_publisher_;
rclcpp::Publisher<amrl_msgs::msg::Localization2DMsg>::SharedPtr localization_publisher_;
std::shared_ptr<rclcpp::Node> node_;
VisualizationMsg vis_msg_;
sensor_msgs::msg::LaserScan last_laser_msg_;

void InitializeMsgs() {
  std_msgs::msg::Header header;
  header.frame_id = "map";
  vis_msg_ = visualization::NewVisualizationMessage("map", "slam");
}

void PublishMap() {
  static double t_last = 0;
  if (GetMonotonicTime() - t_last < 0.5) {
    // Rate-limit visualization.
    return;
  }
  t_last = GetMonotonicTime();
  vis_msg_.header.stamp = node_->get_clock()->now();
  ClearVisualizationMsg(vis_msg_);

  const vector<Vector2f> map = slam_.GetMap();
  printf("Map: %lu points\n", map.size());
  for (const Vector2f& p : map) {
    visualization::DrawPoint(p, 0xC0C0C0, vis_msg_);
  }
  visualization_publisher_->publish(vis_msg_);
}

void PublishPose() {
  Vector2f robot_loc(0, 0);
  float robot_angle(0);
  slam_.GetPose(&robot_loc, &robot_angle);
  amrl_msgs::msg::Localization2DMsg localization_msg;
  localization_msg.pose.x = robot_loc.x();
  localization_msg.pose.y = robot_loc.y();
  localization_msg.pose.theta = robot_angle;
  localization_publisher_->publish(localization_msg);
}

void LaserCallback(const sensor_msgs::msg::LaserScan& msg) {
  if (FLAGS_v > 0) {
    printf("Laser t=%f\n", ros_helpers::rosHeaderStampToSeconds(msg.header));
  }
  last_laser_msg_ = msg;
  slam_.ObserveLaser(
      msg.ranges,
      msg.range_min,
      msg.range_max,
      msg.angle_min,
      msg.angle_max);
  PublishMap();
  PublishPose();
}

void OdometryCallback(const nav_msgs::msg::Odometry& msg) {
  if (FLAGS_v > 0) {
    printf("Odometry t=%f\n", ros_helpers::rosHeaderStampToSeconds(msg.header));
  }
  const Vector2f odom_loc(msg.pose.pose.position.x, msg.pose.pose.position.y);
  const float odom_angle =
      2.0 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  slam_.ObserveOdometry(odom_loc, odom_angle);
}


int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  // Initialize ROS.
  rclcpp::init(argc, argv);
  node_ = std::make_shared<rclcpp::Node>("slam");
  InitializeMsgs();

  visualization_publisher_ =
      node_->create_publisher<VisualizationMsg>("visualization", 1);
  localization_publisher_ =
      node_->create_publisher<amrl_msgs::msg::Localization2DMsg>("localization", 1);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      FLAGS_laser_topic.c_str(),
      1,
      LaserCallback);
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub = node_->create_subscription<nav_msgs::msg::Odometry>(
      FLAGS_odom_topic.c_str(),
      1,
      OdometryCallback);
  rclcpp::spin(node_);

  return 0;
}
