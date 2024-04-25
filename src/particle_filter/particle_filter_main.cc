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
\file    particle-filter-main.cc
\brief   Main entry point for particle filter based
         mobile robot localization
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

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/msg/localization2_d_msg.hpp"
#include "amrl_msgs/msg/visualization_msg.hpp"
#include "gflags/gflags.h"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "ros2_helpers.h"
#include "config_reader/config_reader.h"
#include "shared/math/math_util.h"
#include "shared/math/line2d.h"
#include "shared/util/timer.h"

#include "particle_filter.h"
#include "visualization/visualization.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

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
DEFINE_string(init_topic,
              "/set_pose",
              "Name of ROS topic for initialization");

DECLARE_int32(v);

// Create config reader entries
CONFIG_STRING(map_name_, "map");
CONFIG_FLOAT(init_x_, "init_x");
CONFIG_FLOAT(init_y_, "init_y");
CONFIG_FLOAT(init_r_, "init_r");
config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

bool run_ = true;
particle_filter::ParticleFilter particle_filter_;
rclcpp::Publisher<amrl_msgs::msg::VisualizationMsg>::SharedPtr visualization_publisher_;
rclcpp::Publisher<amrl_msgs::msg::Localization2DMsg>::SharedPtr localization_publisher_;
rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_publisher_;
std::shared_ptr<rclcpp::Node> node_;
VisualizationMsg vis_msg_;
amrl_msgs::msg::Localization2DMsg localization_msg_;
sensor_msgs::msg::LaserScan last_laser_msg_;

vector<Vector2f> trajectory_points_;
string current_map_;

void InitializeMsgs() {
  std_msgs::msg::Header header;
  header.frame_id = "map";
  localization_msg_.header = header;
  vis_msg_ = visualization::NewVisualizationMessage("map", "particle_filter");
}

void PublishParticles() {
  vector<particle_filter::Particle> particles;
  particle_filter_.GetParticles(&particles);
  for (const particle_filter::Particle &p : particles) {
    DrawParticle(p.loc, p.angle, vis_msg_);
  }
}

void PublishPredictedScan() {
  const uint32_t kColor = 0xd67d00;
  Vector2f robot_loc(0, 0);
  float robot_angle(0);
  particle_filter_.GetLocation(&robot_loc, &robot_angle);
  vector<Vector2f> predicted_scan;
  particle_filter_.GetPredictedPointCloud(
      robot_loc,
      robot_angle,
      last_laser_msg_.ranges.size(),
      last_laser_msg_.range_min,
      last_laser_msg_.range_max,
      last_laser_msg_.angle_min,
      last_laser_msg_.angle_max,
      &predicted_scan);
  for (const Vector2f &p : predicted_scan) {
    DrawPoint(p, kColor, vis_msg_);
  }
}

void PublishTrajectory() {
  const uint32_t kColor = 0xadadad;
  Vector2f robot_loc(0, 0);
  float robot_angle(0);
  particle_filter_.GetLocation(&robot_loc, &robot_angle);
  static Vector2f last_loc_(0, 0);
  if (!trajectory_points_.empty() &&
      (last_loc_ - robot_loc).squaredNorm() > Sq(1.5)) {
    trajectory_points_.clear();
  }
  if (trajectory_points_.empty() ||
      (robot_loc - last_loc_).squaredNorm() > 0.25) {
    trajectory_points_.push_back(robot_loc);
    last_loc_ = robot_loc;
  }
  for (size_t i = 0; i + 1 < trajectory_points_.size(); ++i) {
    DrawLine(trajectory_points_[i],
             trajectory_points_[i + 1],
             kColor,
             vis_msg_);
  }
}

void PublishVisualization() {
  static double t_last = 0;
  if (GetMonotonicTime() - t_last < 0.05) {
    // Rate-limit visualization.
    return;
  }
  t_last = GetMonotonicTime();
  vis_msg_.header.stamp = node_->get_clock()->now();
  ClearVisualizationMsg(vis_msg_);

  PublishParticles();
  PublishPredictedScan();
  PublishTrajectory();
  visualization_publisher_->publish(vis_msg_);
}

void LaserCallback(const sensor_msgs::msg::LaserScan &msg) {
  if (FLAGS_v > 0) {
    printf("Laser t=%f\n", ros_helpers::rosHeaderStampToSeconds(msg.header));
  }
  last_laser_msg_ = msg;
  particle_filter_.ObserveLaser(
      msg.ranges,
      msg.range_min,
      msg.range_max,
      msg.angle_min,
      msg.angle_max);
  PublishVisualization();
}

void PublishLocation() {
  Vector2f robot_loc(0, 0);
  float robot_angle(0);
  particle_filter_.GetLocation(&robot_loc, &robot_angle);
  localization_msg_.header.stamp = node_->get_clock()->now();
  localization_msg_.map = current_map_;
  localization_msg_.pose.x = robot_loc.x();
  localization_msg_.pose.y = robot_loc.y();
  localization_msg_.pose.theta = robot_angle;
  localization_publisher_->publish(localization_msg_);
}

void OdometryCallback(const nav_msgs::msg::Odometry &msg) {
  if (FLAGS_v > 0) {
    printf("Odometry t=%f\n", ros_helpers::rosHeaderStampToSeconds(msg.header));
  }
  const Vector2f odom_loc(msg.pose.pose.position.x, msg.pose.pose.position.y);
  const float odom_angle =
      2.0 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  particle_filter_.Predict(odom_loc, odom_angle);
  PublishLocation();
  PublishVisualization();
}

string GetMapFileFromName(const string &map) {
  string maps_dir_ = ament_index_cpp::get_package_share_directory("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

void InitCallback(const amrl_msgs::msg::Localization2DMsg &msg) {
  const Vector2f init_loc(msg.pose.x, msg.pose.y);
  const float init_angle = msg.pose.theta;
  current_map_ = msg.map;
  const string map_file = GetMapFileFromName(current_map_);
  printf("Initialize: %s (%f,%f) %f\u00b0\n",
         current_map_.c_str(),
         init_loc.x(),
         init_loc.y(),
         RadToDeg(init_angle));
  particle_filter_.Initialize(map_file, init_loc, init_angle);
  trajectory_points_.clear();
}

void ProcessLive(std::shared_ptr<rclcpp::Node> &n) {
  rclcpp::Subscription<amrl_msgs::msg::Localization2DMsg>::SharedPtr initial_pose_sub = n->create_subscription<amrl_msgs::msg::Localization2DMsg>(
      FLAGS_init_topic.c_str(),
      1,
      InitCallback);
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub = n->create_subscription<sensor_msgs::msg::LaserScan>(
      FLAGS_laser_topic.c_str(),
      1,
      LaserCallback);
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub = n->create_subscription<nav_msgs::msg::Odometry>(
      FLAGS_odom_topic.c_str(),
      1,
      OdometryCallback);
  particle_filter_.Initialize(
      GetMapFileFromName(current_map_),
      Vector2f(CONFIG_init_x_, CONFIG_init_x_),
      DegToRad(CONFIG_init_r_));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(n);
  while (rclcpp::ok() && run_) {
    executor.spin_once();
    PublishVisualization();
    Sleep(0.01);
  }
}

void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  run_ = false;
}

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler);
  // Initialize ROS.
  rclcpp::init(argc, argv);
  node_ = std::make_shared<rclcpp::Node>("particle_filter");
  InitializeMsgs();
  current_map_ = CONFIG_map_name_;

  visualization_publisher_ =
      node_->create_publisher<VisualizationMsg>("visualization", 1);
  localization_publisher_ =
      node_->create_publisher<amrl_msgs::msg::Localization2DMsg>("localization", 1);
  laser_publisher_ =
      node_->create_publisher<sensor_msgs::msg::LaserScan>("scan", 1);

  ProcessLive(node_);

  return 0;
}
