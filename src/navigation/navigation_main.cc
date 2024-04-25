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
\file    navigation_main.cc
\brief   Main entry point for reference Navigation implementation
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <vector>

#include "glog/logging.h"
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/msg/localization2_d_msg.hpp"
#include "gflags/gflags.h"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "ros2_helpers.h"

#include "std_msgs/msg/string.hpp"

#include "navigation.h"

using amrl_msgs::msg::Localization2DMsg;
using math_util::DegToRad;
using math_util::RadToDeg;
using navigation::Navigation;
using ros_helpers::Eigen3DToRosPoint;
using ros_helpers::Eigen2DToRosPoint;
using ros_helpers::RosPoint;
using ros_helpers::SetRosVector;
using std::string;
using std::vector;
using Eigen::Vector2f;

// Create command line arguments
DEFINE_string(laser_topic, "scan", "Name of ROS topic for LIDAR data");
DEFINE_string(odom_topic, "odom", "Name of ROS topic for odometry data");
DEFINE_string(loc_topic, "localization", "Name of ROS topic for localization");
DEFINE_string(init_topic,
              "initialpose",
              "Name of ROS topic for initialization");
DEFINE_string(map, "GDC1", "Name of vector map file");

bool run_ = true;
sensor_msgs::msg::LaserScan last_laser_msg_;
Navigation *navigation_ = nullptr;

void LaserCallback(const sensor_msgs::msg::LaserScan &msg) {
    if (FLAGS_v > 0) {
        printf("Laser t=%f, dt=%f\n",
               ros_helpers::rosHeaderStampToSeconds(msg.header),
               GetWallTime() - ros_helpers::rosHeaderStampToSeconds(msg.header));
    }
    // Location of the laser on the robot. Assumes the laser is forward-facing.
    const Vector2f kLaserLoc(0.2, 0);

    static vector<Vector2f> point_cloud_;
    // TODO Convert the LaserScan to a point cloud
    // The LaserScan parameters are accessible as follows:
    // msg.angle_increment // Angular increment between subsequent rays
    // msg.angle_max // Angle of the first ray
    // msg.angle_min // Angle of the last ray
    // msg.range_max // Maximum observable range
    // msg.range_min // Minimum observable range
    // msg.ranges[i] // The range of the i'th ray
    navigation_->ObservePointCloud(point_cloud_, ros_helpers::rosHeaderStampToSeconds(msg.header));
    last_laser_msg_ = msg;
}

void OdometryCallback(const nav_msgs::msg::Odometry &msg) {
    if (FLAGS_v > 0) {
        printf("Odometry t=%f\n", ros_helpers::rosHeaderStampToSeconds(msg.header));
    }
    navigation_->UpdateOdometry(
            Vector2f(msg.pose.pose.position.x, msg.pose.pose.position.y),
            2.0 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
            Vector2f(msg.twist.twist.linear.x, msg.twist.twist.linear.y),
            msg.twist.twist.angular.z);
}

void GoToCallback(const geometry_msgs::msg::PoseStamped &msg) {
    const Vector2f loc(msg.pose.position.x, msg.pose.position.y);
    const float angle =
            2.0 * atan2(msg.pose.orientation.z, msg.pose.orientation.w);
    printf("Goal: (%f,%f) %f\u00b0\n", loc.x(), loc.y(), angle);
    navigation_->SetNavGoal(loc, angle);
}

void SignalHandler(int) {
    if (!run_) {
        printf("Force Exit.\n");
        exit(0);
    }
    printf("Exiting.\n");
    run_ = false;
}

void LocalizationCallback(const amrl_msgs::msg::Localization2DMsg msg) {
    if (FLAGS_v > 0) {
        printf("Localization t=%f\n", GetWallTime());
    }
    navigation_->UpdateLocation(Vector2f(msg.pose.x, msg.pose.y), msg.pose.theta);
}

void StringCallback(const std_msgs::msg::String &msg) {
    std::cout << msg.data << "\n";
}

int main(int argc, char **argv) {
    google::ParseCommandLineFlags(&argc, &argv, false);
    signal(SIGINT, SignalHandler);
    // Initialize ROS.
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("ut_navigation");
    navigation_ = new Navigation(FLAGS_map, node);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_sub = node->create_subscription<std_msgs::msg::String>(
            "string_topic", 1, &StringCallback);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr velocity_sub =
            node->create_subscription<nav_msgs::msg::Odometry>(FLAGS_odom_topic, 1, &OdometryCallback);
    rclcpp::Subscription<amrl_msgs::msg::Localization2DMsg>::SharedPtr localization_sub =
            node->create_subscription<amrl_msgs::msg::Localization2DMsg>(FLAGS_loc_topic, 1, &LocalizationCallback);
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub =
            node->create_subscription<sensor_msgs::msg::LaserScan>(FLAGS_laser_topic, 1, &LaserCallback);
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goto_sub =
            node->create_subscription<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1, &GoToCallback);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    RateLoop loop(20.0);
    while (run_ && rclcpp::ok()) {
        executor.spin_once();
        navigation_->Run();
        loop.Sleep();
    }
    delete navigation_;
    return 0;
}
