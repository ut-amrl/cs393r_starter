
#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <vector>

#include "glog/logging.h"
#include "gflags/gflags.h"
//#include "eigen3/Eigen/Dense"
//#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "sensor_msgs/LaserScan.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"

#include "cuda_ranges_to_cloud_lib.cuh"


DEFINE_string(laser_topic, "scan", "Name of ROS topic for LIDAR data");
DECLARE_string(helpon);
DECLARE_int32(v);

bool run_ = true;

void SignalHandler(int) {
    if (!run_) {
        printf("Force Exit.\n");
        exit(0);
    }
    printf("Exiting.\n");
    run_ = false;
}

void LaserCallbackMain(const sensor_msgs::LaserScan &msg) {
    if (FLAGS_v > 0) {
        printf("Laser t=%f, dt=%f\n",
               msg.header.stamp.toSec(),
               GetWallTime() - msg.header.stamp.toSec());
    }
    std::vector<laser_processing::Vector2f> returnedPointCloud = laser_processing::LaserCallback(msg);
    LOG(INFO) << "Constructed point cloud of size " << returnedPointCloud.size();
}

int main(int argc, char **argv) {
    google::ParseCommandLineFlags(&argc, &argv, false);
    signal(SIGINT, SignalHandler);
    // Initialize ROS.
    ros::init(argc, argv, "cuda_ranges_to_cloud", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;

    ros::Subscriber laser_sub =
            n.subscribe(FLAGS_laser_topic, 1, &LaserCallbackMain    );

    RateLoop loop(20.0);
    while (run_ && ros::ok()) {
        ros::spinOnce();
        loop.Sleep();
    }
    return 0;
}
