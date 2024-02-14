

#include "sensor_msgs/LaserScan.h"

//#include "eigen3/Eigen/Geometry"
#include <vector>

namespace laser_processing {
    struct Vector2f {
        float x_;
        float y_;

        Vector2f() = default;

        Vector2f(const float &x, const float &y) : x_(x), y_(y) {}
    };

std::vector<Vector2f> LaserCallback(const sensor_msgs::LaserScan& msg);
}