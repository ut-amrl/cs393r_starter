
#include <cuda_runtime_api.h>
#include "cuda_ranges_to_cloud_lib.cuh"

#include <gflags/gflags.h>

namespace laser_processing
{

  __global__ void convertRangeToPoint(int num_points, double angle_min, double angle_inc, double range_min, double range_max, 
    float laser_loc_x, float laser_loc_y, float *ranges, float *points_x, float *points_y) {
    int index = blockIdx.x * blockDim.x + threadIdx.x;
    int stride = blockDim.x * gridDim.x;
    for (int i = index; i < num_points; i += stride) {
      const float a = angle_min + static_cast<float>(i) * angle_inc;
      
      const float r = ((ranges[i] > range_min && ranges[i] < range_max) ? ranges[i] : range_max);
      points_x[i] = r * cos(a) + laser_loc_x;
      points_y[i] = r * sin(a) + laser_loc_y;
    }
  }

  std::vector<Vector2f> LaserCallback(const sensor_msgs::LaserScan &msg) {

    // Location of the laser on the robot. Assumes the laser is forward-facing.
    const Vector2f kLaserLoc(0.2, 0);

    std::vector<Vector2f> point_cloud;

    int num_ranges = msg.ranges.size();
    int num_bytes = num_ranges * sizeof(float);
    float *points_x_device;
    float *points_y_device;
    float *ranges_device;

    float points_x_host[num_ranges];
    float points_y_host[num_ranges];
    float ranges_host[num_ranges];

    copy(msg.ranges.begin(),msg.ranges.end(),ranges_host);

    // Allocate Unified Memory – accessible from CPU or GPU
    cudaMalloc(&points_x_device, num_bytes);
    cudaMalloc(&points_y_device, num_bytes);
    cudaMalloc(&ranges_device, num_bytes);

    cudaMemcpy(ranges_device, ranges_host, num_bytes, cudaMemcpyHostToDevice);

    int blockSize = 256;
    int numBlocks = (num_ranges + blockSize - 1) / blockSize;
    convertRangeToPoint<<<numBlocks, blockSize>>>(
      num_ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max, 
      kLaserLoc.x_, kLaserLoc.y_, ranges_device, points_x_device, points_y_device);

    cudaMemcpy(points_x_host, points_x_device, num_bytes, cudaMemcpyDeviceToHost);
    cudaMemcpy(points_y_host, points_y_device, num_bytes, cudaMemcpyDeviceToHost);

    // Wait for GPU to finish before accessing on host
    cudaDeviceSynchronize();

    // Free memory
    cudaFree(points_x_device);
    cudaFree(points_y_device);
    cudaFree(ranges_device);

    for (size_t point_idx = 0; point_idx < num_ranges; point_idx++) {
      point_cloud.emplace_back(Vector2f(points_x_host[point_idx], points_y_host[point_idx]));
    }

    return point_cloud;
  }

}