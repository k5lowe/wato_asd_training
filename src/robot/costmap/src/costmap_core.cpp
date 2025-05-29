#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::initializeCostmap(double resolution, int width, int height, int default_value) {
    resolution_ = resolution;
    width_ = width;
    height_ = height;
    costmap_.resize(height_, std::vector<int>(width_, default_value));
    RCLCPP_INFO(logger_, "Initialized costmap: resolution=%.2f, width=%d, height=%d, default_value=%d", resolution_, width_, height_, default_value);
}

}