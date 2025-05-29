#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    // Initialize the costmap grid
    void initializeCostmap(double resolution, int width, int height, int default_value = 0);

    // Add a method to set a cell value in the costmap
    void setCell(int x, int y, int value) {
        if (x >= 0 && x < width_ && y >= 0 && y < height_) {
            costmap_[y][x] = value;
        }
    }

    // Add a method to get a cell value in the costmap
    int getCell(int x, int y) const {
        if (x >= 0 && x < width_ && y >= 0 && y < height_) {
            return costmap_[y][x];
        }
        return 0; // or a special value for out-of-bounds
    }

  private:
    rclcpp::Logger logger_;
    double resolution_;
    int width_;
    int height_;
    std::vector<std::vector<int>> costmap_;
};

}  


#endif