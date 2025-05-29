#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/header.hpp"
 


CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
    costmap_.initializeCostmap(0.1, 100, 100, 0);
    string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/robot/chassis/lidar", 10,
        std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}



 
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Step 1: (Optional) Re-initialize costmap if needed
    initializeCostmap();

    // Step 2: Convert LaserScan to grid and mark obstacles
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        if (range < scan->range_max && range > scan->range_min) {
            int x_grid, y_grid;
            convertToGrid(range, angle, x_grid, y_grid);
            markObstacle(x_grid, y_grid);
        }
    }

    inflateObstacles();

    // Step 4: Publish costmap
    publishCostmap();
}






void CostmapNode::initializeCostmap() {
    costmap_.initializeCostmap(0.1, 100, 100, 0);
}


void CostmapNode::convertToGrid(double range, double angle, int& x_grid, int& y_grid) {

    double x = range * cos(angle);
    double y = range * sin(angle);

    double resolution = 0.1;
    int width = 100;
    int height = 100;

    x_grid = static_cast<int>(x / resolution + width / 2);
    y_grid = static_cast<int>(y / resolution + height / 2);
 
}


void CostmapNode::markObstacle(int x_grid, int y_grid) {
    int width = 100;
    int height = 100;
    if (x_grid >= 0 && x_grid < width && y_grid >= 0 && y_grid < height) {
        costmap_.setCell(x_grid, y_grid, 100);
    }
}


void CostmapNode::inflateObstacles() {
    double inflation_radius_m = 1.0; // meters
    int max_cost = 99; // less than 100 (occupied)
    double resolution = 0.1;
    int width = 100;
    int height = 100;
    int inflation_radius_cells = static_cast<int>(inflation_radius_m / resolution);

    // For each cell in the costmap
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (costmap_.getCell(x, y) == 100) {
                // Inflate around this obstacle
                for (int dy = -inflation_radius_cells; dy <= inflation_radius_cells; ++dy) {
                    for (int dx = -inflation_radius_cells; dx <= inflation_radius_cells; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            double distance = std::sqrt(dx * dx + dy * dy) * resolution;
                            if (distance <= inflation_radius_m && distance > 0.0) {
                                int cost = static_cast<int>(max_cost * (1.0 - distance / inflation_radius_m));
                                if (cost > costmap_.getCell(nx, ny)) {
                                    costmap_.setCell(nx, ny, cost);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void CostmapNode::publishCostmap() {
    nav_msgs::msg::OccupancyGrid msg;
    // Header
    msg.header.stamp = this->now();
    msg.header.frame_id = "map"; // or your costmap frame

    // Info
    msg.info.resolution = 0.1; // meters per cell
    msg.info.width = 100;
    msg.info.height = 100;
    msg.info.origin.position.x = - (msg.info.width * msg.info.resolution) / 2.0;
    msg.info.origin.position.y = - (msg.info.height * msg.info.resolution) / 2.0;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.w = 1.0;

    // Data (flatten 2D array to 1D)
    msg.data.resize(msg.info.width * msg.info.height);
    for (unsigned int y = 0; y < msg.info.height; ++y) {
        for (unsigned int x = 0; x < msg.info.width; ++x) {
            int value = costmap_.getCell(x, y);
            // OccupancyGrid expects values: 0 (free), 100 (occupied), -1 (unknown)
            msg.data[y * msg.info.width + x] = value;
        }
    }
    costmap_pub_->publish(msg);
}
 
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}