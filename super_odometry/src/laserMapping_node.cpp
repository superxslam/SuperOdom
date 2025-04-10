//
// Created by shibo zhao on 2020-09-27.
//
#include "rclcpp/rclcpp.hpp"
#include "super_odometry/LaserMapping/laserMapping.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.arguments({"laser_mapping_node"});

    std::shared_ptr<super_odometry::laserMapping> laserMapping =
      std::make_shared<super_odometry::laserMapping>(options);

    laserMapping->initInterface();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(laserMapping);
    executor.spin();
    // rclcpp::spin(laserMapping->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}

