//
// Created by shibo zhao on 2020-09-27.
//
#include "rclcpp/rclcpp.hpp"
#include "super_odometry/ImuPreintegration/imuPreintegration.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    rclcpp::NodeOptions options;
    options.arguments({"imu_preintegration_node"});

    std::shared_ptr<super_odometry::imuPreintegration> imuPreintegration =
        std::make_shared<super_odometry::imuPreintegration>(options);
    
    imuPreintegration->imuBuf.allocate(1000);
    imuPreintegration->lidarOdomBuf.allocate(100);
    imuPreintegration->initInterface();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(imuPreintegration);
    executor.spin();
    // rclcpp::spin(imuPreintegration->get_node_base_interface());
    rclcpp::shutdown();
    
    return 0;
}
