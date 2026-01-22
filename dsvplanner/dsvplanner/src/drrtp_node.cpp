/*
drrtp_node.cpp
node to launch drrtplanner

Created by Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
05/25/2020
*/

#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <dsvplanner/drrtp.h>
#include <rclcpp/executors.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 10);

    rclcpp::Node::SharedPtr node_handle = rclcpp::Node::make_shared("dsvPlanner");

    dsvplanner_ns::drrtPlanner planner(node_handle);

    executor.add_node(node_handle);
    executor.spin();
    return 0;
}
