/**************************************************************************
drrtp.h
Header of the drrtp (dynamic_rrtplanner) class

Hongbiao Zhu(hongbiaz@andrew.cmu.edu)
05/25/2020

**************************************************************************/

#ifndef DRRTP_H_
#define DRRTP_H_

#include "dsvplanner/drrt.h"
#include "dsvplanner/drrt_base.h"
#include "dsvplanner/srv/dsvplanner.hpp"
#include "dsvplanner/srv/clean_frontier.hpp"
#include "dsvplanner/dual_state_frontier.h"
#include "dsvplanner/dual_state_graph.h"

// #include "rmw/qos_profiles.h"
// #include "octomap_world/octomap_manager.h"

#include <visualization_msgs/msg/marker.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace dsvplanner_ns
{
    class drrtPlanner
    {
        public:
        rclcpp::Node::SharedPtr nh_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
        rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr boundarySub_;

        rclcpp::Service<dsvplanner::srv::Dsvplanner>::SharedPtr plannerService_;
        rclcpp::Service<dsvplanner::srv::CleanFrontier>::SharedPtr cleanFrontierService_;

        Params params_;
        octomanager* manager_;
        DualStateGraph* dual_state_graph_;
        DualStateFrontier* dual_state_frontier_;
        Drrt* drrt_;

        nav_msgs::msg::OccupancyGrid::SharedPtr occupy_map_2d;

        bool init();
        bool setParams();
        // bool setPublisherPointer();
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr pose);
        void boundaryCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr boundary);
        void plannerServiceCallback(const dsvplanner::srv::Dsvplanner::Request::SharedPtr req,
            dsvplanner::srv::Dsvplanner::Response::SharedPtr res);
        void cleanFrontierServiceCallback(const dsvplanner::srv::CleanFrontier::Request::SharedPtr req,
            dsvplanner::srv::CleanFrontier::Response::SharedPtr res);
        void cleanLastSelectedGlobalFrontier();

        drrtPlanner(rclcpp::Node::SharedPtr& node_handle);
        ~drrtPlanner();
        // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav2_goal_Pub_;

        private:
        std::string odomSubTopic;
        std::string boundarySubTopic;
        std::string terrainCloudSubTopic;
        std::string newTreePathPubTopic;
        std::string remainingTreePathPubTopic;
        std::string boundaryPubTopic;
        std::string globalSelectedFrontierPubTopic;
        std::string localSelectedFrontierPubTopic;
        std::string plantimePubTopic;
        std::string nextGoalPubTopic;
        std::string randomSampledPointsPubTopic;
        std::string shutDownTopic;
        std::string plannerServiceName;
        std::string cleanFrontierServiceName;

        std::chrono::steady_clock::time_point plan_start_;
        std::chrono::steady_clock::time_point RRT_generate_over_;
        std::chrono::steady_clock::time_point gain_computation_over_;
        std::chrono::steady_clock::time_point plan_over_;
        std::chrono::steady_clock::duration time_span;
    };
}

#endif // DRRTP_H
