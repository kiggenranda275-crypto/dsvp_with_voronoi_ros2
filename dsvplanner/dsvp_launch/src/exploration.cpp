
#include <chrono>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>

#include "dsvplanner/srv/clean_frontier.hpp"
#include "dsvplanner/srv/dsvplanner.hpp"
#include "graph_planner/msg/graph_planner_command.hpp"
#include "graph_planner/msg/graph_planner_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono;
#define cursup "\033[A"
#define cursclean "\033[2K"
#define curshome "\033[0;0H"
#define UNIT 1000000000

//定义航点 目标点 原点 地图规划器命令消息 有效规划时间 总规划时间
geometry_msgs::msg::Point wayPoint;
geometry_msgs::msg::Point wayPoint_pre;
geometry_msgs::msg::Point goal_point;
geometry_msgs::msg::Point home_point;
graph_planner::msg::GraphPlannerCommand graph_planner_command;
std_msgs::msg::Float32 effective_time;
std_msgs::msg::Float32 total_time;

bool play_bag = false;     // control whether use graph planner to follow path
bool begin_signal = false; // trigger the planner 触发规划器信号
bool gp_in_progress = false; // 地图规划器是否正在执行
bool wp_state = false;
bool return_home = false;
bool odom_is_ready = false;
bool call_dsvp_successfully = true; //是否成功调用dsvp规划器
bool initialized = false;
double current_odom_x = 0;
double current_odom_y = 0;
double current_odom_z = 0;
double previous_odom_x = 0;
double previous_odom_y = 0;
double previous_odom_z = 0;
double dtime = 0.0;
double init_x = 2; //初始位置的坐标和初始时间
double init_y = 0;
double init_z = 0;
double init_time = 2;
double return_home_threshold = 1.5;
double robot_moving_threshold = 6; //判断机器人是否移动的阈值
std::string map_frame = "map";
std::string waypoint_topic = "/way_point";
std::string gp_command_topic = "/graph_planner_command";
std::string effective_plan_time_topic = "/runtime";
std::string total_plan_time_topic = "/totaltime";
std::string gp_status_topic = "/graph_planner_status";
std::string odom_topic = "/state_estimation";
std::string begin_signal_topic = "/start_exploring";
std::string stop_signal_topic = "/stop_exploring";

// 到map坐标系的变换
tf2::Transform transformToMap;

steady_clock::time_point plan_start;
steady_clock::time_point plan_over;
steady_clock::duration time_span;

rclcpp::Node::SharedPtr nh = nullptr;
rclcpp::Node::SharedPtr nh2 = nullptr;
rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_pub;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub;

rclcpp::Publisher<graph_planner::msg::GraphPlannerCommand>::SharedPtr gp_command_pub;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr effective_plan_time_pub;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr total_plan_time_pub;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_signal_pub;

rclcpp::Subscription<graph_planner::msg::GraphPlannerStatus>::SharedPtr gp_status_sub;
rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_sub;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr begin_signal_sub;

// 边界清除服务客户端  DSV规划器服务客户端
rclcpp::Client<dsvplanner::srv::CleanFrontier>::SharedPtr frontier_cleaner_client;
rclcpp::Client<dsvplanner::srv::Dsvplanner>::SharedPtr drrt_planner_client;

void gp_status_callback(const graph_planner::msg::GraphPlannerStatus::SharedPtr msg)
{
    if (msg->status == graph_planner::msg::GraphPlannerStatus::STATUS_IN_PROGRESS)
        gp_in_progress = true;
    else
    {
        gp_in_progress = false;
    }
}

void waypoint_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    wayPoint = msg->point;
    wp_state = true;
}

void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_odom_x = msg->pose.pose.position.x;
    current_odom_y = msg->pose.pose.position.y;
    current_odom_z = msg->pose.pose.position.z;

    // 更新到地图坐标系的变换
    transformToMap.setOrigin(
        tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    transformToMap.setRotation(tf2::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
    odom_is_ready = true;
}

void begin_signal_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    begin_signal = msg->data;
}

// 检查机器人位置是否改变
bool robotPositionChange()
{
    // 计算当前位置和上一时刻位置的距离 通过前一时刻和现在时刻的里程计
    double dist = sqrt((current_odom_x - previous_odom_x) * (current_odom_x - previous_odom_x) +
        (current_odom_y - previous_odom_y) * (current_odom_y - previous_odom_y) +
        (current_odom_z - previous_odom_z) * (current_odom_z - previous_odom_z));
    // 如果距离小于阈值，认为机器人没有移动
    if (dist < robot_moving_threshold)
        return false;
    previous_odom_x = current_odom_x;
    previous_odom_y = current_odom_y;
    previous_odom_z = current_odom_z;
    return true;
}

// 主要执行函数，通过间隔一定时间循环调用
void execute()
{
    static int iteration = 0;
    if (initialized)
    {
        if (!return_home)
        {
            // if (iteration != 0)
            // {
            // for (int i = 0; i < 8; i++)
            // {
            //     printf(cursup);
            //     printf(cursclean);
            // }
            // }
            // 逻辑判断
            if (call_dsvp_successfully)
            {
                std::cout << "Planning iteration " << iteration << std::endl;
                auto planSrv = std::make_shared<dsvplanner::srv::Dsvplanner::Request>();
                auto cleanSrv = std::make_shared<dsvplanner::srv::CleanFrontier::Request>();
                planSrv->header.stamp = nh->now();

                call_dsvp_successfully = false;
                auto callback = [&](rclcpp::Client<dsvplanner::srv::Dsvplanner>::SharedFuture future)
                    {
                        auto result = future.get();

                        plan_over = steady_clock::now();
                        time_span = plan_over - plan_start;
                        effective_time.data = float(time_span.count()) * steady_clock::period::num / steady_clock::period::den;

                        if (result->goal.size() == 0)
                        { // usually the size should be 1 if planning successfully
                            effective_plan_time_pub->publish(effective_time);
                        }
                        else
                        {
                            if (result->mode == 2)
                            {
                                return_home = true;
                                goal_point = home_point;
                                std::cout << std::endl
                                    << "\033[1;32m Exploration completed, returning home \033[0m" << std::endl
                                    << std::endl;
                                effective_time.data = 0;
                                effective_plan_time_pub->publish(effective_time);
                            }
                            else
                            {
                                return_home = false;
                                goal_point = result->goal[0];
                                // time_span = plan_over - plan_start;
                                // effective_time.data = float(time_span.count()) * steady_clock::period::num / steady_clock::period::den;
                                effective_plan_time_pub->publish(effective_time);
                            }

                            total_time.data += effective_time.data;
                            total_plan_time_pub->publish(total_time);
                            // 不使用测试模拟模式
                            if (!play_bag)
                            {
                                graph_planner_command.command = graph_planner::msg::GraphPlannerCommand::COMMAND_GO_TO_LOCATION;
                                graph_planner_command.location = goal_point;
                                gp_command_pub->publish(graph_planner_command); // 地图规划器目标指令，前往目标点

                                geometry_msgs::msg::PoseStamped goal_pose;
                                goal_pose.header.stamp = nh->now(); //时间戳
                                goal_pose.header.frame_id = map_frame; //坐标系
                                goal_pose.pose.position.x = goal_point.x; //位置和姿态
                                goal_pose.pose.position.y = goal_point.y;
                                goal_pose.pose.position.z = 0.0;
                                goal_pose.pose.orientation.x = 0.0;
                                goal_pose.pose.orientation.y = 0.0;
                                goal_pose.pose.orientation.z = 0.0;
                                goal_pose.pose.orientation.w = 1.0;
                                goal_pose_pub->publish(goal_pose); // 发布目标点（目标位姿）

                                sleep(1);
                                int count = 200;
                                previous_odom_x = current_odom_x;
                                previous_odom_y = current_odom_y;
                                previous_odom_z = current_odom_z;
                                while (gp_in_progress)
                                {
                                    // 如果机器人没有移动，计数器减1，如果计数器小于等于0，清除对应的边界；20s内
                                    usleep(100000); // seconds, then give up the goal
                                    wayPoint_pre = wayPoint;
                                    bool robotMoving = robotPositionChange(); // 检查机器人是否移动
                                    if (robotMoving) //如果机器人移动了，重置计数器
                                    {
                                        count = 200;
                                    }
                                    else //否则计数器减1
                                    {
                                        count--;
                                    }
                                    if (count <= 0)
                                    { // when the goal point cannot be reached, clean
                                        // its correspoinding frontier if there is
                                        cleanSrv->header.stamp = nh->now(); // 设置边界清除服务请求的时间戳
                                        cleanSrv->header.frame_id = map_frame; // 设置边界清除服务请求的坐标系
                                        auto response = frontier_cleaner_client->async_send_request(cleanSrv); // 清除边界点

                                        usleep(1000);
                                        break;
                                    }
                                }
                                graph_planner_command.command = graph_planner::msg::GraphPlannerCommand::COMMAND_DISABLE;
                                gp_command_pub->publish(graph_planner_command);
                            }
                            else
                            { // 测试该规划算法时使用模拟模式
                                // 带有机器人将要使用的包文件
                                // 没有达到计划的目标。当处于模拟模式时，机器人将
                                // 保持每两秒重新计划一次
                                for (size_t i = 0; i < result->goal.size(); i++)
                                {
                                    // 设置地图规划器命令为前往目标点
                                    graph_planner_command.command = graph_planner::msg::GraphPlannerCommand::COMMAND_GO_TO_LOCATION;
                                    graph_planner_command.location = result->goal[i]; // 设置目标点
                                    gp_command_pub->publish(graph_planner_command); // 发布地图规划器命令
                                    while (1)
                                    {
                                    }
                                    break;
                                }
                            }
                            plan_start = steady_clock::now(); // 记录下一次规划开始时间
                        }
                        call_dsvp_successfully = true; // 设置调用DSVP规划器标志位为true
                        // Use result->responses[] and do something
                    };
                auto result_future = drrt_planner_client->async_send_request(planSrv, callback); // 异步发送DSV规划器服务请求
                iteration++; // 迭代次数加1
            }
        }
        // 返回原点位置逻辑
        else
        {
            // 认为返回原点
            if (fabs(current_odom_x - home_point.x) + fabs(current_odom_y - home_point.y) +
                fabs(current_odom_z - home_point.z) <=
                return_home_threshold)
            {
                printf(cursclean);
                std::cout << "\033[1;32mReturn home completed\033[0m" << std::endl;
                printf(cursup);
                std_msgs::msg::Bool stop_exploring;
                stop_exploring.data = true;
                stop_signal_pub->publish(stop_exploring);
            }
            else
            {
                while (!gp_in_progress)
                {
                    sleep(2);
                    // executor.spin_some();
                    graph_planner_command.command = graph_planner::msg::GraphPlannerCommand::COMMAND_GO_TO_LOCATION;
                    graph_planner_command.location = goal_point;
                    gp_command_pub->publish(graph_planner_command);
                }
            }
            usleep(100000);
        }
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv); // 初始化ROS2
    rclcpp::executors::MultiThreadedExecutor executor; // 创建多线程执行器
    nh = rclcpp::Node::make_shared("exploration");  // 创建名为exploration的节点
    executor.add_node(nh);

    // 声明参数
    nh->declare_parameter("interface/replay", play_bag);
    nh->declare_parameter("interface/dtime", dtime);
    nh->declare_parameter("interface/initX", init_x);
    nh->declare_parameter("interface/initY", init_y);
    nh->declare_parameter("interface/initZ", init_z);
    nh->declare_parameter("interface/initTime", init_time);
    nh->declare_parameter("interface/returnHomeThres", return_home_threshold);
    nh->declare_parameter("interface/robotMovingThres", robot_moving_threshold);
    nh->declare_parameter("interface/tfFrame", map_frame);
    nh->declare_parameter("interface/autoExp", begin_signal);
    nh->declare_parameter("interface/waypointTopic", waypoint_topic);
    nh->declare_parameter("interface/graphPlannerCommandTopic", gp_command_topic);
    nh->declare_parameter("interface/effectivePlanTimeTopic", effective_plan_time_topic);
    nh->declare_parameter("interface/totalPlanTimeTopic", total_plan_time_topic);
    nh->declare_parameter("interface/gpStatusTopic", gp_status_topic);
    nh->declare_parameter("interface/odomTopic", odom_topic);
    nh->declare_parameter("interface/beginSignalTopic", begin_signal_topic);
    nh->declare_parameter("interface/stopSignalTopic", stop_signal_topic);

    // 获取参数
    nh->get_parameter("interface/replay", play_bag);
    nh->get_parameter("interface/dtime", dtime);
    nh->get_parameter("interface/initX", init_x);
    nh->get_parameter("interface/initY", init_y);
    nh->get_parameter("interface/initZ", init_z);
    nh->get_parameter("interface/initTime", init_time);
    nh->get_parameter("interface/returnHomeThres", return_home_threshold);
    nh->get_parameter("interface/robotMovingThres", robot_moving_threshold);
    nh->get_parameter("interface/tfFrame", map_frame);
    nh->get_parameter("interface/autoExp", begin_signal);
    nh->get_parameter("interface/waypointTopic", waypoint_topic);
    nh->get_parameter("interface/graphPlannerCommandTopic", gp_command_topic);
    nh->get_parameter("interface/effectivePlanTimeTopic", effective_plan_time_topic);
    nh->get_parameter("interface/totalPlanTimeTopic", total_plan_time_topic);
    nh->get_parameter("interface/gpStatusTopic", gp_status_topic);
    nh->get_parameter("interface/odomTopic", odom_topic);
    nh->get_parameter("interface/beginSignalTopic", begin_signal_topic);
    nh->get_parameter("interface/stopSignalTopic", stop_signal_topic);

    // 创建可重入的回调组
    rclcpp::CallbackGroup::SharedPtr client_cb_group_ = nh->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::CallbackGroup::SharedPtr sub_cb_group_ = nh->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options;  // 创建订阅选项
    options.callback_group = sub_cb_group_; // 设置订阅选项的回调组
    // 创建各种发布者
    waypoint_pub = nh->create_publisher<geometry_msgs::msg::PointStamped>(waypoint_topic, 5);
    goal_pose_pub = nh->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    gp_command_pub = nh->create_publisher<graph_planner::msg::GraphPlannerCommand>(gp_command_topic, 2); // 创建地图规划器命令发布器
    effective_plan_time_pub = nh->create_publisher<std_msgs::msg::Float32>(effective_plan_time_topic, 2);
    total_plan_time_pub = nh->create_publisher<std_msgs::msg::Float32>(total_plan_time_topic, 2);
    stop_signal_pub = nh->create_publisher<std_msgs::msg::Bool>(stop_signal_topic, 2);

    gp_status_sub = nh->create_subscription<graph_planner::msg::GraphPlannerStatus>(gp_status_topic, 1, gp_status_callback, options);
    waypoint_sub = nh->create_subscription<geometry_msgs::msg::PointStamped>(waypoint_topic, 5, waypoint_callback, options);
    odom_sub = nh->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 5, odom_callback, options);
    begin_signal_sub = nh->create_subscription<std_msgs::msg::Bool>(begin_signal_topic, 5, begin_signal_callback, options);
    
    frontier_cleaner_client =
        nh->create_client<dsvplanner::srv::CleanFrontier>("cleanFrontierSrv", rmw_qos_profile_services_default, client_cb_group_);
    drrt_planner_client =
        nh->create_client<dsvplanner::srv::Dsvplanner>("drrtPlannerSrv", rmw_qos_profile_services_default, client_cb_group_);

    // 创建定时器，每隔0.1秒调用一次execute函数
    rclcpp::TimerBase::SharedPtr executeTimer_ = nh->create_wall_timer(0.1s, execute, sub_cb_group_);

    sleep(1);
    executor.spin_some();

    while (!begin_signal || !odom_is_ready)
    {
        usleep(500000);
        executor.spin_some();
        RCLCPP_INFO(nh->get_logger(), "Waiting for Odometry");  //之前经常卡在这里 等待里程计
    }

    RCLCPP_INFO(nh->get_logger(), "Starting the planner: Performing initialization motion");
    {
        tf2::Vector3 vec_init(init_x, init_y, init_z);
        tf2::Vector3 vec_goal;
        vec_goal = transformToMap * vec_init;
        geometry_msgs::msg::PointStamped wp;
        wp.header.frame_id = map_frame; //航点消息坐标系 时间戳 位姿
        wp.header.stamp = nh->now();
        wp.point.x = vec_goal.x();
        wp.point.y = vec_goal.y();
        wp.point.z = vec_goal.z();
        home_point.x = current_odom_x;
        home_point.y = current_odom_y;
        home_point.z = current_odom_z;

        usleep(500000); // wait for sometime to make sure waypoint can be
        // published properly

        waypoint_pub->publish(wp);

        geometry_msgs::msg::PoseStamped goal_msg;
        goal_msg.header.stamp = nh->now();
        goal_msg.header.frame_id = "map"; // 坐标系，通常使用 "map"

        // 设置目标位置坐标
        goal_msg.pose.position.x = wp.point.x; // 目标位置的 X 坐标
        goal_msg.pose.position.y = wp.point.y; // 目标位置的 Y 坐标
        goal_msg.pose.position.z = 0.0;

        // 设置目标朝向
        goal_msg.pose.orientation.x = 0.0;
        goal_msg.pose.orientation.y = 0.0;
        goal_msg.pose.orientation.z = 0.0;
        goal_msg.pose.orientation.w = 1.0; // 单位四元数（没有旋转）
        goal_pose_pub->publish(goal_msg);

        bool wp_ongoing = true;
        int init_time_count = 0;
        // 使得机器人到达初始位置，Vetintx
        while (wp_ongoing)
        { // Keep publishing initial waypoint until the robot
            // reaches that point
            init_time_count++;
            usleep(100000);
            executor.spin_some();
            vec_goal = transformToMap * vec_init;
            wp.point.x = vec_goal.x();
            wp.point.y = vec_goal.y();
            wp.point.z = vec_goal.z();
            waypoint_pub->publish(wp);

            geometry_msgs::msg::PoseStamped goal_msg;
            goal_msg.header.stamp = nh->now();
            goal_msg.header.frame_id = "map"; // 坐标系，通常使用 "map"

            // 设置目标位置坐标
            goal_msg.pose.position.x = wp.point.x; // 目标位置的 X 坐标
            goal_msg.pose.position.y = wp.point.y; // 目标位置的 Y 坐标
            goal_msg.pose.position.z = 0.0; // Z 坐标（一般为 0）

            // 设置目标朝向
            goal_msg.pose.orientation.x = 0.0;
            goal_msg.pose.orientation.y = 0.0;
            goal_msg.pose.orientation.z = 0.0;
            goal_msg.pose.orientation.w = 1.0; // 单位四元数（没有旋转）
            goal_pose_pub->publish(goal_msg);

            // 计算当前位置和目标位置的距离
            double dist = sqrt((wp.point.x - current_odom_x) * (wp.point.x - current_odom_x) +
                (wp.point.y - current_odom_y) * (wp.point.y - current_odom_y));
            // 计算当前位置和原点的距离
            double dist_to_home = sqrt((home_point.x - current_odom_x) * (home_point.x - current_odom_x) +
                (home_point.y - current_odom_y) * (home_point.y - current_odom_y));
            // 如果距离小于0.5且距离原点大于0.5，认为到达目标位置
            if (dist < 0.5 && dist_to_home > 0.5)
                wp_ongoing = false;
            // 如果初始化时间计数器大于等于初始化时间除以0.1且距离原点大于0.5，认为到达目标位置
            if (init_time_count >= init_time / 0.1 && dist_to_home > 0.5)
                wp_ongoing = false;
        }
    }
    // initilization();
    sleep(1);
    initialized = true;

    std::cout << std::endl
        << "\033[1;32mExploration Started\033[0m\n"
        << std::endl;
    total_time.data = 0;
    plan_start = steady_clock::now();
    executor.spin();
}
