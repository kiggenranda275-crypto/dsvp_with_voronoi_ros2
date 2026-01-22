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
// 定义控制终端光标移动和清除的转义字符
#define cursup "\033[A"
#define cursclean "\033[2K"
#define curshome "\033[0;0H"
#define UNIT 1000000000

// 定义航点、目标点、原点等
geometry_msgs::msg::Point wayPoint;
geometry_msgs::msg::Point wayPoint_pre;
geometry_msgs::msg::Point goal_point;
geometry_msgs::msg::Point home_point;
// 地图规划器命令消息
graph_planner::msg::GraphPlannerCommand graph_planner_command;
// 有效规划时间消息
std_msgs::msg::Float32 effective_time;
// 总规划时间消息
std_msgs::msg::Float32 total_time;

// 控制是否使用地图规划器跟随路径
bool play_bag = false;     
// 触发规划器的信号
bool begin_signal = false; 
// 地图规划器是否正在执行
bool gp_in_progress = false;
// 航点状态
bool wp_state = false;
// 是否返回原点
bool return_home = false;
// 里程计数据是否准备好
bool odom_is_ready = false;
// 是否成功调用DSVP规划器
bool call_dsvp_successfully = true;
// 是否初始化完成
bool initialized = false;
// 当前里程计的x、y、z坐标
double current_odom_x = 0;
double current_odom_y = 0;
double current_odom_z = 0;
// 上一时刻里程计的x、y、z坐标
double previous_odom_x = 0;
double previous_odom_y = 0;
double previous_odom_z = 0;
// 时间间隔
double dtime = 0.0;
// 初始位置的x、y、z坐标和初始时间
double init_x = 2;
double init_y = 0;
double init_z = 0;
double init_time = 2;
// 返回原点的阈值
double return_home_threshold = 1.5;
// 机器人移动的阈值
double robot_moving_threshold = 6;
// 地图坐标系名称
std::string map_frame = "map";
// 航点话题名称
std::string waypoint_topic = "/way_point";
// 地图规划器命令话题名称
std::string gp_command_topic = "/graph_planner_command";
// 有效规划时间话题名称
std::string effective_plan_time_topic = "/runtime";
// 总规划时间话题名称
std::string total_plan_time_topic = "/totaltime";
// 地图规划器状态话题名称
std::string gp_status_topic = "/graph_planner_status";
// 里程计话题名称
std::string odom_topic = "/state_estimation";
// 开始探索信号话题名称
std::string begin_signal_topic = "/start_exploring";
// 停止探索信号话题名称
std::string stop_signal_topic = "/stop_exploring";

// 到地图坐标系的变换
tf2::Transform transformToMap;

// 规划开始和结束时间点
steady_clock::time_point plan_start;
steady_clock::time_point plan_over;
// 规划时间间隔
steady_clock::duration time_span;

// ROS2节点指针
rclcpp::Node::SharedPtr nh = nullptr;
rclcpp::Node::SharedPtr nh2 = nullptr;
// 航点发布器
rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_pub;
// 目标位姿发布器
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub;

// 地图规划器命令发布器
rclcpp::Publisher<graph_planner::msg::GraphPlannerCommand>::SharedPtr gp_command_pub;
// 有效规划时间发布器
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr effective_plan_time_pub;
// 总规划时间发布器
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr total_plan_time_pub;
// 停止信号发布器
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_signal_pub;

// 地图规划器状态订阅器
rclcpp::Subscription<graph_planner::msg::GraphPlannerStatus>::SharedPtr gp_status_sub;
// 航点订阅器
rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_sub;
// 里程计订阅器
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
// 开始信号订阅器
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr begin_signal_sub;

// 边界清除服务客户端
rclcpp::Client<dsvplanner::srv::CleanFrontier>::SharedPtr frontier_cleaner_client;
// DSV规划器服务客户端
rclcpp::Client<dsvplanner::srv::Dsvplanner>::SharedPtr drrt_planner_client;

// 地图规划器状态回调函数
void gp_status_callback(const graph_planner::msg::GraphPlannerStatus::SharedPtr msg)
{
    // 如果地图规划器正在执行，设置标志位为true
    if (msg->status == graph_planner::msg::GraphPlannerStatus::STATUS_IN_PROGRESS)
        gp_in_progress = true;
    else
    {
        // 否则设置标志位为false
        gp_in_progress = false;
    }
}

// 航点回调函数
void waypoint_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    // 更新航点
    wayPoint = msg->point;
    // 设置航点状态为true
    wp_state = true;
}

// 里程计回调函数
void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // 更新当前里程计的x、y、z坐标
    current_odom_x = msg->pose.pose.position.x;
    current_odom_y = msg->pose.pose.position.y;
    current_odom_z = msg->pose.pose.position.z;

    // 更新到地图坐标系的变换
    transformToMap.setOrigin(
        tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    transformToMap.setRotation(tf2::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
    // 设置里程计数据准备好标志位为true
    odom_is_ready = true;
}

// 开始信号回调函数
void begin_signal_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // 更新开始信号
    begin_signal = msg->data;
}

// 检查机器人位置是否改变
bool robotPositionChange()
{
    // 计算当前位置和上一时刻位置的距离
    double dist = sqrt((current_odom_x - previous_odom_x) * (current_odom_x - previous_odom_x) +
        (current_odom_y - previous_odom_y) * (current_odom_y - previous_odom_y) +
        (current_odom_z - previous_odom_z) * (current_odom_z - previous_odom_z));
    // 如果距离小于阈值，认为机器人没有移动
    if (dist < robot_moving_threshold)
        return false;
    // 更新上一时刻的位置
    previous_odom_x = current_odom_x;
    previous_odom_y = current_odom_y;
    previous_odom_z = current_odom_z;
    return true;
}

// 主要执行函数，通过间隔一定时间循环调用
void execute()
{
    // 迭代次数
    static int iteration = 0;
    if (initialized)
    {
        if (!return_home)
        {
            // 逻辑判断
            if (call_dsvp_successfully)
            {
                // 输出当前迭代次数
                std::cout << "Planning iteration " << iteration << std::endl;
                // 创建DSV规划器服务请求
                auto planSrv = std::make_shared<dsvplanner::srv::Dsvplanner::Request>();
                // 创建边界清除服务请求
                auto cleanSrv = std::make_shared<dsvplanner::srv::CleanFrontier::Request>();
                // 设置请求的时间戳
                planSrv->header.stamp = nh->now();

                // 设置调用DSVP规划器标志位为false
                call_dsvp_successfully = false;
                // 定义异步请求的回调函数
                auto callback = [&](rclcpp::Client<dsvplanner::srv::Dsvplanner>::SharedFuture future)
                    {
                        // 获取服务响应结果
                        auto result = future.get();

                        // 记录规划结束时间
                        plan_over = steady_clock::now();
                        // 计算规划时间间隔
                        time_span = plan_over - plan_start;
                        // 计算有效规划时间
                        effective_time.data = float(time_span.count()) * steady_clock::period::num / steady_clock::period::den;

                        if (result->goal.size() == 0)
                        { // usually the size should be 1 if planning successfully
                            // 如果规划失败，发布有效规划时间
                            effective_plan_time_pub->publish(effective_time);
                        }
                        else
                        {
                            if (result->mode == 2)
                            {
                                // 如果规划模式为2，设置返回原点标志位为true
                                return_home = true;
                                // 设置目标点为原点
                                goal_point = home_point;
                                // 输出探索完成，返回原点的信息
                                std::cout << std::endl
                                    << "\033[1;32m Exploration completed, returning home \033[0m" << std::endl
                                    << std::endl;
                                // 设置有效规划时间为0
                                effective_time.data = 0;
                                // 发布有效规划时间
                                effective_plan_time_pub->publish(effective_time);
                            }
                            else
                            {
                                // 否则设置返回原点标志位为false
                                return_home = false;
                                // 设置目标点为规划结果的第一个目标点
                                goal_point = result->goal[0];
                                // 发布有效规划时间
                                effective_plan_time_pub->publish(effective_time);
                            }

                            // 累加总规划时间
                            total_time.data += effective_time.data;
                            // 发布总规划时间
                            total_plan_time_pub->publish(total_time);
                            // 不使用测试模拟模式
                            if (!play_bag)
                            {
                                // 设置地图规划器命令为前往目标点
                                graph_planner_command.command = graph_planner::msg::GraphPlannerCommand::COMMAND_GO_TO_LOCATION;
                                // 设置目标点
                                graph_planner_command.location = goal_point;
                                // 发布地图规划器命令
                                gp_command_pub->publish(graph_planner_command); 

                                // 创建目标位姿消息
                                geometry_msgs::msg::PoseStamped goal_pose;
                                // 设置时间戳
                                goal_pose.header.stamp = nh->now();
                                // 设置坐标系
                                goal_pose.header.frame_id = map_frame;
                                // 设置目标位置
                                goal_pose.pose.position.x = goal_point.x;
                                goal_pose.pose.position.y = goal_point.y;
                                goal_pose.pose.position.z = 0.0;
                                // 设置目标朝向
                                goal_pose.pose.orientation.x = 0.0;
                                goal_pose.pose.orientation.y = 0.0;
                                goal_pose.pose.orientation.z = 0.0;
                                goal_pose.pose.orientation.w = 1.0;
                                // 发布目标位姿
                                goal_pose_pub->publish(goal_pose); 

                                // 睡眠1秒
                                sleep(1);
                                // 计数器
                                int count = 200;
                                // 更新上一时刻的位置
                                previous_odom_x = current_odom_x;
                                previous_odom_y = current_odom_y;
                                previous_odom_z = current_odom_z;
                                while (gp_in_progress)
                                {
                                    // 如果机器人没有移动，计数器减1，如果计数器小于等于0，清除对应的边界；20s内
                                    usleep(100000); // seconds, then give up the goal
                                    // 保存上一时刻的航点
                                    wayPoint_pre = wayPoint;
                                    // 检查机器人是否移动
                                    bool robotMoving = robotPositionChange(); 
                                    if (robotMoving)
                                    {
                                        // 如果机器人移动，重置计数器
                                        count = 200;
                                    }
                                    else
                                    {
                                        // 否则计数器减1
                                        count--;
                                    }
                                    if (count <= 0)
                                    { // when the goal point cannot be reached, clean
                                        // its correspoinding frontier if there is
                                        // 设置边界清除服务请求的时间戳
                                        cleanSrv->header.stamp = nh->now();
                                        // 设置边界清除服务请求的坐标系
                                        cleanSrv->header.frame_id = map_frame;
                                        // 异步发送边界清除服务请求
                                        auto response = frontier_cleaner_client->async_send_request(cleanSrv); 

                                        // 睡眠1毫秒
                                        usleep(1000);
                                        break;
                                    }
                                }
                                // 设置地图规划器命令为禁用
                                graph_planner_command.command = graph_planner::msg::GraphPlannerCommand::COMMAND_DISABLE;
                                // 发布地图规划器命令
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
                                    // 设置目标点
                                    graph_planner_command.location = result->goal[i];
                                    // 发布地图规划器命令
                                    gp_command_pub->publish(graph_planner_command);
                                    while (1)
                                    {
                                    }
                                    break;
                                }
                            }
                            // 记录下一次规划开始时间
                            plan_start = steady_clock::now();
                        }
                        // 设置调用DSVP规划器标志位为true
                        call_dsvp_successfully = true;
                        // Use result->responses[] and do something
                    };
                // 异步发送DSV规划器服务请求
                auto result_future = drrt_planner_client->async_send_request(planSrv, callback);
                // 迭代次数加1
                iteration++;
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
                // 清除当前行
                printf(cursclean);
                // 输出返回原点完成的信息
                std::cout << "\033[1;32mReturn home completed\033[0m" << std::endl;
                // 光标上移一行
                printf(cursup);
                // 创建停止探索消息
                std_msgs::msg::Bool stop_exploring;
                // 设置停止探索标志位为true
                stop_exploring.data = true;
                // 发布停止探索消息
                stop_signal_pub->publish(stop_exploring);
            }
            else
            {
                while (!gp_in_progress)
                {
                    // 睡眠2秒
                    sleep(2);
                    // executor.spin_some();
                    // 设置地图规划器命令为前往目标点
                    graph_planner_command.command = graph_planner::msg::GraphPlannerCommand::COMMAND_GO_TO_LOCATION;
                    // 设置目标点
                    graph_planner_command.location = goal_point;
                    // 发布地图规划器命令
                    gp_command_pub->publish(graph_planner_command);
                }
            }
            // 睡眠100毫秒
            usleep(100000);
        }
    }
}

int main(int argc, char** argv)
{
    // 初始化ROS2
    rclcpp::init(argc, argv);
    // 创建多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor;
    // 创建名为exploration的节点
    nh = rclcpp::Node::make_shared("exploration");
    // 将节点添加到执行器中
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
    // 创建订阅选项
    rclcpp::SubscriptionOptions options;
    // 设置订阅选项的回调组
    options.callback_group = sub_cb_group_;

    // 创建航点发布器
    waypoint_pub = nh->create_publisher<geometry_msgs::msg::PointStamped>(waypoint_topic, 5);
    // 创建目标位姿发布器
    goal_pose_pub = nh->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    // 创建地图规划器命令发布器
    gp_command_pub = nh->create_publisher<graph_planner::msg::GraphPlannerCommand>(gp_command_topic, 2);
    // 创建有效规划时间发布器
    effective_plan_time_pub = nh->create_publisher<std_msgs::msg::Float32>(effective_plan_time_topic, 2);
    // 创建总规划时间发布器
    total_plan_time_pub = nh->create_publisher<std_msgs::msg::Float32>(total_plan_time_topic, 2);
    // 创建停止信号发布器
    stop_signal_pub = nh->create_publisher<std_msgs::msg::Bool>(stop_signal_topic, 2);

    // 创建地图规划器状态订阅器
    gp_status_sub = nh->create_subscription<graph_planner::msg::GraphPlannerStatus>(gp_status_topic, 1, gp_status_callback, options);
    // 创建航点订阅器
    waypoint_sub = nh->create_subscription<geometry_msgs::msg::PointStamped>(waypoint_topic, 5, waypoint_callback, options);
    // 创建里程计订阅器
    odom_sub = nh->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 5, odom_callback, options);
    // 创建开始信号订阅器
    begin_signal_sub = nh->create_subscription<std_msgs::msg::Bool>(begin_signal_topic, 5, begin_signal_callback, options);

    // 创建边界清除服务客户端
    frontier_cleaner_client =
        nh->create_client<dsvplanner::srv::CleanFrontier>("cleanFrontierSrv", rmw_qos_profile_services_default, client_cb_group_);
    // 创建DSV规划器服务客户端
    drrt_planner_client =
        nh->create_client<dsvplanner::srv::Dsvplanner>("drrtPlannerSrv", rmw_qos_profile_services_default, client_cb_group_);

    // 创建定时器，每隔0.1秒调用一次execute函数
    rclcpp::TimerBase::SharedPtr executeTimer_ = nh->create_wall_timer(0.1s, execute, sub_cb_group_);

    // 睡眠1秒
    sleep(1);
    // 执行器处理一次回调
    executor.spin_some();

    // 等待开始信号和里程计数据准备好
    while (!begin_signal || !odom_is_ready)
    {
        // 睡眠500毫秒
        usleep(500000);
        // 执行器处理一次回调
        executor.spin_some();
        // 输出等待里程计数据的信息
        RCLCPP_INFO(nh->get_logger(), "Waiting for Odometry");
    }

    // 输出开始规划器，进行初始化运动的信息
    RCLCPP_INFO(nh->get_logger(), "Starting the planner: Performing initialization motion");
    {
        // 创建初始位置向量
        tf2::Vector3 vec_init(init_x, init_y, init_z);
        // 创建目标位置向量
        tf2::Vector3 vec_goal;
        // 计算目标位置向量
        vec_goal = transformToMap * vec_init;
        // 创建航点消息
        geometry_msgs::msg::PointStamped wp;
        // 设置航点消息的坐标系
        wp.header.frame_id = map_frame;
        // 设置航点消息的时间戳
        wp.header.stamp = nh->now();
        // 设置航点消息的位置
        wp.point.x = vec_goal.x();
        wp.point.y = vec_goal.y();
        wp.point.z = vec_goal.z();
        // 设置原点位置
        home_point.x = current_odom_x;
        home_point.y = current_odom_y;
        home_point.z = current_odom_z;

        // 睡眠500毫秒
        usleep(500000); // wait for sometime to make sure waypoint can be
        // published properly

        // 发布航点消息
        waypoint_pub->publish(wp);

        // 创建目标位姿消息
        geometry_msgs::msg::PoseStamped goal_msg;
        // 设置目标位姿消息的时间戳
        goal_msg.header.stamp = nh->now();
        // 设置目标位姿消息的坐标系
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
        // 发布目标位姿消息
        goal_pose_pub->publish(goal_msg);

        // 航点执行标志位
        bool wp_ongoing = true;
        // 初始化时间计数器
        int init_time_count = 0;
        // 使得机器人到达初始位置，Vetintx
        while (wp_ongoing)
        { // Keep publishing initial waypoint until the robot
            // reaches that point
            // 初始化时间计数器加1
            init_time_count++;
            // 睡眠100毫秒
            usleep(100000);
            // 执行器处理一次回调
            executor.spin_some();
            // 计算目标位置向量
            vec_goal = transformToMap * vec_init;
            // 更新航点消息的位置
            wp.point.x = vec_goal.x();
            wp.point.y = vec_goal.y();
            wp.point.z = vec_goal.z();
            // 发布航点消息
            waypoint_pub->publish(wp);

            // 创建目标位姿消息
            geometry_msgs::msg::PoseStamped goal_msg;
            // 设置目标位姿消息的时间戳
            goal_msg.header.stamp = nh->now();
            // 设置目标位姿消息的坐标系
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
            // 发布目标位姿消息
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
    // 睡眠1秒
    sleep(1);
    // 设置初始化完成标志位为true
    initialized = true;

    // 输出探索开始的信息
    std::cout << std::endl
        << "\033[1;32mExploration Started\033[0m\n"
        << std::endl;
    // 初始化总规划时间为0
    total_time.data = 0;
    // 记录规划开始时间
    plan_start = steady_clock::now();
    // 执行器循环处理回调
    executor.spin();
}
