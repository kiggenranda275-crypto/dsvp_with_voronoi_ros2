/*
drrtp.cpp
Implementation of drrt_planner class

Created by Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
05/25/2020
*/

#include <eigen3/Eigen/Dense>

#include <dsvplanner/drrtp.h>

using namespace Eigen;

dsvplanner_ns::drrtPlanner::drrtPlanner(rclcpp::Node::SharedPtr& node_handle)
    : nh_(node_handle)
{
    if (!setParams())
    {
        RCLCPP_ERROR(nh_->get_logger(), "Set parameters fail. Cannot start planning!");
    }

    manager_ = new octomanager(nh_);
    dual_state_graph_ = new DualStateGraph(nh_, manager_);
    dual_state_frontier_ = new DualStateFrontier(nh_, manager_);
    drrt_ = new Drrt(nh_, manager_, dual_state_graph_, dual_state_frontier_);

    init();
    drrt_->setParams(params_);
    drrt_->init();

    RCLCPP_INFO(nh_->get_logger(), "Successfully launched DSVP node");
}

dsvplanner_ns::drrtPlanner::~drrtPlanner()
{
    if (manager_)
    {
        delete manager_;
    }
    if (dual_state_graph_)
    {
        delete dual_state_graph_;
    }
    if (dual_state_frontier_)
    {
        delete dual_state_frontier_;
    }
    if (drrt_)
    {
        delete drrt_;
    }
}

void dsvplanner_ns::drrtPlanner::odomCallback(const nav_msgs::msg::Odometry::SharedPtr pose)
{
    drrt_->setRootWithOdom(*pose);
    // 设置规划器准备好
    drrt_->plannerReady_ = true;
}

void dsvplanner_ns::drrtPlanner::boundaryCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr boundary)
{
    drrt_->setBoundary(*boundary);
    dual_state_frontier_->setBoundary(*boundary);
}

// 主要执行函数，通过service调用
void dsvplanner_ns::drrtPlanner::plannerServiceCallback(const dsvplanner::srv::Dsvplanner::Request::SharedPtr req,
    dsvplanner::srv::Dsvplanner::Response::SharedPtr res)
{
    plan_start_ = std::chrono::steady_clock::now();
    // drrt_->gotoxy(0, 10);  // Go to the specific line on the screen
    // Check if the planner is ready.
    if (!drrt_->plannerReady_)
    {
        std::cout << "No odometry. Planner is not ready!" << std::endl;
        return;
    }
    if (manager_ == NULL)
    {
        std::cout << "No octomap. Planner is not ready!" << std::endl;
        return;
    }
    if (manager_->octree_->size() == 0)
    {
        std::cout << "Octomap is empty. Planner is not set up!" << std::endl;
        return;
    }
    // 设置地形图，terrain map
    drrt_->setTerrainVoxelElev();
    // 清除上一次选择的全局边界
    cleanLastSelectedGlobalFrontier();
    drrt_->clear();
    //  重新初始化
    drrt_->plannerInit();

    // 循环迭代Kdtree
    int loopCount = 0;
    while (rclcpp::ok() && drrt_->remainingFrontier_ && drrt_->getNodeCounter() < params_.kCuttoffIterations &&
        !(drrt_->normal_local_iteration_ && (drrt_->getNodeCounter() >= params_.kVertexSize))) //&& drrt_->gainFound()
    {
        if (loopCount > drrt_->loopCount_ * (drrt_->getNodeCounter() + 1))
        {
            break;
        }
        drrt_->plannerIterate();
        loopCount++;
    }

    RCLCPP_INFO(nh_->get_logger(), "local graph vetex size: %d  RRT node num %d", dual_state_graph_->local_graph_.vertices.size(), drrt_->getNodeCounter());
    // 发布RRT树
    drrt_->publishNode();

    RCLCPP_INFO(nh_->get_logger(), "\033[31m RRT树扩展迭代完成 \033[0m");
    RCLCPP_INFO(nh_->get_logger(), "     New node number is %d \n      Current local graph size is %d \n      Current global graph size is %d",
        drrt_->getNodeCounter(), dual_state_graph_->getLocalVertexSize(), dual_state_graph_->getGlobalVertexSize());
    // RCLCPP_INFO( "     New node number is " << drrt_->getNodeCounter() << "\n"
    //           << "     Current local RRT size is " << dual_state_graph_->getLocalVertexSize() << "\n"
    //           << "     Current global graph size is " << dual_state_graph_->getGlobalVertexSize() << std::endl;

    RRT_generate_over_ = std::chrono::steady_clock::now();
    time_span = RRT_generate_over_ - plan_start_;
    double rrtGenerateTime =
        double(time_span.count()) * std::chrono::steady_clock::period::num / std::chrono::steady_clock::period::den;
    // 重置规划期状态
    drrt_->global_plan_pre_ = drrt_->global_plan_;
    drrt_->global_plan_ = false;
    drrt_->local_plan_ = false;

    // 更新最后一次rrt规划状态
    dual_state_frontier_->setPlannerStatus(drrt_->global_plan_pre_);

    // 更新下一次迭代的规划器状态，重新设置robot position 根据odomCallback ，由slam给出
    geometry_msgs::msg::Point robot_position;
    robot_position.x = drrt_->root_[0];
    robot_position.y = drrt_->root_[1];
    robot_position.z = drrt_->root_[2];

    // 如果drrt没有下一个节点，切全局规划开启，且增益小于0，返回home
    if (!drrt_->nextNodeFound_ && drrt_->global_plan_pre_ && drrt_->gainFound() <= 0) //
    {
        drrt_->return_home_ = true;
        geometry_msgs::msg::Point home_position;
        home_position.x = 0;
        home_position.y = 0;
        home_position.z = 0;
        res->goal.push_back(home_position);
        res->mode = 2; // mode 2 means returning home

        dual_state_frontier_->cleanAllUselessFrontiers();
        return;
    }
    // 如果drrt没有下一个节点，切全局规划开启，且增益大于0，
    else if (!drrt_->nextNodeFound_ && !drrt_->global_plan_pre_ && dual_state_graph_->getGain(robot_position) <= 0)
    {
        drrt_->global_plan_ = true;
        RCLCPP_INFO(nh_->get_logger(), "\033[31m RRT树没有对象,返回全局重新定位阶段 \033[0m");
        RCLCPP_INFO(nh_->get_logger(), "     No Remaining local frontiers  \n    Switch to relocation stage \n     Total plan lasted %f", 0);
        // std::cout << "     No Remaining local frontiers  "
        //           << "\n"
        //           << "     Switch to relocation stage "
        //           << "\n"
        //           << "     Total plan lasted " << 0 << std::endl;
        return;
    }
    else
    {
        drrt_->local_plan_ = true;
    }

    gain_computation_over_ = std::chrono::steady_clock::now();
    time_span = gain_computation_over_ - RRT_generate_over_;
    double getGainTime =
        double(time_span.count()) * std::chrono::steady_clock::period::num / std::chrono::steady_clock::period::den;

    // 扩展提取下一个目标，获取下一个最优视点，赋值给next_goal_position
    geometry_msgs::msg::Point next_goal_position;
    if (drrt_->nextNodeFound_)
    {
        dual_state_graph_->best_vertex_id_ = drrt_->NextBestNodeIdx_;
        dual_state_graph_->updateExploreDirection();
        next_goal_position = dual_state_graph_->getBestGlobalVertexPosition();
    }
    else if (drrt_->global_plan_pre_ == true && drrt_->gainFound())
    {
        dual_state_graph_->best_vertex_id_ = drrt_->bestNodeId_;
        dual_state_graph_->updateExploreDirection();
        next_goal_position = dual_state_graph_->getBestLocalVertexPosition();
    }
    else
    {
        dual_state_graph_->updateGlobalGraph();
        dual_state_graph_->updateExploreDirection();
        next_goal_position = dual_state_graph_->getBestLocalVertexPosition();
    }
    dual_state_graph_->setCurrentPlannerStatus(drrt_->global_plan_pre_);
    res->goal.push_back(next_goal_position);
    res->mode = 1; // mode 1 means exploration

    // 发布下一个目标点
    geometry_msgs::msg::PointStamped next_goal_point;
    next_goal_point.header.frame_id = "map";
    next_goal_point.point = next_goal_position;
    params_.nextGoalPub_->publish(next_goal_point);

    // geometry_msgs::msg::PoseStamped next_goal_pose;
    // next_goal_pose.header.stamp = nh_->now();
    // next_goal_pose.header.frame_id = "map";
    // next_goal_pose.pose.position.x = next_goal_position.x;
    // next_goal_pose.pose.position.y = next_goal_position.y;
    // next_goal_pose.pose.position.z = next_goal_position.z;
    // next_goal_pose.pose.orientation.w = 1;
    // next_goal_pose.pose.orientation.x = 0;
    // next_goal_pose.pose.orientation.y = 0;
    // next_goal_pose.pose.orientation.z = 0;
    // params_.nextGoalPosePub_->publish(next_goal_pose);


    plan_over_ = std::chrono::steady_clock::now();
    time_span = plan_over_ - plan_start_;
    double plantime = double(time_span.count()) * std::chrono::steady_clock::period::num / std::chrono::steady_clock::period::den;
    RCLCPP_INFO(nh_->get_logger(), "\033[31m RRT生成规划阶段完成 \033[0m");
    std::cout << "     RRT generation lasted  " << rrtGenerateTime << "\n"
        << "     Computiong gain lasted " << getGainTime << "\n"
        << "     Total plan lasted " << plantime << std::endl;
}

void dsvplanner_ns::drrtPlanner::cleanFrontierServiceCallback(const dsvplanner::srv::CleanFrontier::Request::SharedPtr req,
    dsvplanner::srv::CleanFrontier::Response::SharedPtr res)
{
    RCLCPP_INFO(nh_->get_logger(), "Clean frontier service called !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1");
    if (drrt_->nextNodeFound_)
    {
        dual_state_frontier_->updateToCleanFrontier(drrt_->selectedGlobalFrontier_); // 清除这一次的前沿点
        dual_state_frontier_->gloabalFrontierUpdate();
    }
    else
    {
        dual_state_graph_->clearLocalGraph();
    }
    res->success = true;
}

// 只有当最后一个计划是全局计划时，才会执行此函数来清除
void dsvplanner_ns::drrtPlanner::cleanLastSelectedGlobalFrontier()
{

    if (drrt_->nextNodeFound_)
    {
        dual_state_frontier_->updateToCleanFrontier(drrt_->selectedGlobalFrontier_);
        dual_state_frontier_->gloabalFrontierUpdate();
    }
}

bool dsvplanner_ns::drrtPlanner::setParams()
{
    nh_->declare_parameter("drrt/gain/kFree", params_.kGainFree);
    nh_->declare_parameter("drrt/gain/kOccupied", params_.kGainOccupied);
    nh_->declare_parameter("drrt/gain/kUnknown", params_.kGainUnknown);
    nh_->declare_parameter("drrt/gain/kMinEffectiveGain", params_.kMinEffectiveGain);
    nh_->declare_parameter("drrt/gain/kRange", params_.kGainRange);
    nh_->declare_parameter("drrt/gain/kRangeZMinus", params_.kGainRangeZMinus);
    nh_->declare_parameter("drrt/gain/kRangeZPlus", params_.kGainRangeZPlus);
    nh_->declare_parameter("drrt/gain/kZero", params_.kZeroGain);
    nh_->declare_parameter("drrt/gain/kGainFrontier", params_.kGainFrontier);
    nh_->declare_parameter("drrt/tree/kExtensionRange", params_.kExtensionRange);
    nh_->declare_parameter("drrt/tree/kMinExtensionRange", params_.kMinextensionRange);
    nh_->declare_parameter("drrt/tree/kMaxExtensionAlongZ", params_.kMaxExtensionAlongZ);
    nh_->declare_parameter("drrt/tree/kExactRoot", params_.kExactRoot);
    nh_->declare_parameter("drrt/tree/kCuttoffIterations", params_.kCuttoffIterations);
    nh_->declare_parameter("drrt/tree/kGlobalExtraIterations", params_.kGlobalExtraIterations);
    nh_->declare_parameter("drrt/tree/kRemainingNodeScaleSize", params_.kRemainingNodeScaleSize);
    nh_->declare_parameter("drrt/tree/kRemainingBranchScaleSize", params_.kRemainingBranchScaleSize);
    nh_->declare_parameter("drrt/tree/kNewNodeScaleSize", params_.kNewNodeScaleSize);
    nh_->declare_parameter("drrt/tree/kNewBranchScaleSize", params_.kNewBranchScaleSize);
    nh_->declare_parameter("drrt/tfFrame", params_.explorationFrame);
    nh_->declare_parameter("drrt/vertexSize", params_.kVertexSize);
    nh_->declare_parameter("drrt/keepTryingNum", params_.kKeepTryingNum);
    nh_->declare_parameter("drrt/kLoopCountThres", params_.kLoopCountThres);

    nh_->declare_parameter("planner/odomSubTopic", odomSubTopic);
    nh_->declare_parameter("planner/boundarySubTopic", boundarySubTopic);
    nh_->declare_parameter("planner/terrainCloudSubTopic", terrainCloudSubTopic);
    nh_->declare_parameter("planner/newTreePathPubTopic", newTreePathPubTopic);
    nh_->declare_parameter("planner/remainingTreePathPubTopic", remainingTreePathPubTopic);
    nh_->declare_parameter("planner/boundaryPubTopic", boundaryPubTopic);
    nh_->declare_parameter("planner/globalSelectedFrontierPubTopic", globalSelectedFrontierPubTopic);
    nh_->declare_parameter("planner/localSelectedFrontierPubTopic", localSelectedFrontierPubTopic);
    nh_->declare_parameter("planner/plantimePubTopic", plantimePubTopic);
    nh_->declare_parameter("planner/nextGoalPubTopic", nextGoalPubTopic);
    nh_->declare_parameter("planner/randomSampledPointsPubTopic", randomSampledPointsPubTopic);
    nh_->declare_parameter("planner/shutDownTopic", shutDownTopic);
    nh_->declare_parameter("planner/plannerServiceName", plannerServiceName);
    nh_->declare_parameter("planner/cleanFrontierServiceName", cleanFrontierServiceName);

    nh_->declare_parameter("lb/kMinXLocal", params_.kMinXLocalBound);
    nh_->declare_parameter("lb/kMinYLocal", params_.kMinYLocalBound);
    nh_->declare_parameter("lb/kMinZLocal", params_.kMinZLocalBound);
    nh_->declare_parameter("lb/kMaxXLocal", params_.kMaxXLocalBound);
    nh_->declare_parameter("lb/kMaxYLocal", params_.kMaxYLocalBound);
    nh_->declare_parameter("lb/kMaxZLocal", params_.kMaxZLocalBound);

    nh_->declare_parameter("gb/kMinXGlobal", params_.kMinXGlobalBound);
    nh_->declare_parameter("gb/kMinYGlobal", params_.kMinYGlobalBound);
    nh_->declare_parameter("gb/kMinZGlobal", params_.kMinZGlobalBound);
    nh_->declare_parameter("gb/kMaxXGlobal", params_.kMaxXGlobalBound);
    nh_->declare_parameter("gb/kMaxYGlobal", params_.kMaxYGlobalBound);
    nh_->declare_parameter("gb/kMaxZGlobal", params_.kMaxZGlobalBound);

    nh_->declare_parameter("rm/kSensorPitch", params_.sensorPitch);
    nh_->declare_parameter("rm/kSensorHorizontal", params_.sensorHorizontalView);
    nh_->declare_parameter("rm/kSensorVertical", params_.sensorVerticalView);
    nh_->declare_parameter("rm/kVehicleHeight", params_.kVehicleHeight);
    nh_->declare_parameter("rm/kBoundX", params_.boundingBox[0]);
    nh_->declare_parameter("rm/kBoundY", params_.boundingBox[1]);
    nh_->declare_parameter("rm/kBoundZ", params_.boundingBox[2]);

    nh_->declare_parameter("elevation/kTerrainVoxelSize", params_.kTerrainVoxelSize);
    nh_->declare_parameter("elevation/kTerrainVoxelWidth", params_.kTerrainVoxelWidth);
    nh_->declare_parameter("elevation/kTerrainVoxelHalfWidth", params_.kTerrainVoxelHalfWidth);

    nh_->get_parameter("rm/kSensorPitch", params_.sensorPitch);
    nh_->get_parameter("rm/kSensorHorizontal", params_.sensorHorizontalView);
    nh_->get_parameter("rm/kSensorVertical", params_.sensorVerticalView);
    nh_->get_parameter("rm/kVehicleHeight", params_.kVehicleHeight);
    nh_->get_parameter("rm/kBoundX", params_.boundingBox[0]);
    nh_->get_parameter("rm/kBoundY", params_.boundingBox[1]);
    nh_->get_parameter("rm/kBoundZ", params_.boundingBox[2]);
    nh_->get_parameter("drrt/gain/kFree", params_.kGainFree);
    nh_->get_parameter("drrt/gain/kOccupied", params_.kGainOccupied);
    nh_->get_parameter("drrt/gain/kUnknown", params_.kGainUnknown);
    nh_->get_parameter("drrt/gain/kMinEffectiveGain", params_.kMinEffectiveGain);
    nh_->get_parameter("drrt/gain/kRange", params_.kGainRange);
    nh_->get_parameter("drrt/gain/kRangeZMinus", params_.kGainRangeZMinus);
    nh_->get_parameter("drrt/gain/kRangeZPlus", params_.kGainRangeZPlus);
    nh_->get_parameter("drrt/gain/kZero", params_.kZeroGain);
    nh_->get_parameter("drrt/gain/kGainFrontier", params_.kGainFrontier);

    nh_->get_parameter("drrt/tree/kExtensionRange", params_.kExtensionRange);
    nh_->get_parameter("drrt/tree/kMinExtensionRange", params_.kMinextensionRange);
    nh_->get_parameter("drrt/tree/kMaxExtensionAlongZ", params_.kMaxExtensionAlongZ);
    nh_->get_parameter("/rrt/tree/kExactRoot", params_.kExactRoot);
    nh_->get_parameter("drrt/tree/kCuttoffIterations", params_.kCuttoffIterations);
    nh_->get_parameter("drrt/tree/kGlobalExtraIterations", params_.kGlobalExtraIterations);
    nh_->get_parameter("drrt/tree/kRemainingNodeScaleSize", params_.kRemainingNodeScaleSize);
    nh_->get_parameter("drrt/tree/kRemainingBranchScaleSize", params_.kRemainingBranchScaleSize);
    nh_->get_parameter("drrt/tree/kNewNodeScaleSize", params_.kNewNodeScaleSize);
    nh_->get_parameter("drrt/tree/kNewBranchScaleSize", params_.kNewBranchScaleSize);
    nh_->get_parameter("drrt/tfFrame", params_.explorationFrame);
    nh_->get_parameter("drrt/vertexSize", params_.kVertexSize);
    nh_->get_parameter("drrt/keepTryingNum", params_.kKeepTryingNum);
    nh_->get_parameter("drrt/kLoopCountThres", params_.kLoopCountThres);
    nh_->get_parameter("lb/kMinXLocal", params_.kMinXLocalBound);
    nh_->get_parameter("lb/kMinYLocal", params_.kMinYLocalBound);
    nh_->get_parameter("lb/kMinZLocal", params_.kMinZLocalBound);
    nh_->get_parameter("lb/kMaxXLocal", params_.kMaxXLocalBound);
    nh_->get_parameter("lb/kMaxYLocal", params_.kMaxYLocalBound);
    nh_->get_parameter("lb/kMaxZLocal", params_.kMaxZLocalBound);
    nh_->get_parameter("gb/kMinXGlobal", params_.kMinXGlobalBound);
    nh_->get_parameter("gb/kMinYGlobal", params_.kMinYGlobalBound);
    nh_->get_parameter("gb/kMinZGlobal", params_.kMinZGlobalBound);
    nh_->get_parameter("gb/kMaxXGlobal", params_.kMaxXGlobalBound);
    nh_->get_parameter("gb/kMaxYGlobal", params_.kMaxYGlobalBound);
    nh_->get_parameter("gb/kMaxZGlobal", params_.kMaxZGlobalBound);
    nh_->get_parameter("elevation/kTerrainVoxelSize", params_.kTerrainVoxelSize);
    nh_->get_parameter("elevation/kTerrainVoxelWidth", params_.kTerrainVoxelWidth);
    nh_->get_parameter("elevation/kTerrainVoxelHalfWidth", params_.kTerrainVoxelHalfWidth);
    nh_->get_parameter("planner/odomSubTopic", odomSubTopic);
    nh_->get_parameter("planner/boundarySubTopic", boundarySubTopic);
    nh_->get_parameter("planner/newTreePathPubTopic", newTreePathPubTopic);
    nh_->get_parameter("planner/remainingTreePathPubTopic", remainingTreePathPubTopic);
    nh_->get_parameter("planner/boundaryPubTopic", boundaryPubTopic);
    nh_->get_parameter("planner/globalSelectedFrontierPubTopic", globalSelectedFrontierPubTopic);
    nh_->get_parameter("planner/localSelectedFrontierPubTopic", localSelectedFrontierPubTopic);
    nh_->get_parameter("planner/plantimePubTopic", plantimePubTopic);
    nh_->get_parameter("planner/nextGoalPubTopic", nextGoalPubTopic);
    nh_->get_parameter("planner/randomSampledPointsPubTopic", randomSampledPointsPubTopic);
    nh_->get_parameter("planner/shutDownTopic", shutDownTopic);
    nh_->get_parameter("planner/plannerServiceName", plannerServiceName);
    nh_->get_parameter("planner/cleanFrontierServiceName", cleanFrontierServiceName);

    return true;
}

bool dsvplanner_ns::drrtPlanner::init()
{

    odomSub_ = nh_->create_subscription<nav_msgs::msg::Odometry>(odomSubTopic, 1,
        std::bind(&dsvplanner_ns::drrtPlanner::odomCallback, this, std::placeholders::_1));

    boundarySub_ = nh_->create_subscription<geometry_msgs::msg::PolygonStamped>(boundarySubTopic, 1,
        std::bind(&dsvplanner_ns::drrtPlanner::boundaryCallback, this, std::placeholders::_1));

    params_.newTreePathPub_ = nh_->create_publisher<visualization_msgs::msg::Marker>(newTreePathPubTopic, 1000);
    params_.remainingTreePathPub_ = nh_->create_publisher<visualization_msgs::msg::Marker>(remainingTreePathPubTopic, 1000);
    params_.boundaryPub_ = nh_->create_publisher<visualization_msgs::msg::Marker>(boundaryPubTopic, 1000);
    params_.globalSelectedFrontierPub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(globalSelectedFrontierPubTopic, 1000);
    params_.localSelectedFrontierPub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(localSelectedFrontierPubTopic, 1000);
    params_.randomSampledPointsPub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(randomSampledPointsPubTopic, 1000);
    params_.plantimePub_ = nh_->create_publisher<std_msgs::msg::Float32>(plantimePubTopic, 1000);
    params_.nextGoalPub_ = nh_->create_publisher<geometry_msgs::msg::PointStamped>(nextGoalPubTopic, 1000);
    params_.shutdownSignalPub = nh_->create_publisher<std_msgs::msg::Bool>(shutDownTopic, 1);

    // nav2_goal_Pub_ = nh_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    plannerService_ = nh_->create_service<dsvplanner::srv::Dsvplanner>(plannerServiceName, std::bind(&dsvplanner_ns::drrtPlanner::plannerServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    cleanFrontierService_ = nh_->create_service<dsvplanner::srv::CleanFrontier>(cleanFrontierServiceName, std::bind(&dsvplanner_ns::drrtPlanner::cleanFrontierServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    return true;
}
