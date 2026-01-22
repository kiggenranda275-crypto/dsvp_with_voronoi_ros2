/**************************************************************************
dual_state_fontier.cpp
Implementation of dual_state frontier detection. Detect local and global
frontiers
to guide the extension of local and global graph.

Hongbiao Zhu(hongbiaz@andrew.cmu.edu)
5/25/2020
**************************************************************************/

#include "dsvplanner/dual_state_frontier.h"

#include <misc_utils/misc_utils.h>

namespace dsvplanner_ns
{
    DualStateFrontier::DualStateFrontier(rclcpp::Node::SharedPtr& node_handle, octomanager* manager)
        : nh_(node_handle)
    {
        manager_ = manager;
        initialize();
    }

    DualStateFrontier::~DualStateFrontier()
    {
    }

    bool DualStateFrontier::readParameters() // 声明参数以及获取参数
    {
        nh_->declare_parameter("frontier/world_frame_id", world_frame_id_);
        nh_->declare_parameter("frontier/sub_graph_points_topic", sub_graph_points_topic_);
        nh_->declare_parameter("frontier/pub_unknown_points_topic", pub_unknown_points_topic_);
        nh_->declare_parameter("frontier/pub_global_frontier_points_topic", pub_global_frontier_points_topic_);
        nh_->declare_parameter("frontier/pub_local_frontier_points_topic", pub_local_frontier_points_topic_);
        nh_->declare_parameter("frontier/kExecuteFrequency", kExecuteFrequency_);
        nh_->declare_parameter("frontier/kFrontierResolution", kFrontierResolution);
        nh_->declare_parameter("frontier/kFrontierFilterSize", kFrontierFilterSize);
        nh_->declare_parameter("frontier/kSearchRadius", kSearchRadius);
        nh_->declare_parameter("frontier/kSearchBoundingX", search_bounding[0]);
        nh_->declare_parameter("frontier/kSearchBoundingY", search_bounding[1]);
        nh_->declare_parameter("frontier/kSearchBoundingZ", search_bounding[2]);
        nh_->declare_parameter("frontier/kEffectiveUnknownNumAroundFrontier", kEffectiveUnknownNumAroundFrontier);
        nh_->declare_parameter("frontier/kFrontierNeighbourSearchRadius", kFrontierNeighbourSearchRadius);
        nh_->declare_parameter("frontier/kEliminateFrontiersAroundRobots", kEliminateFrontiersAroundRobots);

        nh_->get_parameter("planner/odomSubTopic", sub_odom_topic_);
        nh_->get_parameter("planner/terrainCloudSubTopic", sub_terrain_point_cloud_topic_);
        nh_->get_parameter("frontier/world_frame_id", world_frame_id_);
        nh_->get_parameter("frontier/sub_graph_points_topic", sub_graph_points_topic_);
        nh_->get_parameter("frontier/pub_unknown_points_topic", pub_unknown_points_topic_);
        nh_->get_parameter("frontier/pub_global_frontier_points_topic", pub_global_frontier_points_topic_);
        nh_->get_parameter("frontier/pub_local_frontier_points_topic", pub_local_frontier_points_topic_);
        nh_->get_parameter("frontier/kExecuteFrequency", kExecuteFrequency_);
        nh_->get_parameter("frontier/kFrontierResolution", kFrontierResolution);
        nh_->get_parameter("frontier/kFrontierFilterSize", kFrontierFilterSize);
        nh_->get_parameter("frontier/kSearchRadius", kSearchRadius);
        nh_->get_parameter("frontier/kSearchBoundingX", search_bounding[0]);
        nh_->get_parameter("frontier/kSearchBoundingY", search_bounding[1]);
        nh_->get_parameter("frontier/kSearchBoundingZ", search_bounding[2]);
        nh_->get_parameter("frontier/kEffectiveUnknownNumAroundFrontier", kEffectiveUnknownNumAroundFrontier);
        nh_->get_parameter("frontier/kFrontierNeighbourSearchRadius", kFrontierNeighbourSearchRadius);
        nh_->get_parameter("frontier/kEliminateFrontiersAroundRobots", kEliminateFrontiersAroundRobots);
        nh_->get_parameter("gb/kMaxXGlobal", kGlobalMaxX);
        nh_->get_parameter("gb/kMaxYGlobal", kGlobalMaxY);
        nh_->get_parameter("gb/kMaxZGlobal", kGlobalMaxZ);
        nh_->get_parameter("gb/kMinXGlobal", kGlobalMinX);
        nh_->get_parameter("gb/kMinYGlobal", kGlobalMinY);
        nh_->get_parameter("gb/kMinZGlobal", kGlobalMinZ);
        nh_->get_parameter("rm/kBoundX", robot_bounding[0]);
        nh_->get_parameter("rm/kBoundY", robot_bounding[1]);
        nh_->get_parameter("rm/kBoundZ", robot_bounding[2]);
        nh_->get_parameter("rm/kSensorVertical", kSensorVerticalView);
        nh_->get_parameter("rm/kSensorHorizontal", kSensorHorizontalView);
        nh_->get_parameter("rm/kVehicleHeight", kVehicleHeight);
        nh_->get_parameter("elevation/kTerrainVoxelSize", kTerrainVoxelSize);
        nh_->get_parameter("elevation/kTerrainVoxelHalfWidth", kTerrainVoxelHalfWidth);
        nh_->get_parameter("elevation/kTerrainVoxelWidth", kTerrainVoxelWidth);

        nh_->declare_parameter("voronoi/min_z", kVoronoiMinZ);
        nh_->declare_parameter("voronoi/max_z", kVoronoiMaxZ);
        nh_->get_parameter("voronoi/min_z", kVoronoiMinZ);
        nh_->get_parameter("voronoi/max_z", kVoronoiMaxZ);
        nh_->declare_parameter("voronot/voronoi_unknown_num", kVoronoiUnknownNum);
        nh_->get_parameter("voronot/voronoi_unknown_num", kVoronoiUnknownNum);

        return true;
    }

    // 在边界框的范围内查找未知的点云；Vector2D
    void DualStateFrontier::getUnknowPointcloudInBoundingBox(const StateVec& center, const StateVec& bounding_box_size)
    {
        unknown_points_->clear();
        local_frontier_->clear();
        pcl::PointCloud<pcl::PointXYZ>::Ptr local_frontier_temp =
            pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

        const double epsilon = 0.001; // Small offset to not hit boundary of nodes.
        StateVec epsilon_3d;
        epsilon_3d.setConstant(epsilon);

        // Determine correct center of voxel.修正体素中心
        const StateVec center_corrected(
            kFrontierResolution * std::floor(center.x() / kFrontierResolution) + kFrontierResolution / 2.0,
            kFrontierResolution * std::floor(center.y() / kFrontierResolution) + kFrontierResolution / 2.0, center.z());
        StateVec bbx_min = -bounding_box_size / 2 - epsilon_3d;
        StateVec bbx_max = bounding_box_size / 2 + epsilon_3d;

        for (double x_position = bbx_min.x(); x_position <= bbx_max.x(); x_position += kFrontierResolution)
        {
            for (double y_position = bbx_min.y(); y_position <= bbx_max.y(); y_position += kFrontierResolution)
            {
                double x = center_corrected[0] + x_position;
                double y = center_corrected[1] + y_position;
                double z = getZvalue(x_position, y_position);
                if (z >= 1000)
                    continue;
                octomap::point3d point = octomap::point3d(x, y, z);
                octomap::OcTreeKey key = manager_->octree_->coordToKey(point);
                octomap::OcTreeNode* node = manager_->octree_->search(key); // 8叉树寻找
                if (node == NULL)
                {
                    unknown_points_->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
                    if (FrontierInBoundry(point) && frontierDetect(point))
                    {
                        local_frontier_temp->push_back(pcl::PointXYZ(point.x(), point.y(), point.z())); // 添加进局部边界
                    }
                }
            }
        }
        pcl::VoxelGrid<pcl::PointXYZ> point_ds;
        // 未知点云
        point_ds.setLeafSize(kFrontierFilterSize, kFrontierFilterSize, kFrontierFilterSize);
        point_ds.setInputCloud(local_frontier_temp);
        point_ds.filter(*local_frontier_);
    }

    //  函数frontierDetect 判断该点是否是前沿点
    bool DualStateFrontier::frontierDetect(octomap::point3d point) const
    {
        const double resolution = manager_->octree_->getResolution();
        bool xPositive = false, xNegative = false, yPositive = false, yNegative = false;
        bool effectiveFree = false;
        bool effectiveUnknown = false;
        int unknowCount = 0;
        octomap::OcTreeNode* node_inside;
        octomap::OcTreeNode* node_outside;
        octomap::OcTreeKey key_inside, key_outside;
        octomap::point3d surround_point_inside, surround_point_outside;
        surround_point_inside.x() = point.x();
        surround_point_inside.y() = point.y() - resolution;
        surround_point_inside.z() = point.z();
        key_inside = manager_->octree_->coordToKey(surround_point_inside);
        node_inside = manager_->octree_->search(key_inside);
        surround_point_outside.x() = point.x();
        surround_point_outside.y() = point.y() - 2 * resolution;
        surround_point_outside.z() = point.z();
        key_outside = manager_->octree_->coordToKey(surround_point_outside);
        node_outside = manager_->octree_->search(key_outside);
        if (node_inside != NULL && (!manager_->octree_->isNodeOccupied(node_inside)))
        {
            yNegative = true;
        }
        else if ((node_inside != NULL && manager_->octree_->isNodeOccupied(node_inside)))
        {
            return false;
        }
        else if (node_inside == NULL)
        {
            unknowCount++;
        }
        surround_point_inside.x() = point.x();
        surround_point_inside.y() = point.y() + resolution;
        surround_point_inside.z() = point.z();
        key_inside = manager_->octree_->coordToKey(surround_point_inside);
        node_inside = manager_->octree_->search(key_inside);
        surround_point_outside.x() = point.x();
        surround_point_outside.y() = point.y() + 2 * resolution;
        surround_point_outside.z() = point.z();
        key_outside = manager_->octree_->coordToKey(surround_point_outside);
        node_outside = manager_->octree_->search(key_outside);
        if (node_inside != NULL && (!manager_->octree_->isNodeOccupied(node_inside)))
        {
            yPositive = true;
        }
        else if ((node_inside != NULL && manager_->octree_->isNodeOccupied(node_inside)))
        {
            return false;
        }
        else if (node_inside == NULL)
        {
            unknowCount++;
        }
        surround_point_inside.x() = point.x() - resolution;
        surround_point_inside.y() = point.y();
        surround_point_inside.z() = point.z();
        key_inside = manager_->octree_->coordToKey(surround_point_inside);
        node_inside = manager_->octree_->search(key_inside);
        surround_point_outside.x() = point.x() - 2 * resolution;
        surround_point_outside.y() = point.y();
        surround_point_outside.z() = point.z();
        key_outside = manager_->octree_->coordToKey(surround_point_outside);
        node_outside = manager_->octree_->search(key_outside);
        if (node_inside != NULL && (!manager_->octree_->isNodeOccupied(node_inside)))
        {
            xNegative = true;
        }
        else if ((node_inside != NULL && manager_->octree_->isNodeOccupied(node_inside)))
        {
            return false;
        }
        else if (node_inside == NULL)
        {
            unknowCount++;
        }
        surround_point_inside.x() = point.x() + resolution;
        surround_point_inside.y() = point.y();
        surround_point_inside.z() = point.z();
        key_inside = manager_->octree_->coordToKey(surround_point_inside);
        node_inside = manager_->octree_->search(key_inside);
        surround_point_outside.x() = point.x() + 2 * resolution;
        surround_point_outside.y() = point.y();
        surround_point_outside.z() = point.z();
        key_outside = manager_->octree_->coordToKey(surround_point_outside);
        node_outside = manager_->octree_->search(key_outside);
        if (node_inside != NULL && (!manager_->octree_->isNodeOccupied(node_inside)))
        {
            xPositive = true;
        }
        else if ((node_inside != NULL && manager_->octree_->isNodeOccupied(node_inside)))
        {
            return false;
        }
        else if (node_inside == NULL)
        {
            unknowCount++;
        }
        effectiveFree = xPositive || xNegative || yPositive || yNegative;
        effectiveUnknown = unknowCount >= kEffectiveUnknownNumAroundFrontier;
        return (effectiveFree && effectiveUnknown);
    }

    // 是否在边界
    bool DualStateFrontier::FrontierInBoundry(octomap::point3d point) const
    {
        if (boundaryLoaded_)
        {
            geometry_msgs::msg::Point node_point;
            node_point.x = point.x();
            node_point.y = point.y();
            node_point.z = point.z();
            if (!misc_utils_ns::PointInPolygon(node_point, boundary_polygon_))
            {
                return false;
            }
        }
        else
        {
            if (point.x() > kGlobalMaxX)
                return false;
            else if (point.y() > kGlobalMaxY)
                return false;
            else if (point.x() < kGlobalMinX)
                return false;
            else if (point.y() < kGlobalMinY)
                return false;
        }
        if (point.z() > kGlobalMaxZ)
            return false;
        if (point.z() < kGlobalMinZ)
            return false;
        else
            return true;
    }

    void DualStateFrontier::updateToCleanFrontier(pcl::PointXYZ point)
    {
        cleanedFrontier_->points.push_back(point);
    }

    bool DualStateFrontier::isCleanedFrontier(pcl::PointXYZ point)
    {
        if (cleanedFrontier_->points.size() > 0)
        {
            for (int i = 0; i < cleanedFrontier_->points.size(); i++)
            {
                double dist = sqrt((point.x - cleanedFrontier_->points[i].x) * (point.x - cleanedFrontier_->points[i].x) +
                    (point.y - cleanedFrontier_->points[i].y) * (point.y - cleanedFrontier_->points[i].y) +
                    (point.z - cleanedFrontier_->points[i].z) * (point.x - cleanedFrontier_->points[i].z));
                if (dist < 3)
                {
                    return true;
                }
            }
        }
        return false;
    }

    bool DualStateFrontier::inSensorRangeofGraphPoints(StateVec point)
    {
        pcl::PointXYZ check_point;
        check_point.x = point[0];
        check_point.y = point[1];
        check_point.z = point[2];
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchDist;
        if (graphPoints_->points.size() > 0)
        {
            kdtree_->setInputCloud(graphPoints_);
            kdtree_->radiusSearch(check_point, kSearchRadius, pointSearchInd, pointSearchDist);
            for (int i = 0; i < pointSearchInd.size(); i++)
            {
                StateVec node_point(graphPoints_->points[pointSearchInd[i]].x, graphPoints_->points[pointSearchInd[i]].y,
                    graphPoints_->points[pointSearchInd[i]].z);
                StateVec dir = point - node_point;
                // Skip if distance is too large
                double rangeSq = pow(kSearchRadius, 2.0);
                if (dir.transpose().dot(dir) > rangeSq)
                {
                    continue;
                }
                bool insideAFieldOfView = false;
                if (fabs(dir[2] < sqrt(dir[0] * dir[0] + dir[1] * dir[1]) * tan(M_PI * kSensorVerticalView / 360)))
                {
                    insideAFieldOfView = true;
                }
                if (!insideAFieldOfView)
                {
                    continue;
                }
                if (CellStatus::kOccupied != manager_->getLineStatusBoundingBox(node_point, point, robot_bounding))
                {
                    return true;
                }
            }
            return false;
        }
        return false;
    }

    bool DualStateFrontier::inSensorRangeofRobot(StateVec point)
    {
        pcl::PointXYZ check_point;
        check_point.x = point[0];
        check_point.y = point[1];
        check_point.z = point[2];
        StateVec dir = point - robot_position_;
        // Skip if distance is too large
        double rangeSq = pow(kSearchRadius, 2.0);
        if (dir.transpose().dot(dir) > rangeSq)
        {
            return false;
        }
        bool insideAFieldOfView = false;
        if (fabs(dir[2] < sqrt(dir[0] * dir[0] + dir[1] * dir[1]) * tan(M_PI * kSensorVerticalView / 360)))
        {
            insideAFieldOfView = true;
        }
        if (!insideAFieldOfView)
        {
            return false;
        }
        if (CellStatus::kFree != manager_->getVisibility(robot_position_, point, false))
        {
            return false;
        }
        return true;
    }

    void DualStateFrontier::globalFrontiersNeighbourCheck() // global frontier有邻域值
    {
        global_frontier_pcl_->clear();

        int size = global_frontier_->points.size();
        pcl::PointXYZ p1;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchDist;
        if (size > 0)
        {
            global_frontiers_kdtree_->setInputCloud(global_frontier_);
            for (int i = 0; i < size; i++)
            {
                p1 = global_frontier_->points[i];
                pointSearchInd.clear();
                pointSearchDist.clear();
                global_frontiers_kdtree_->radiusSearch(p1, kFrontierNeighbourSearchRadius, pointSearchInd, pointSearchDist);
                if (pointSearchInd.size() > 1)
                    global_frontier_pcl_->points.push_back(p1);
            }
        }
        global_frontier_->clear();
        *global_frontier_ = *global_frontier_pcl_;
    }

    void DualStateFrontier::cleanAllUselessFrontiers()
    {
        global_frontier_->clear();
        local_frontier_->clear();
        publishFrontiers();
    }

    void DualStateFrontier::getFrontiers()
    {
        getUnknowPointcloudInBoundingBox(robot_position_, search_bounding);
        localFrontierUpdate(robot_position_);
        gloabalFrontierUpdate();
        globalFrontiersNeighbourCheck();
        publishFrontiers();
    }

    void DualStateFrontier::publishFrontiers()
    {
        sensor_msgs::msg::PointCloud2 unknown_pcl, local_frontier_pcl, global_frontier_pcl;
        pcl::toROSMsg(*unknown_points_, unknown_pcl);
        pcl::toROSMsg(*global_frontier_, global_frontier_pcl);
        pcl::toROSMsg(*local_frontier_, local_frontier_pcl);
        unknown_pcl.header.frame_id = world_frame_id_;
        global_frontier_pcl.header.frame_id = world_frame_id_;
        local_frontier_pcl.header.frame_id = world_frame_id_;
        unknown_points_pub_->publish(unknown_pcl);
        global_frontier_points_pub_->publish(global_frontier_pcl);
        local_frontier_points_pub_->publish(local_frontier_pcl);
    }

    void DualStateFrontier::terrainCloudAndOdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrain_msg)
    {
        robot_position_[0] = odom_msg->pose.pose.position.x;
        robot_position_[1] = odom_msg->pose.pose.position.y;
        robot_position_[2] = odom_msg->pose.pose.position.z;
        terrain_cloud_->clear();
        terrain_cloud_ds->clear();
        terrain_elev_cloud_->clear();
        terrain_voxel_points_num_.clear();
        terrain_voxel_min_elev_.clear();
        terrain_voxel_max_elev_.clear();
        terrain_voxel_points_num_.resize(kTerrainVoxelWidth * kTerrainVoxelWidth);
        terrain_voxel_min_elev_.resize(kTerrainVoxelWidth * kTerrainVoxelWidth, 1000);
        terrain_voxel_max_elev_.resize(kTerrainVoxelWidth * kTerrainVoxelWidth, -1000);
        pcl::fromROSMsg(*terrain_msg, *terrain_cloud_);

        pcl::VoxelGrid<pcl::PointXYZI> point_ds;
        point_ds.setLeafSize(0.3, 0.3, 0.3);
        point_ds.setInputCloud(terrain_cloud_);
        point_ds.filter(*terrain_cloud_ds);

        updateTerrainMinElevation();
        updateTerrainElevationForKnown();
        updateTerrainElevationForUnknow();

        sensor_msgs::msg::PointCloud2 elevVoxel2;
        pcl::toROSMsg(*terrain_elev_cloud_, elevVoxel2);
        elevVoxel2.header.stamp = nh_->now();
        elevVoxel2.header.frame_id = "map";
        terrain_elev_cloud_pub_->publish(elevVoxel2);
    }

    void DualStateFrontier::updateTerrainMinElevation()
    {
        pcl::PointXYZI point;
        int terrainCloudSize = terrain_cloud_ds->points.size();
        for (int i = 0; i < terrainCloudSize; i++)
        {
            point.x = terrain_cloud_ds->points[i].x;
            point.y = terrain_cloud_ds->points[i].y;
            point.z = terrain_cloud_ds->points[i].z;
            point.intensity = terrain_cloud_ds->points[i].intensity;
            int indX = int((point.x - robot_position_[0] + kTerrainVoxelSize / 2) / kTerrainVoxelSize) + kTerrainVoxelHalfWidth;
            int indY = int((point.y - robot_position_[1] + kTerrainVoxelSize / 2) / kTerrainVoxelSize) + kTerrainVoxelHalfWidth;
            if (point.x - robot_position_[0] + kTerrainVoxelSize / 2 < 0)
                indX--;
            if (point.y - robot_position_[1] + kTerrainVoxelSize / 2 < 0)
                indY--;
            if (indX > kTerrainVoxelWidth - 1)
                indX = kTerrainVoxelWidth - 1;
            if (indX < 0)
                indX = 0;
            if (indY > kTerrainVoxelWidth - 1)
                indY = kTerrainVoxelWidth - 1;
            if (indY < 0)
                indY = 0;
            int indVoxel = kTerrainVoxelWidth * indX + indY;

            terrain_voxel_points_num_[indVoxel]++;
            if (point.z < terrain_voxel_min_elev_[indVoxel])
                terrain_voxel_min_elev_[indVoxel] = point.z;
            if (point.z > terrain_voxel_max_elev_[indVoxel])
                terrain_voxel_max_elev_[indVoxel] = point.z;
            for (int dX = -1; dX <= 1; dX = dX + 2)
            {
                for (int dY = -1; dY <= 1; dY = dY + 2)
                {
                    if (indX + dX >= 0 && indX + dX < kTerrainVoxelWidth && indY + dY >= 0 && indY + dY < kTerrainVoxelWidth)
                    {
                        terrain_voxel_points_num_[kTerrainVoxelWidth * (indX + dX) + indY + dY]++;
                        if (point.z < terrain_voxel_min_elev_[kTerrainVoxelWidth * (indX + dX) + indY + dY])
                            terrain_voxel_min_elev_[kTerrainVoxelWidth * (indX + dX) + indY + dY] = point.z;
                    }
                }
            }
        }
    }

    void DualStateFrontier::updateTerrainElevationForKnown()
    {
        pcl::PointXYZI point;
        for (int i = 0; i < kTerrainVoxelWidth * kTerrainVoxelWidth; i++)
        {
            if (terrain_voxel_points_num_[i] > 0)
            {
                if (terrain_voxel_max_elev_[i] - terrain_voxel_min_elev_[i] >= 0.4)
                    terrain_voxel_elev_[i] = 1000; // set a high value to untraversable
                // voxel
                else
                    terrain_voxel_elev_[i] = terrain_voxel_min_elev_[i];

                int indX = int(i / kTerrainVoxelWidth);
                int indY = i % kTerrainVoxelWidth;
                if (indX - kTerrainVoxelHalfWidth < 0)
                {
                    indX++;
                }
                if (indY - kTerrainVoxelWidth < 0)
                {
                    indY++;
                }
                point.x = (indX - kTerrainVoxelHalfWidth) * kTerrainVoxelSize - kTerrainVoxelSize / 2 + robot_position_[0];
                point.y = (indY - kTerrainVoxelHalfWidth) * kTerrainVoxelSize - kTerrainVoxelSize / 2 + robot_position_[1];
                point.z = 0;
                point.intensity = terrain_voxel_elev_[i];
                terrain_elev_cloud_->push_back(point);
            }
        }
    }

    void DualStateFrontier::updateTerrainElevationForUnknow()
    {
        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        kdtree.setInputCloud(terrain_elev_cloud_);

        pcl::PointXYZI point;
        if (terrain_voxel_points_num_[0] <= 0)
        {
            terrain_voxel_elev_[0] = robot_position_[2] - kVehicleHeight;
        }
        for (int i = 1; i < kTerrainVoxelWidth * kTerrainVoxelWidth; i++)
        {
            if (terrain_voxel_points_num_[i] <= 0)
            {
                int indX = int(i / kTerrainVoxelWidth);
                int indY = i % kTerrainVoxelWidth;
                point.x = (indX - kTerrainVoxelHalfWidth) * kTerrainVoxelSize - kTerrainVoxelSize / 2 + robot_position_[0];
                point.y = (indY - kTerrainVoxelHalfWidth) * kTerrainVoxelSize - kTerrainVoxelSize / 2 + robot_position_[1];
                point.z = 0;

                if (terrain_elev_cloud_->points.size() > 0)
                {
                    if (kdtree.nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
                    {
                        point.intensity = terrain_elev_cloud_->points[pointIdxNKNSearch[0]].intensity;
                        terrain_voxel_elev_[i] = point.intensity;
                    }
                    terrain_elev_cloud_->push_back(point);
                }
            }
        }
    }

    void DualStateFrontier::graphPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr graph_msg)
    {
        graphPoints_->clear();
        pcl::fromROSMsg(*graph_msg, *graphPoints_);
    }

    double DualStateFrontier::getZvalue(double x_position, double y_position)
    {
        int indX = int((x_position + kTerrainVoxelSize / 2) / kTerrainVoxelSize) + kTerrainVoxelHalfWidth;
        int indY = int((y_position + kTerrainVoxelSize / 2) / kTerrainVoxelSize) + kTerrainVoxelHalfWidth;
        if (x_position + kTerrainVoxelSize / 2 < 0)
            indX--;
        if (y_position + kTerrainVoxelSize / 2 < 0)
            indY--;
        if (indX > kTerrainVoxelWidth - 1)
            indX = kTerrainVoxelWidth - 1;
        if (indX < 0)
            indX = 0;
        if (indY > kTerrainVoxelWidth - 1)
            indY = kTerrainVoxelWidth - 1;
        if (indY < 0)
            indY = 0;
        return terrain_voxel_elev_[kTerrainVoxelWidth * indX + indY] + kVehicleHeight;
    }

    std::vector<double> DualStateFrontier::getTerrainVoxelElev()
    {
        return terrain_voxel_elev_;
    }

    void DualStateFrontier::setPlannerStatus(bool status)
    {
        planner_status_ = status;
    }

    void DualStateFrontier::setBoundary(const geometry_msgs::msg::PolygonStamped& boundary)
    {
        boundary_polygon_ = boundary.polygon;
        boundaryLoaded_ = true;
    }

    bool DualStateFrontier::initialize()
    {
        // Read in parameters
        if (!readParameters())
            return false;

        // Initialize subscriber
        graph_points_sub_ = nh_->create_subscription<sensor_msgs::msg::PointCloud2>(sub_graph_points_topic_, 1,
            std::bind(&DualStateFrontier::graphPointsCallback, this, std::placeholders::_1));

        odom_sub_.subscribe(nh_, sub_odom_topic_, qos_profile);
        terrain_point_cloud_sub_.subscribe(nh_, sub_terrain_point_cloud_topic_, qos_profile);
        sync_.reset(new Sync(syncPolicy(100), odom_sub_, terrain_point_cloud_sub_));
        sync_->registerCallback(std::bind(&DualStateFrontier::terrainCloudAndOdomCallback, this, std::placeholders::_1, std::placeholders::_2));

        unknown_points_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(pub_unknown_points_topic_, 1);
        global_frontier_points_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(pub_global_frontier_points_topic_, 1);
        local_frontier_points_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(pub_local_frontier_points_topic_, 1);
        terrain_elev_cloud_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("/elevation_map", 1);

        // TODO
        voronoi_local_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("/voronoi_local_points", 10);
        voronoi_global_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("/voronoi_global_points", 10);

        if (kExecuteFrequency_ > 0.0)
        {
            executeTimer_ = nh_->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DualStateFrontier::execute, this));
        }

        for (int i = 0; i < kTerrainVoxelWidth * kTerrainVoxelWidth; i++)
        {
            terrain_voxel_elev_.push_back(robot_position_.z());
            terrain_voxel_points_num_.push_back(0);
            terrain_voxel_min_elev_.push_back(1000);
            terrain_voxel_max_elev_.push_back(-1000);
        }

        boundaryLoaded_ = false;

        voronoi_local_ = std::make_shared<Voronoi>("/local_voronoi");   // 局部Voronoi
        voronoi_global_ = std::make_shared<Voronoi>("/global_voronoi"); // 全局Voronoi

        RCLCPP_INFO(nh_->get_logger(), "Successfully launched DualStateFrontier node");
        global_forniter_timer_ = nh_->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&DualStateFrontier::globalVoronnoiFrontierExecuter, this));

        return true;
    }

    // 计算全局Voronoi边界
    void DualStateFrontier::globalVoronnoiFrontierExecuter()
    {
        if (manager_->occupancy_map->data.size() != 0)
        {
            getGlobalVoronoiForntier();
            *global_frontier_ = *voronoi_global_frontier;
            // gloabalFrontierUpdate();
            // globalFrontiersNeighbourCheck();
            // RCLCPP_INFO(nh_->get_logger(), "Global frontier size: %d", global_frontier_->points.size());
        }
        else
            return;
    }

    // 循环0.1s执行，查找边界
    void DualStateFrontier::execute()
    {
        if (manager_->occupancy_map->data.size() != 0)
        {
            getLoaclVoronoiForntier();
            *local_frontier_ = *voronoi_local_forntier;
            // localFrontierUpdate(robot_position_);
            publishFrontiers();
        }
        else
            return;
    }

    // 获取全局边界
    void DualStateFrontier::getGlobalVoronoiForntier()
    {
        // 全局占据栅格地图转化为bool_map
        std::vector<Eigen::Vector3i> edges;
        std::vector<std::pair<int, int>> paths;
        std::vector<std::vector<bool>> bool_map;
        int width = manager_->global_map_[0].size();
        int height = manager_->global_map_.size();
        if (width == 0 || height == 0)
            return;
        bool_map.resize(height);
        for (int i = 0; i < height; i++)
        {
            bool_map[i].resize(width);
            for (int j = 0; j < width; j++)
            {
                if (manager_->global_map_[i][j] == 0)
                    bool_map[i][j] = false;
                else
                    bool_map[i][j] = true;
            }
        }
        if (bool_map.size() > 0)
        {
            voronoi_global_->setMap(bool_map);
            voronoi_global_->update_voronoi(false);
            voronoi_global_->visualize_publish();
            voronoi_global_->getVoronoiEdges(edges, paths);
            voronoi_global_frontier = VoronoiEdgesToFrontier(edges, false);

            sensor_msgs::msg::PointCloud2 voronoi_pcl;
            pcl::toROSMsg(*voronoi_global_frontier, voronoi_pcl);
            voronoi_pcl.header.frame_id = world_frame_id_;
            voronoi_global_pub_->publish(voronoi_pcl);
        }
    }

    void DualStateFrontier::getLoaclVoronoiForntier()
    {
        int local_num = manager_->local_map_.size();
        if (local_num == 0)
            return;
        std::vector<std::vector<bool>> local_bool_map(local_num, std::vector<bool>(local_num, false));
        for (int i = 0; i < local_num; i++)
        {
            for (int j = 0; j < local_num; j++)
            {
                if (manager_->local_map_[i][j] == 100) // 100 表示占用
                    local_bool_map[i][j] = true;
                else if (manager_->local_map_[i][j] == 0) // 0 表示空闲
                    local_bool_map[i][j] = false;
                else if (manager_->local_map_[i][j] == -1) // -1 表示未知
                    local_bool_map[i][j] = true;
            }
        }
        std::vector<Eigen::Vector3i> edges;
        std::vector<std::pair<int, int>> paths;
        if (local_bool_map.size() > 0)
        {
            voronoi_local_->setMap(local_bool_map);
            voronoi_local_->update_voronoi(false);
            voronoi_local_->visualize_publish();
            voronoi_local_->getVoronoiEdges(edges, paths);
            voronoi_local_forntier = VoronoiEdgesToFrontier(edges, true);

            sensor_msgs::msg::PointCloud2 voronoi_pcl;
            pcl::toROSMsg(*voronoi_local_forntier, voronoi_pcl);
            voronoi_pcl.header.frame_id = world_frame_id_;
            voronoi_local_pub_->publish(voronoi_pcl);
        }
    }

    // 将数组voronoi_edges转为pcl点云,并且滤除超过的点
    pcl::PointCloud<pcl::PointXYZ>::Ptr DualStateFrontier::VoronoiEdgesToFrontier(std::vector<Eigen::Vector3i>& edges, bool is_local)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr frontier = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        if (edges.size() == 0)
            return nullptr;
        std::vector<std::vector<int>> map;
        if (is_local)
            map = manager_->local_map_;
        else
            map = manager_->global_map_;

        // 遍历edges节点
        for (int i = 0; i < edges.size(); i++)
        {
            std::vector<int> result;

            int k = (int)sqrt(edges[i].z()); // voronoi边界点的半径
            if (k > 4)
                continue;
            int x = edges[i].x();
            int y = edges[i].y();
            int rows = map.size();
            int cols = (rows > 0) ? map[0].size() : 0;
            // RCLCPP_INFO(nh_->get_logger(), "voronoi edges radius %d %d", k, edges[i].z());
            if (rows == 0 || cols == 0 || x < 0 || x >= rows || y < 0 || y >= cols)
            {
                continue;
            }
            // 半径范围内查找
            int start_row = std::max(0, x - k);
            int end_row = std::min(rows - 1, x + k);
            int start_col = std::max(0, y - k);
            int end_col = std::min(cols - 1, y + k);
            int occupancy_num = 0, unknown_num = 0, free_num = 0;
            for (int di = start_row; di <= end_row; ++di)
            {
                for (int dj = start_col; dj <= end_col; ++dj)
                {
                    if (map[di][dj] == 0)
                        free_num++;
                    else if (map[di][dj] == 100)
                        occupancy_num++;
                    else if (map[di][dj] == -1)
                        unknown_num++;
                }
            }
            if (unknown_num > 0) //occupancy_num > 0 && && free_num > 0
            {
                Eigen::Vector3d edge_point;

                edge_point = getRealPosition(x, y, is_local);

                pcl::PointXYZ point;
                point.x = edge_point.x();
                point.y = edge_point.y();
                point.z = getZvalue(edge_point.x(), edge_point.y());

                if (point.z > 1.0)
                    continue;

                frontier->points.push_back(point);
            }
        }
        // RCLCPP_INFO(nh_->get_logger(), "Voronoi edges frontier size: %d", frontier->points.size());
        return frontier;
    }

    // 局部边界更新
    void DualStateFrontier::localFrontierUpdate(StateVec& center)
    {
        local_frontier_pcl_->clear();
        StateVec checkedPoint;

        for (int i = 0; i < local_frontier_->size(); i++)
        {
            int sqdist = local_frontier_->points[i].z;
            checkedPoint.x() = local_frontier_->points[i].x;
            checkedPoint.y() = local_frontier_->points[i].y;
            checkedPoint.z() = getZvalue(checkedPoint.x(), checkedPoint.y());
            local_frontier_->points[i].z = checkedPoint.z();
            // 原来的局部边界，如果不在机器人的传感器范围内，且不在机器人周围的范围内，且不在地图中被占用，或者在图中的点附近
            // 在机器人传感器范围内；AND ( (当前点和目前点之间没有被阻挡，可以被看见 AND 没有被地形阻挡) OR 在探索阶段并在地图的范围内 ）
            if (!(kEliminateFrontiersAroundRobots && inSensorRangeofRobot(checkedPoint)) &&
                ((CellStatus::kOccupied != manager_->getVisibility(center, checkedPoint, false)) ||
                    (!planner_status_ && inSensorRangeofGraphPoints(checkedPoint))))
            {
                local_frontier_pcl_->points.push_back(local_frontier_->points[i]);
            }
        }
        // 降采样后重新赋值
        local_frontier_->clear();
        pcl::VoxelGrid<pcl::PointXYZ> point_ds;
        point_ds.setLeafSize(kFrontierFilterSize, kFrontierFilterSize, kFrontierFilterSize);
        point_ds.setInputCloud(local_frontier_pcl_);
        point_ds.filter(*local_frontier_);
    }

    // 全局边界更新,遍历以前的边界点，如果已经被清理，则跳过，如果没有被清理，再次判断是否是边界点
    void DualStateFrontier::gloabalFrontierUpdate()
    {
        global_frontier_pcl_->clear();
        int size = global_frontier_->points.size();
        StateVec checkedPoint;
        for (int i = 0; i < size; i++)
        {
            int sqdist = global_frontier_->points[i].z;
            checkedPoint.x() = global_frontier_->points[i].x;
            checkedPoint.y() = global_frontier_->points[i].y;
            checkedPoint.z() = getZvalue(checkedPoint.x(), checkedPoint.y());
            global_frontier_->points[i].z = checkedPoint.z();

            if (isCleanedFrontier(global_frontier_->points[i]))
            {
                continue;
            }
            octomap::point3d point(checkedPoint.x(), checkedPoint.y(), checkedPoint.z());
            if (frontierDetect(point)) // 检查是否是边界点
            {
                global_frontier_pcl_->points.push_back(global_frontier_->points[i]);
            }
        }
        global_frontier_->clear();
        pcl::VoxelGrid<pcl::PointXYZ> point_ds_;
        point_ds_.setLeafSize(kFrontierFilterSize, kFrontierFilterSize, kFrontierFilterSize);
        point_ds_.setInputCloud(global_frontier_pcl_);
        point_ds_.filter(*global_frontier_);
    }

    Eigen::Vector3d DualStateFrontier::getRealPosition(int i, int j, bool is_local)
    {
        double origin_x, origin_y;
        int col, row;
        const double resolution = manager_->occupancy_map->info.resolution;
        if (is_local)
        {
            origin_x = manager_->robot_pos.x;
            origin_y = manager_->robot_pos.y;
            int k = (manager_->local_map_.size()) / 2;
            col = i - k;
            row = j - k; // 中心点
        }
        else
        {
            origin_x = manager_->occupancy_map->info.origin.position.x;
            origin_y = manager_->occupancy_map->info.origin.position.y;
            col = i;
            row = j;
        }

        // 计算真实世界中的坐标 (x, y)
        double y = origin_y + col * resolution;
        double x = origin_x + row * resolution;
        Eigen::Vector3d position;
        position.x() = x;
        position.y() = y;
        position.z() = getZvalue(x, y);
        return position;
    }

    void DualStateFrontier::get2Dindex(StateVec point, int& i, int& j, bool is_local) // [x][y]
    {
        const double resolution = manager_->occupancy_map->info.resolution;
        double origin_x;
        double origin_y;
        std::vector<std::vector<int>>
            map;
        if (is_local)
        {
            map = manager_->local_map_;
            origin_x = robot_position_.x();
            origin_y = robot_position_.y();
        }
        else
        {
            map = manager_->global_map_;
            origin_x = 0;
            origin_y = 0;
        }
        int height = map.size();
        int width = map[0].size();

        // 计算真实世界中的坐标 (x, y)
        // 计算相对于原点的偏移量
        double delta_x = point.x() - origin_x;
        double delta_y = point.y() - origin_y;

        j = static_cast<int>(std::floor(delta_x / resolution)); // 列索引
        i = static_cast<int>(std::floor(delta_y / resolution)); // 行索引

        if (i < 0 || i >= height || j < 0 || j >= width) // 错误值
        {
            i = -1;
            j = -1;
        }
    }

    bool DualStateFrontier::IsPointFrontier(StateVec point, bool is_local)
    {
        double dis;
        double resolution = manager_->getResolution();
        if (is_local)
        {
            for (auto& pt : local_frontier_->points)
            {
                dis = sqrt((point[0] - pt.x) * (point[0] - pt.x) + (point[1] - pt.y) * (point[1] - pt.y));
                if (dis < resolution)
                    return true;
            }
        }
        else
        {
            for (auto& pt : global_frontier_->points)
            {
                dis = sqrt((point[0] - pt.x) * (point[0] - pt.x) + (point[1] - pt.y) * (point[1] - pt.y));
                if (dis < resolution)
                    return true;
            }
        }
        return false;
    }
}
