#ifndef OCTO_HPP
#define OCTO_HPP

#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "octomap/octomap.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include <Eigen/Core>
#include <vector>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "sensor_msgs/msg/image.hpp"

namespace dsvplanner_ns
{
    enum CellStatus
    {
        kFree = 0,
        kOccupied = 1,
        kUnknown = 2
    };

    typedef Eigen::Vector3d StateVec;
    class octomanager
    {
    public:
        rclcpp::Node::SharedPtr nh_;
        rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

        octomap::OcTree *octree_;
        nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_map;
        nav_msgs::msg::OccupancyGrid::SharedPtr local_occupancy_map;
        std::vector<std::vector<int>> local_map_;
        std::vector<std::vector<int>> global_map_;
        sensor_msgs::msg::Image grid_image_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_map_pub_;
        geometry_msgs::msg::Point robot_pos;
        const double kradius;

    public:
        octomanager(rclcpp::Node::SharedPtr &node_handle) : nh_(node_handle), kradius(30.0)
        {
            map_sub_ = nh_->create_subscription<nav_msgs::msg::OccupancyGrid>("/projected_map", 10,
                                                                              std::bind(&octomanager::OccupancyMapCallback, this, std::placeholders::_1));
            octomap_sub_ = nh_->create_subscription<octomap_msgs::msg::Octomap>("/octomap_full_octo", 10,
                                                                                std::bind(&octomanager::OctomapCallback, this, std::placeholders::_1));
            occupancy_map = nav_msgs::msg::OccupancyGrid::SharedPtr(new nav_msgs::msg::OccupancyGrid());
            local_occupancy_map = nav_msgs::msg::OccupancyGrid::SharedPtr(new nav_msgs::msg::OccupancyGrid());
            image_pub_ = nh_->create_publisher<sensor_msgs::msg::Image>("/local_grid_image", 10);
            odom_sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg)
                                                                          {
                // RCLCPP_INFO(nh_->get_logger(), "odom_sub_");
                robot_pos = msg->pose.pose.position; });
            local_map_pub_ = nh_->create_publisher<nav_msgs::msg::OccupancyGrid>("/local_map", 10);
        }
        ~octomanager()
        {
            delete octree_;
        }

        void OccupancyMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
        {
            *occupancy_map = *map;

            global_map_ = get_global_map();
            local_map_ = get_local_map(robot_pos);
            pub2DloacalMap();
            get_loacl_occupancy_grid();
        }

        void OctomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
        {
            // RCLCPP_INFO(nh_->get_logger(), "OctomapCallback");
            auto *octree = (octomap_msgs::fullMsgToMap(*msg));
            octree_ = dynamic_cast<octomap::OcTree *>(octree);
        }

        CellStatus getVisibility(const StateVec &view_point, const StateVec &voxel_to_test,
                                 bool stop_at_unknown_cell) const
        {
            // Get all node keys for this line.
            // This is actually a typedef for a vector of OcTreeKeys.
            // Can't use the key_ray_ temp member here because this is a const function.
            octomap::KeyRay key_ray;

            octree_->computeRayKeys(pointEigenToOctomap(view_point), pointEigenToOctomap(voxel_to_test), key_ray);

            const octomap::OcTreeKey &voxel_to_test_key = octree_->coordToKey(pointEigenToOctomap(voxel_to_test));

            // Now check if there are any unknown or occupied nodes in the ray,
            // except for the voxel_to_test key.
            for (octomap::OcTreeKey key : key_ray)
            {
                if (key != voxel_to_test_key)
                {
                    octomap::OcTreeNode *node = octree_->search(key);
                    if (node == NULL)
                    {
                        if (stop_at_unknown_cell)
                        {
                            return CellStatus::kUnknown;
                        }
                    }
                    else if (octree_->isNodeOccupied(node))
                    {
                        return CellStatus::kOccupied;
                    }
                }
            }
            return CellStatus::kFree;
        }

        octomap::point3d pointEigenToOctomap(const Eigen::Vector3d &point) const
        {
            return octomap::point3d(point.x(), point.y(), point.z());
        }
        CellStatus getLineStatus(const StateVec &start, const StateVec &end) const
        {
            // Get all node keys for this line.
            // This is actually a typedef for a vector of OcTreeKeys.
            // Can't use the key_ray_ temp member here because this is a const function.
            octomap::KeyRay key_ray;
            octree_->computeRayKeys(pointEigenToOctomap(start), pointEigenToOctomap(end), key_ray);

            // Now check if there are any unknown or occupied nodes in the ray.
            for (octomap::OcTreeKey key : key_ray)
            {
                octomap::OcTreeNode *node = octree_->search(key);
                if (node == NULL)
                {
                    return CellStatus::kUnknown;
                }
                else if (octree_->isNodeOccupied(node))
                {
                    return CellStatus::kOccupied;
                }
            }
            return CellStatus::kFree;
        }
        CellStatus getLineStatusBoundingBox(const StateVec &start, const StateVec &end,
                                            const StateVec &bounding_box_size) const
        {
            // TODO(helenol): Probably best way would be to get all the coordinates along
            // the line, then make a set of all the OcTreeKeys in all the bounding boxes
            // around the nodes... and then just go through and query once.
            const double epsilon = 0.001; // Small offset
            CellStatus ret = CellStatus::kFree;
            const double &resolution = getResolution();

            // Check corner connections and depending on resolution also interior:
            // Discretization step is smaller than the octomap resolution, as this way
            // no cell can possibly be missed
            double x_disc = bounding_box_size.x() / ceil((bounding_box_size.x() + epsilon) / resolution);
            double y_disc = bounding_box_size.y() / ceil((bounding_box_size.y() + epsilon) / resolution);
            double z_disc = bounding_box_size.z() / ceil((bounding_box_size.z() + epsilon) / resolution);

            // Ensure that resolution is not infinit
            if (x_disc <= 0.0)
                x_disc = 1.0;
            if (y_disc <= 0.0)
                y_disc = 1.0;
            if (z_disc <= 0.0)
                z_disc = 1.0;

            const StateVec bounding_box_half_size = bounding_box_size * 0.5;

            for (double x = -bounding_box_half_size.x(); x <= bounding_box_half_size.x(); x += x_disc)
            {
                for (double y = -bounding_box_half_size.y(); y <= bounding_box_half_size.y(); y += y_disc)
                {
                    for (double z = -bounding_box_half_size.z(); z <= bounding_box_half_size.z(); z += z_disc)
                    {
                        StateVec offset(x, y, z);
                        ret = getLineStatus(start + offset, end + offset);
                        if (ret != CellStatus::kFree)
                        {
                            return ret;
                        }
                    }
                }
            }
            return CellStatus::kFree;
        }

        double getResolution() const
        {
            return octree_->getResolution();
        }
        octomap::point3d pointEigenToOctomap(const Eigen::Vector3d &point)
        {
            return octomap::point3d(point.x(), point.y(), point.z());
        }

        CellStatus getCellStatusPoint(const StateVec &point) const
        {
            octomap::OcTreeNode *node = octree_->search(point.x(), point.y(), point.z());
            if (node == NULL)
            {
                return CellStatus::kUnknown;
            }
            else if (octree_->isNodeOccupied(node))
            {
                return CellStatus::kOccupied;
            }
            else
            {
                return CellStatus::kFree;
            }
        }
        double getSensorMaxRange() const
        {
            return 15.0;
        }

        CellStatus getCellProbabilityPoint(const StateVec &point, double *probability) const
        {
            octomap::OcTreeNode *node = octree_->search(point.x(), point.y(), point.z());
            if (node == NULL)
            {
                if (probability)
                {
                    *probability = -1.0;
                }
                return CellStatus::kUnknown;
            }
            else
            {
                if (probability)
                {
                    *probability = node->getOccupancy();
                }
                if (octree_->isNodeOccupied(node))
                {
                    return CellStatus::kOccupied;
                }
                else
                {
                    return CellStatus::kFree;
                }
            }
        }
        void get_loacl_occupancy_grid()
        {
            double grid_resolution = occupancy_map->info.resolution;
            int k = static_cast<int>(std::floor(kradius / grid_resolution)) + 1;
            k = std::max(k, 1);
            int32_t center_col = std::floor((robot_pos.x - occupancy_map->info.origin.position.x) / grid_resolution);
            int32_t center_row = std::floor((robot_pos.y - occupancy_map->info.origin.position.y) / grid_resolution);

            // 计算局部地图的起始索引
            int32_t start_col = center_col - (k - 1) / 2;
            int32_t start_row = center_row - (k - 1) / 2;
            local_occupancy_map->info.origin = occupancy_map->info.origin;
            local_occupancy_map->info.resolution = occupancy_map->info.resolution;
            local_occupancy_map->info.width = k;
            local_occupancy_map->info.height = k;
            double local_origin_x = occupancy_map->info.origin.position.x + start_col * grid_resolution;
            double local_origin_y = occupancy_map->info.origin.position.y + start_row * grid_resolution;
            // double local_origin_x = robot_pos.x;
            // double local_origin_y = robot_pos.y;
            local_occupancy_map->info.origin.position.x = local_origin_x;
            local_occupancy_map->info.origin.position.y = local_origin_y;

            // 初始化数据数组
            local_occupancy_map->data.resize(k * k, 0);

            // 填充局部地图数据
            for (int i = 0; i < k; ++i)
            {
                for (int j = 0; j < k; ++j)
                {
                    int global_col = start_col + j;
                    int global_row = start_row + i;

                    if (global_col >= 0 && global_col < occupancy_map->info.width &&
                        global_row >= 0 && global_row < occupancy_map->info.height)
                    {
                        size_t index = global_row * occupancy_map->info.width + global_col;
                        local_occupancy_map->data[i * k + j] = occupancy_map->data[index];
                    }
                }
            }
            local_occupancy_map->header = occupancy_map->header;
            local_map_pub_->publish(*local_occupancy_map);
        }

        std::vector<std::vector<int>> get_local_map(const geometry_msgs::msg::Point robot_pos)
        {
            if (!occupancy_map)
            {
                RCLCPP_WARN(rclcpp::get_logger("get_local_map"), "Invalid input pointers");
                return {};
            }

            // 计算所需网格尺寸
            double grid_resolution = occupancy_map->info.resolution;
            int k = static_cast<int>(std::floor(kradius / grid_resolution)) + 1;
            k = std::max(k, 1); // 保证至少1x1的地图

            // 计算机器人在全局地图中的网格索引
            int32_t center_col = std::floor((robot_pos.x - occupancy_map->info.origin.position.x) / grid_resolution);
            int32_t center_row = std::floor((robot_pos.y - occupancy_map->info.origin.position.y) / grid_resolution);

            // 构造局部地图数组，初始化为0
            std::vector<std::vector<int>> local_map(k, std::vector<int>(k, 0));

            // 填充局部地图
            for (int i = 0; i < k; ++i)
            {
                for (int j = 0; j < k; ++j)
                {
                    int delta_col = j - (k / 2);
                    int actual_col = center_col + delta_col;

                    int delta_row = i - (k / 2);
                    int actual_row = center_row + delta_row;

                    // 检查索引有效性
                    if (actual_col >= 0 && actual_col < occupancy_map->info.width &&
                        actual_row >= 0 && actual_row < occupancy_map->info.height)
                    {
                        size_t index = actual_row * occupancy_map->info.width + actual_col;
                        local_map[i][j] = occupancy_map->data[index];
                    }
                }
            }
            return local_map;
        }

        std::vector<std::vector<int>> get_global_map()
        {
            int width = occupancy_map->info.width;
            int height = occupancy_map->info.height;
            if (width == 0 || height == 0)
                return {};
            std::vector<std::vector<int>> map(height, std::vector<int>(width, -1)); // 初始化为 -1（未知）

            // 遍历占用网格数据，并填充二维数组
            for (unsigned int i = 0; i < height; ++i)
            {
                for (unsigned int j = 0; j < width; ++j)
                {
                    unsigned int index = i * width + j;
                    int occupancy_value = occupancy_map->data[index];
                    if (occupancy_value == -1)
                    {
                        map[i][j] = -1; // -1 表示未知
                    }
                    else if (occupancy_value == 0)
                    {
                        map[i][j] = 0; // 0 表示空闲
                    }
                    else if (occupancy_value == 100)
                    {
                        map[i][j] = 100; // 100 表示占用
                    }
                    else
                    {
                        map[i][j] = -1; // 如果值不是 0 或 100，则认为是未知
                    }
                }
            }
            return map;
        }

        void pub2DloacalMap()
        {
            if (local_map_.empty() || local_map_[0].empty())
            {
                RCLCPP_WARN(nh_->get_logger(), "Grid state is empty. Cannot publish image.");
                return;
            }

            size_t rows = local_map_.size();
            size_t cols = local_map_[0].size();
            // 将二维向量展平为一维，并确保像素值在 0-255 之间
            std::vector<uint8_t> data;
            data.reserve(rows * cols);
            for (const auto &row : local_map_)
            {
                for (const auto &pixel : row)
                {
                    // 根据需要调整像素值映射，这里假设像素值已经在 0-255 范围内
                    uint8_t val;
                    if (pixel == 0)
                        val = 255;
                    else if (pixel == 100)
                        val = 0;
                    else if (pixel == -1)
                        val = 125;
                    else
                        val = 50;
                    data.push_back(val);
                }
            }

            // 创建 Image 消息
            auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
            image_msg->header.stamp = nh_->get_clock()->now(); // 设置时间戳
            image_msg->height = static_cast<uint32_t>(rows);
            image_msg->width = static_cast<uint32_t>(cols);
            image_msg->encoding = "mono8"; // 灰度图像编码
            image_msg->is_bigendian = 0;
            image_msg->step = static_cast<uint32_t>(cols); // 每行字节数
            image_msg->data = data;
            grid_image_ = *image_msg;
            image_pub_->publish(*image_msg);
        }
    };

}

#endif