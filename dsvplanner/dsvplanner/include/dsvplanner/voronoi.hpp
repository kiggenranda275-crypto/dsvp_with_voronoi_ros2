#ifndef VORONOI_HPP
#define VORONOI_HPP

#include "dynamicvoronoi.h"
#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_cloud.h"
#include <pcl/point_types.h>
#include <Eigen/Dense>

namespace dsvplanner_ns
{
    class Voronoi : public rclcpp::Node
    {
    public:
        Voronoi(const std::string image_topic = "/voronoi_image") : Node("dynamic_voronoi"), image_topic(image_topic)
        {
            voronoi = new DynamicVoronoi();
            image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(image_topic, 10);

            this->declare_parameter("kOccuptGridLimit", 50);
            this->get_parameter_or<int>("kOccuptGridLimit", kOccuptGridLimit, 50);
        }
        ~Voronoi()
        {
            delete voronoi;
        }

        void setMap(std::vector<std::vector<bool>> map)
        {
            grid_map = map;
            height = map.size();
            width = map[0].size();
        }

        // 根据地图更新Voronoi图
        void update_voronoi(bool purgeAlt = false)
        {
            bool **bool_map = new bool *[height];
            for (int i = 0; i < height; i++)
            {
                bool_map[i] = new bool[width];
                for (int j = 0; j < width; j++)
                {
                    bool_map[i][j] = grid_map[i][j];
                }
            }

            auto start = std::chrono::high_resolution_clock::now();
            voronoi->initializeMap(height, width, bool_map);
            voronoi->update();
            if (!purgeAlt)
                voronoi->prune();
            else
                voronoi->updateAlternativePrunedDiagram(); // 转化为稀疏Voronoi图
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            // RCLCPP_INFO(this->get_logger(), "Voronoi update time: %f", elapsed.count());
            vis_map = voronoi->get_visual();
            delete[] bool_map;
        }

        // 获取Voronoi图的点
        void get_voronoi_split()
        {
            vis_map = voronoi->get_visual();
        }

        // 转为图像发布
        void visualize_publish()
        {
            sensor_msgs::msg::Image image;
            image.header.frame_id = "map";
            image.header.stamp = this->now();
            image.height = height;
            image.width = width;
            image.encoding = "rgb8";
            image.step = width * 3;
            image.data.resize(width * height * 3, 0);
            for (int i = 0; i < height; i++)
            {
                for (int j = 0; j < width; j++)
                {
                    image.data[(i * width + j) * 3] = vis_map[i][j].r;
                    image.data[(i * width + j) * 3 + 1] = vis_map[i][j].g;
                    image.data[(i * width + j) * 3 + 2] = vis_map[i][j].b;
                }
            }
            for (int i = 0; i < voronoi_edges.size(); i++)
            {
                int x = voronoi_edges[i].x();
                int y = voronoi_edges[i].y();

                image.data[(x * width + y) * 3] = 0;
                image.data[(x * width + y) * 3 + 1] = 255;
                image.data[(x * width + y) * 3 + 2] = 0;
                if (voronoi_edges[i].z() > 100)
                {
                    image.data[(x * width + y) * 3] = 255;
                    image.data[(x * width + y) * 3 + 1] = 0;
                    image.data[(x * width + y) * 3 + 2] = 0;
                }
            }
            image_pub_->publish(image);
        }

        void getVoronoiEdges(std::vector<Eigen::Vector3i> &edges, std::vector<std::pair<int, int>> &paths)
        {
            edges.clear();
            paths.clear();
            const std::vector<std::pair<int, int>> Find_dir = {
                {1, 0},
                {0, 1},
                {-1, 0},
                {0, -1}};
            // 4个方向查找{1, 1}, {1, -1}, {-1, -1}, {-1, 1}
            for (int i = 0; i < height; i++)
            {
                for (int j = 0; j < width; j++)
                {
                    if (voronoi->isVoronoi(i, j))
                    {
                        paths.push_back(std::make_pair(i, j));
                        int surroundNum = 0;
                        for (int k = 0; k < 4; k++)
                        {
                            int nx = i + Find_dir[k].first;
                            int ny = j + Find_dir[k].second;
                            if (nx >= 0 && nx < height && ny >= 0 && ny < width)
                            {
                                if (voronoi->isVoronoi(nx, ny))
                                {
                                    surroundNum++;
                                }
                            }
                        }
                        if (surroundNum == 1)
                        {
                            Eigen::Vector3i edge;
                            edge.x() = i;
                            edge.y() = j;
                            edge.z() = voronoi->getSqdistance(i, j);
                            // RCLCPP_INFO(this->get_logger(), "edge %d %d %d", i, j, edge.z());
                            edges.push_back(edge);
                        }
                    }
                }
                voronoi_edges = edges;
            }
        }

    private:
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_; // 可视化Vorioi图

        DynamicVoronoi *voronoi;
        std::string image_topic;
        std::vector<std::vector<bool>> grid_map;
        std::vector<std::vector<RGB>> vis_map;
        std::vector<Eigen::Vector3i> voronoi_edges;
        int kOccuptGridLimit;
        int width;
        int height;
    };
}

#endif // VORONOI_HPP