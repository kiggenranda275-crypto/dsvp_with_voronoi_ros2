#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

const double PI = 3.1415926;

string boundary_file_dir; // 存储边界文件的目录
bool sendBoundary = true; // 标志位，用于控制是否发送边界信息
int sendBoundaryInterval = 2; // 发送边界信息的时间间隔（秒）
int sendBoundaryCount = 0; // 发送边界信息的计数器

pcl::PointCloud<pcl::PointXYZ>::Ptr boundary(new pcl::PointCloud<pcl::PointXYZ>()); // 创建一个PCL点云对象，用于存储边界点信息

// reading boundary from file function
void readBoundaryFile()
{
    FILE *boundary_file = fopen(boundary_file_dir.c_str(), "r");
    if (boundary_file == NULL)
    {
        printf("\nCannot read input files, exit.\n\n");
        exit(1);
    }

    char str[50];
    int val, pointNum;
    string strCur, strLast;
    while (strCur != "end_header")  // 读取文件头，直到遇到 "end_header"
    {
        val = fscanf(boundary_file, "%s", str);
        if (val != 1) // 如果读取文件头时出错，输出错误信息并退出程序
        {
            printf("\nError reading input files, exit.\n\n");
            exit(1);
        }

        strLast = strCur;
        strCur = string(str);

        if (strCur == "vertex" && strLast == "element") // 如果遇到 "element vertex"，则读取顶点数量
        {
            val = fscanf(boundary_file, "%d", &pointNum);
            if (val != 1)
            {
                printf("\nError reading input files, exit.\n\n");
                exit(1);
            }
        }
    }

    boundary->clear();
    pcl::PointXYZ point;
    int val1, val2, val3;
    for (int i = 0; i < pointNum; i++) // 读取每个顶点的坐标信息
    {
        val1 = fscanf(boundary_file, "%f", &point.x);
        val2 = fscanf(boundary_file, "%f", &point.y);
        val3 = fscanf(boundary_file, "%f", &point.z);

        if (val1 != 1 || val2 != 1 || val3 != 1)
        {
            printf("\nError reading input files, exit.\n\n");
            exit(1);
        }

        point.z = 0; //将顶点z坐标设置为0
        boundary->push_back(point); // 将顶点添加到点云对象中
    }

     // 如果第一个点和最后一个点不相同，则将第一个点添加到点云对象的末尾，形成闭环
    if (boundary->points[0].x != boundary->points[pointNum - 1].x || boundary->points[0].y != boundary->points[pointNum - 1].y)
    {
        boundary->push_back(boundary->points[0]);
    }

    fclose(boundary_file); // 5个点 关闭文件
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // 初始化ROS 2节点
    rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("navigationBoundary"); // 创建一个名为 "navigationBoundary" 的节点

    nh->declare_parameter("boundary_file_dir", boundary_file_dir);
    nh->declare_parameter("sendBoundary", sendBoundary);
    nh->declare_parameter("sendBoundaryInterval", sendBoundaryInterval);

    nh->get_parameter("boundary_file_dir", boundary_file_dir);
    nh->get_parameter("sendBoundary", sendBoundary);
    nh->get_parameter("sendBoundaryInterval", sendBoundaryInterval);

    // 创建一个发布者，用于发布 geometry_msgs::msg::PolygonStamped 消息到 /navigation_boundary 话题
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pubBoundary = nh->create_publisher<geometry_msgs::msg::PolygonStamped>("/navigation_boundary", 5);
    // 创建一个 geometry_msgs::msg::PolygonStamped 消息对象
    geometry_msgs::msg::PolygonStamped boundaryMsgs;
    boundaryMsgs.header.frame_id = "map";

    // read boundary from file  如果需要发送边界信息，则从文件中读取边界信息
    if (sendBoundary)
    {
        readBoundaryFile(); // 读取了5个点

        int boundarySize = boundary->points.size();
        boundaryMsgs.polygon.points.resize(boundarySize);
        for (int i = 0; i < boundarySize; i++)
        {
            boundaryMsgs.polygon.points[i].x = boundary->points[i].x;
            boundaryMsgs.polygon.points[i].y = boundary->points[i].y;
            boundaryMsgs.polygon.points[i].z = boundary->points[i].z;
        }
    }

    rclcpp::WallRate loopRate(100);
    while (rclcpp::ok())
    {
        // publish boundary messages at certain frame rate
        sendBoundaryCount++; // 发送边界信息的计数器加1
        if (sendBoundaryCount >= 100 * sendBoundaryInterval && sendBoundary) // 如果计数器达到指定的时间间隔，并且需要发送边界信息，则发布消息
        {
            pubBoundary->publish(boundaryMsgs);
            sendBoundaryCount = 0;
        }
        loopRate.sleep();
    }

    return 0;
}
