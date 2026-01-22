#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <string>
class SlamInterface : public rclcpp::Node
{
    public:
    SlamInterface()
        : Node("slam_interface"), tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())), tf_listener_(*tf_buffer_)
    {
        this->declare_parameter("registered_scan_topic", "/lio_sam/mapping/registered_scan");
        this->declare_parameter("odom_topic", "/lio_sam/mapping/odometry");
        this->declare_parameter("odom_tf_name", "odom");
        this->declare_parameter("map_tf_name", "map");
        this->declare_parameter("robot_tf_name", "base_link");
        this->declare_parameter("base_tf_name", "base_link");
        this->declare_parameter("sensor_tf_name", "velodyne_link");
        this->declare_parameter("publish_tf", false);
        this->declare_parameter("publish_lidar2base", false);

        this->get_parameter_or<std::string>("registered_scan_topic", registered_scan_topic_, "/lio_sam/mapping/registered_scan");
        this->get_parameter_or<std::string>("odom_topic", odom_topic_, "/lio_sam/mapping/odometry");
        this->get_parameter_or<std::string>("odom_tf_name", odom_tf_name_, "odom");
        this->get_parameter_or<std::string>("map_tf_name", map_tf_name_, "map");
        this->get_parameter_or<std::string>("robot_tf_name", robot_tf_name_, "base_link");
        this->get_parameter_or<std::string>("base_tf_name", base_tf_name_, "base_link");
        this->get_parameter_or<std::string>("snesor_tf_name", sensor_tf_name_, "velodyne_link");
        this->get_parameter_or<bool>("publish_tf", publish_tf_, false);
        this->get_parameter_or<bool>("publish_lidar2base", publish_lidar2base_, false);

        // 订阅 Velodyne 的点云话题
        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            registered_scan_topic_, 10,
            std::bind(&SlamInterface::pointCloudCallback, this, std::placeholders::_1)); // odom - map

        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/state_estimation", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10,
            std::bind(&SlamInterface::odomCallback, this, std::placeholders::_1));
        // 发布转换到 map 坐标系的点云
        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/registered_scan", 10);

        // 发布map到base_link的变换
        tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        while (1)
        {
            try
            {
                transform_stamped = tf_buffer_->lookupTransform(map_tf_name_, odom_tf_name_, tf2::TimePointZero);
                transform_stamped_lidar2base = tf_buffer_->lookupTransform(sensor_tf_name_, base_tf_name_, tf2::TimePointZero);
                RCLCPP_INFO(this->get_logger(), "Get transform map to odom and lidar to base");
                break;
            }
            catch (tf2::TransformException& ex)
            {
                RCLCPP_WARN(this->get_logger(), "Could not transform odom 2 map");
            }
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
    }

    private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

    std::string registered_scan_topic_ = "/registered_scan";
    std::string odom_topic_ = "/lio_sam/mapping/odometry";
    std::string odom_tf_name_ = "odom";
    std::string map_tf_name_ = "map";
    std::string robot_tf_name_ = "base_link";
    std::string base_tf_name_ = "base_link";
    std::string sensor_tf_name_ = "velodyne_link";

    bool publish_tf_ = false;         // 根据里程计发布到base的坐标变换，读取lidar2base的tf，结合里程计odom2lidar，发布odom2base的tf
    bool publish_lidar2base_ = false; // 发布lidar到base的坐标变换
    geometry_msgs::msg::TransformStamped transform_stamped;
    geometry_msgs::msg::TransformStamped transform_stamped_lidar2base;

     /**
     * @brief 里程计回调函数，处理接收到的里程计数据
     * 
     * 1. 将odom坐标系下的里程计数据转换到map坐标系
     * 2. 发布转换后的里程计数据
     * 3. 根据配置发布不同的TF变换
     * 
     * @param msg 接收到的里程计消息
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {

        // odom -> lidar 转化为 map -> lidar
        nav_msgs::msg::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = msg->header.stamp;
        laserOdometryROS.header.frame_id = map_tf_name_;
        laserOdometryROS.child_frame_id = sensor_tf_name_;
        laserOdometryROS.pose.pose.position.x = msg->pose.pose.position.x;
        laserOdometryROS.pose.pose.position.y = msg->pose.pose.position.y;
        laserOdometryROS.pose.pose.position.z = msg->pose.pose.position.z + transform_stamped.transform.translation.z; //
        laserOdometryROS.pose.pose.orientation = msg->pose.pose.orientation;

        pub_odom_->publish(laserOdometryROS);

        if (publish_tf_) // odom 2 base
        {
            tf2::Stamped<tf2::Transform> lidar2Baselink;
            tf2::Stamped<tf2::Transform> tCur;
            if (base_tf_name_ != sensor_tf_name_) // body->lidar
            {
                tf2::fromMsg(transform_stamped_lidar2base, lidar2Baselink);

                tf2::Transform temp_odom_to_lidar;
                temp_odom_to_lidar.setOrigin(tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
                temp_odom_to_lidar.setRotation(tf2::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));

                tf2::Stamped<tf2::Transform> tb(
                    temp_odom_to_lidar * lidar2Baselink, tf2_ros::fromRclcpp(msg->header.stamp), odom_tf_name_); // 发布坐标变换
                tCur = tb;
                geometry_msgs::msg::TransformStamped transformStamped;
                tf2::convert(tCur, transformStamped);
                transformStamped.header.stamp = msg->header.stamp;
                transformStamped.header.frame_id = odom_tf_name_;
                transformStamped.child_frame_id = base_tf_name_;
                tfBroadcaster->sendTransform(transformStamped);
            }
        }
        if (publish_lidar2base_) // 发布robot -> base的tf，用于slam tf不一致情况
        {
            tf2::Stamped<tf2::Transform> lidar2Baselink;
            if (base_tf_name_ != robot_tf_name_) // 发布的不是odom2base；slam发布odom2robot;发布robot2base
            {
                tf2::fromMsg(this->tf_buffer_->lookupTransform(
                    sensor_tf_name_, base_tf_name_, rclcpp::Time(0)),
                    lidar2Baselink);
                geometry_msgs::msg::TransformStamped transformStamped_;
                tf2::convert(lidar2Baselink, transformStamped_);
                // transformStamped_.header.stamp = msg->header.stamp;
                transformStamped_.header.frame_id = robot_tf_name_;
                transformStamped_.child_frame_id = base_tf_name_;
                tfBroadcaster->sendTransform(transformStamped_); // 发布robot->base的tf，同步
            }
        }
    }


    /**
     * @brief 点云回调函数，处理接收到的点云数据
     * 
     * 1. 将点云从odom坐标系转换到map坐标系
     * 2. 发布转换后的点云数据
     * 
     * @param msg 接收到的点云消息
     */
    // odom 坐标下的点云转换到 map 坐标系点云
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 获取 odom 2 map 的变换
        Eigen::Affine3f transform_matrix = getTransformMatrix(transform_stamped);
        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *input_cloud);
        pcl::transformPointCloud(*input_cloud, *transformed_cloud, transform_matrix);
        // 转换回 ROS 点云消息,veldoyne -> map
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*transformed_cloud, output_msg);
        output_msg.header.frame_id = map_tf_name_;
        output_msg.header.stamp = msg->header.stamp;

        // 发布转换后的点云
        point_cloud_pub_->publish(output_msg);
    }

    /**
     * @brief 将ROS TransformStamped消息转换为Eigen仿射变换矩阵
     * 
     * @param transform ROS格式的坐标变换消息
     * @return Eigen::Affine3f Eigen格式的仿射变换矩阵
     */
    Eigen::Affine3f getTransformMatrix(const geometry_msgs::msg::TransformStamped& transform)
    {
        // 提取平移
        Eigen::Translation3f translation(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z);

        // 提取旋转
        Eigen::Quaternionf rotation(
            transform.transform.rotation.w,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z);

        // 构造仿射变换矩阵
        return translation * rotation;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlamInterface>());
    rclcpp::shutdown();
    return 0;
}
