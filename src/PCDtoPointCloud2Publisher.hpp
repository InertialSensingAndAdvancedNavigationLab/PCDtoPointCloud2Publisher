#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <filesystem>
#include <memory>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

using namespace std;
using namespace pcl;
using PointCloudT = pcl::PointCloud<pcl::PointXYZI>;

/**  
 * @class       PCDtoPointCloud2Publisher  
 * @brief       PCD文件/文件夹到ROS2 PointCloud2消息的发布器  
 *  @details    支持单文件加载和文件夹模式（暂未实现）  
 *   @note       playRate参数支持动态更新  
 *   @throws     std::invalid_argument 参数无效时抛出  
 **/  
class PCDtoPointCloud2Publisher : public rclcpp::Node  
{  
public:  
    /// 初始化节点并加载参数和数据源（可能抛出std::runtime_error）  
    explicit PCDtoPointCloud2Publisher();  
    /// 输入文件路径（必须存在）  
    string inputPath;  
    /// 是否启用文件夹模式  
    bool inputFolderMode;  
    /// 文件夹模式播放模式（sequential/parallel）  
    string folderOptionsPlayMode;  
    /// 文件夹模式分割文件前缀  
    string folderOptionsSplitFilePrefix;  
    /// 文件夹模式起始索引  
    int folderOptionsSplitStartIndex;  
    /// ROS2话题命名空间  
    string topicNamespace;  
    /// ROS2话题名称  
    string topicName;  
    /// 坐标系ID（如map/base_link）  
    string frameId;  
    /// 播放速率（1.0为实时）  
    double playRate;  
    /// 是否循环播放  
    bool playLoop;  
    /// 点云数据缓存  
    PointCloudT::Ptr pointCloud;  
    /// ROS2 PointCloud2消息发布器  
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;  
    /// 定时器用于控制发布频率  
    rclcpp::TimerBase::SharedPtr timer;  
    /// 记录节点启动时间戳  
    rclcpp::Time start_time_;  
    /// 从ROS2参数服务器获取配置  
    void readParameters();  
    /// 通过ROS2日志输出参数值  
    void printParameters();  
    /// 验证并加载单个PCD文件（仅支持PCD格式）  
    bool loadSingleFile(const string& path);  
    /// 加载文件夹模式（暂未实现）  
    bool loadFolder();  
    /// 根据playRate设置定时器周期  
    void initializePublisherAndTimer();  
    /// 模拟时间戳并发布点云数据  
    void publishPointCloud();  
    /// 处理动态参数更新（仅支持playRate参数）  
    rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter>& parameters);  
    /// 根据新playRate重新配置定时器  
    void updateTimerPeriod();  
};