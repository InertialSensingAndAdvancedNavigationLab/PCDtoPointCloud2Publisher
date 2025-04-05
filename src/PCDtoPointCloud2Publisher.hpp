#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <filesystem>
#include <memory>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <regex>

using namespace std;
using namespace pcl;
using PointCloudT = pcl::PointCloud<pcl::PointXYZI>;

/**  
 * @class       PCDtoPointCloud2Publisher  
 * @brief       PCD文件/文件夹到ROS2 PointCloud2消息的发布器  
 *  @details    支持单文件加载和文件夹模式（分包模式/顺序模式）  
 *   @note       playRate参数支持动态更新，分包模式需文件名含数字后缀  
 *   @throws     std::invalid_argument 参数无效或文件格式错误时抛出  
 **/  
class PCDtoPointCloud2Publisher : public rclcpp::Node  
{  
public:  
    /// 输入文件路径（必须存在）  
    string inputPath;  
    /// 是否启用文件夹模式  
    bool inputFolderMode;  
    /// 文件夹模式播放模式（sequential/parallel）  
    string folderOptionsPlayMode;  
    /// 文件夹模式分割文件前缀（如"scans_"）  
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
    /// 节点启动时间戳  
    rclcpp::Time startTime;  
    /// 文件路径列表（按加载顺序排列）  
    std::vector<std::string> fileList;  
    /// 当前处理的文件索引  
    size_t currentFileIndex;  
    /// 标志文件列表是否已加载  
    bool isFileListLoaded;  
    /// 分包模式起始索引（对应文件名数字后缀）  
    int splitStartIndex;  
    /// 分包模式当前文件数字后缀  
    int currentSplitIndex;  
    /// 预加载异步任务（用于后台加载下一文件）  
    std::future<bool> preloadFuture;  
    /// 保护文件列表的互斥锁  
    std::mutex fileListMutex;  

    /// @brief 节点构造函数，初始化参数并加载数据源  
    explicit PCDtoPointCloud2Publisher();  
    /// @brief 从ROS2参数服务器读取配置参数  
    void readParameters();  
    /// @brief 通过ROS2日志输出当前参数配置  
    void printParameters();  
    /// @brief 加载单个PCD文件（单文件模式使用）  
    bool loadSingleFile(const string& path);  
    /// @brief 加载文件夹模式下的所有PCD文件（支持分包和顺序模式）  
    bool loadFolder();  
    /// @brief 初始化ROS2发布器和定时器配置  
    void initializePublisherAndTimer();  
    /// @brief 发布当前点云数据并切换至下一文件  
    void publishPointCloud();  
    /// @brief 处理动态参数更新（仅支持playRate参数）  
    rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter>& parameters);  
    /// @brief 根据新playRate重新配置定时器周期  
    void updateTimerPeriod();  
    /// @brief 按文件名字典序加载文件列表（顺序模式）  
    bool loadSequentialFiles(const std::string& folderPath);  
    /// @brief 按数字后缀排序加载分包模式文件  
    void loadSplitFiles(const std::string& folderPath);  
    /// @brief 根据playRate计算加速后的时间戳  
    rclcpp::Time calculateSimulatedTime();  
    /// @brief 发布当前文件点云数据  
    bool publishCurrentFile();  
    /// @brief 预加载下一个文件（异步执行）  
    void preloadNextFile();  
    /// @brief 切换到下一个文件并释放资源  
    bool switchToNextFile();  
};