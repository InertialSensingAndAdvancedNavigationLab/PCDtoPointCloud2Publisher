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
 * @brief        PCD文件/文件夹到ROS2 PointCloud2消息的发布器  
 *  @details     支持单文件加载和文件夹模式（暂未实现）  
 *   @note        playRate参数支持动态更新  
 *   @throws      std::invalid_argument 参数无效时抛出  
 **/  
class PCDtoPointCloud2Publisher : public rclcpp::Node
{
public:
/**  
 * @brief        构造函数初始化节点和参数  
 *  @details     读取参数并加载数据源  
 *   @throws      std::runtime_error 文件加载失败时抛出  
 *  
 * @param       参数名称: argc                    数据类型: int  
 *  @details     ROS2命令行参数计数  
 * @param       参数名称: argv                    数据类型: char**  
 *  @details     ROS2命令行参数数组  
 **/  
    explicit PCDtoPointCloud2Publisher()
        : Node("pcd_publisher"), start_time_(now())
    {
        // 声明参数（支持嵌套路径）
        declare_parameter("inputPath", string(""));
        declare_parameter("inputFolderMode", false);
        declare_parameter("FolderOptions.playMode", "sequential");
        declare_parameter("FolderOptions.splitFilePrefix", "scans_");
        declare_parameter("FolderOptions.splitStartIndex", 0);
        declare_parameter("topicNamespace", "point_cloud");
        declare_parameter("topicName", "data");
        declare_parameter("frameId", "map");
        declare_parameter("playRate", 1.0);
        declare_parameter("playLoop", false);

        // 注册动态参数回调
        this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& parameters) -> rcl_interfaces::msg::SetParametersResult {
                return this->paramCallback(parameters);
            });

        // 读取并显示所有参数
        readParameters();
        printParameters();

        // 加载文件
        if (inputFolderMode)
        {
            if (!loadFolder())
                exit(1);
        }
        else
        {
            if (!loadSingleFile(inputPath))
                exit(1);
        }

        // 初始化发布器和定时器
        initializePublisherAndTimer();
    }

    // 参数存储（纯驼峰命名）
    string inputPath;
    bool inputFolderMode;
    string folderOptionsPlayMode;
    string folderOptionsSplitFilePrefix;
    int folderOptionsSplitStartIndex;
    string topicNamespace;
    string topicName;
    string frameId;
    double playRate;
    bool playLoop;

    // 点云数据
    PointCloudT::Ptr pointCloud;

    // ROS2组件
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Time start_time_;

/**  
 * @brief        读取所有参数值  
 *  @details     从ROS2参数服务器获取配置  
 *  
 * @return      void  
 **/  
    void readParameters()
    {
        inputPath = get_parameter("inputPath").as_string();
        inputFolderMode = get_parameter("inputFolderMode").as_bool();
        folderOptionsPlayMode = get_parameter("FolderOptions.playMode").as_string();
        folderOptionsSplitFilePrefix = get_parameter("FolderOptions.splitFilePrefix").as_string();
        folderOptionsSplitStartIndex = get_parameter("FolderOptions.splitStartIndex").as_int();
        topicNamespace = get_parameter("topicNamespace").as_string();
        topicName = get_parameter("topicName").as_string();
        frameId = get_parameter("frameId").as_string();
        playRate = get_parameter("playRate").as_double();
        playLoop = get_parameter("playLoop").as_bool();
    }

/**  
 * @brief        显示当前参数配置  
 *  @details     通过ROS2日志输出参数值  
 *  
 * @return      void  
 **/  
    void printParameters()
    {
        RCLCPP_INFO(get_logger(), "Parameters:");
        RCLCPP_INFO(get_logger(), "  inputPath: %s", inputPath.c_str());
        RCLCPP_INFO(get_logger(), "  inputFolderMode: %s", inputFolderMode ? "true" : "false");
        RCLCPP_INFO(get_logger(), "  FolderOptions.playMode: %s", folderOptionsPlayMode.c_str());
        RCLCPP_INFO(get_logger(), "  FolderOptions.splitFilePrefix: %s", folderOptionsSplitFilePrefix.c_str());
        RCLCPP_INFO(get_logger(), "  FolderOptions.splitStartIndex: %d", folderOptionsSplitStartIndex);
        RCLCPP_INFO(get_logger(), "  topicNamespace: %s", topicNamespace.c_str());
        RCLCPP_INFO(get_logger(), "  topicName: %s", topicName.c_str());
        RCLCPP_INFO(get_logger(), "  frameId: %s", frameId.c_str());
        RCLCPP_INFO(get_logger(), "  playRate: %f", playRate);
        RCLCPP_INFO(get_logger(), "  playLoop: %s", playLoop ? "true" : "false");
    }

/**  
 * @brief        加载单个PCD文件  
 *  @details     验证文件存在性并读取点云数据  
 *   @note        仅支持PCD格式文件  
 *   @throws      std::runtime_error 文件加载失败时抛出  
 *  
 * @param       参数名称: path                       数据类型: const string&  
 *  @details     PCD文件路径（必须存在）  
 *  
 * @return      bool                                  数据类型: bool  
 *  @retval      true 文件加载成功  
 *  @retval      false 文件加载失败  
 **/  
    bool loadSingleFile(const string& path)
    {
        if (!std::filesystem::exists(path))
        {
            RCLCPP_FATAL(get_logger(), "File not found: %s", path.c_str());
            return false;
        }

        if (std::filesystem::is_directory(path))
        {
            RCLCPP_FATAL(get_logger(), "Path %s is a directory (expecting single file)", path.c_str());
            return false;
        }

        PointCloudT::Ptr cloud(new PointCloudT);
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(path, *cloud) == -1)
        {
            RCLCPP_FATAL(get_logger(), "Failed to load PCD file: %s", path.c_str());
            return false;
        }

        pointCloud = cloud;
        RCLCPP_INFO(get_logger(), "Loaded PCD file: %s", path.c_str());
        return true;
    }

/**  
 * @brief        加载文件夹模式（未实现）  
 *  @details     未来支持连续扫描文件加载  
 *   @note        当前为占位实现，返回false  
 *  
 * @return      bool                                  数据类型: bool  
 *  @retval      false 暂不支持文件夹模式  
 **/  
    bool loadFolder()
    {
        if (!std::filesystem::exists(inputPath))
        {
            RCLCPP_FATAL(get_logger(), "Input path %s not found", inputPath.c_str());
            return false;
        }

        if (!std::filesystem::is_directory(inputPath))
        {
            RCLCPP_FATAL(get_logger(), "Path %s is not a directory (required for folder mode)", inputPath.c_str());
            return false;
        }

        RCLCPP_WARN(get_logger(), "Folder mode is not yet implemented. Use parallel mode via launch file.");
        return false;
    }

/**  
 * @brief        初始化ROS2发布器和定时器  
 *  @details     根据playRate设置定时器周期  
 *  
 * @return      void  
 **/  
    void initializePublisherAndTimer()
    {
        string fullTopic = topicNamespace + "/" + topicName;
        publisher = create_publisher<sensor_msgs::msg::PointCloud2>(fullTopic, 10);

        auto period = chrono::duration<double>(1.0 / playRate);
        timer = create_wall_timer(period, [this]() { this->publishPointCloud(); });
    }

/**  
 * @brief        发布点云数据  
 *  @details     模拟时间戳并填充ROS消息  
 *   @note        使用playRate参数控制发布速度  
 *  
 * @return      void  
 **/  
    void publishPointCloud()
    {
        if (!pointCloud)
        {
            RCLCPP_WARN(get_logger(), "No point cloud loaded, skipping publish");
            return;
        }

        auto elapsed = (now() - start_time_).seconds();
        auto simulatedTime = start_time_ + rclcpp::Duration::from_seconds(elapsed * playRate);
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*pointCloud, msg);
        msg.header.frame_id = frameId;
        msg.header.stamp = simulatedTime;
        publisher->publish(msg);

        RCLCPP_DEBUG(get_logger(), "Published at simulated time: %.3f", simulatedTime.seconds());
    }

/**  
 * @brief        处理动态参数更新  
 *  @details     验证并应用新参数值  
 *   @note        仅支持playRate参数的动态更新  
 *   @throws      std::invalid_argument 参数无效时抛出  
 *  
 * @param       参数名称: parameters                    数据类型: const std::vector<rclcpp::Parameter>&  
 *  @details     需要更新的参数列表  
 *  
 * @return      rcl_interfaces::msg::SetParametersResult 数据类型: rcl_interfaces::msg::SetParametersResult  
 *  @retval      包含更新结果的成功状态和错误信息  
 **/  
    rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& param : parameters)
        {
            if (param.get_name() == "playRate")
            {
                double newPlayRate = param.as_double();
                if (newPlayRate <= 0.0)
                {
                    result.successful = false;
                    result.reason = "playRate must be >0.0";
                }
                else
                {
                    playRate = newPlayRate;
                    RCLCPP_INFO(get_logger(), "playRate updated to %.2f", playRate);
                    updateTimerPeriod();
                }
            }
            else
            {
                RCLCPP_WARN(get_logger(), "Unknown parameter: %s", param.get_name().c_str());
            }
        }

        if (!result.successful)
            RCLCPP_WARN(get_logger(), "Parameter update failed: %s", result.reason.c_str());

        return result;
    }

/**  
 * @brief        更新定时器周期  
 *  @details     根据新playRate重新配置定时器  
 *  
 * @return      void  
 **/  
    void updateTimerPeriod()
    {
        if (timer)
        {
            timer->cancel();
            timer.reset();
        }

        auto period = chrono::duration<double>(1.0 / playRate);
        timer = create_wall_timer(period, [this]() { this->publishPointCloud(); });
    }
};

/**  
 * @brief        ROS2节点入口函数  
 *  @details     初始化节点并开始事件循环  
 *  
 * @param       参数名称: argc                    数据类型: int  
 *  @details     ROS命令行参数计数  
 * @param       参数名称: argv                    数据类型: char**  
 *  @details     ROS命令行参数数组  
 *  
 * @return      int                              数据类型: int  
 *  @retval      0 程序正常退出  
 **/  
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCDtoPointCloud2Publisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}