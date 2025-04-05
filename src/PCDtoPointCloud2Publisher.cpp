#include "PCDtoPointCloud2Publisher.hpp"

/**  
 * @brief        构造函数初始化节点和参数  
 *  @details     读取参数并加载数据源  
 *   @throws      std::runtime_error 文件加载失败时抛出  
 *  
 * @param       参数名称: argc                          数据类型: int  
 *  @details     ROS2命令行参数计数  
 * @param       参数名称: argv                          数据类型: char**  
 *  @details     ROS2命令行参数数组  
 **/  
PCDtoPointCloud2Publisher::PCDtoPointCloud2Publisher()
    : Node("pcd_publisher"), start_time_(now())
{
    /// 声明输入文件路径参数（支持空值）  
    declare_parameter("inputPath", string(""));  
    /// 启用文件夹模式参数（默认禁用）  
    declare_parameter("inputFolderMode", false);  
    /// 文件夹模式播放模式（sequential/parallel）  
    declare_parameter("FolderOptions.playMode", "sequential");  
    /// 文件夹模式分割文件前缀（默认"scans_"）  
    declare_parameter("FolderOptions.splitFilePrefix", "scans_");  
    /// 文件夹模式起始索引（默认0）  
    declare_parameter("FolderOptions.splitStartIndex", 0);  
    /// ROS2话题命名空间（默认"point_cloud"）  
    declare_parameter("topicNamespace", "point_cloud");  
    /// ROS2话题名称（默认"data"）  
    declare_parameter("topicName", "data");  
    /// 坐标系ID（默认"map"）  
    declare_parameter("frameId", "map");  
    /// 播放速率（默认1.0实时）  
    declare_parameter("playRate", 1.0);  
    /// 是否循环播放（默认false）  
    declare_parameter("playLoop", false);  
    /// 注册动态参数更新回调函数  
    this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& parameters) -> rcl_interfaces::msg::SetParametersResult  
    {  
        return this->paramCallback(parameters);  
    });  

    /// 从参数服务器读取所有配置  
    readParameters();  
    /// 输出当前参数配置到日志  
    printParameters();  
    /// 根据模式加载数据源  
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
    /// 初始化消息发布器和定时器组件  
    initializePublisherAndTimer();  
}  

// 参数读取
/**  
 * @brief        读取所有参数值  
 *  @details     从ROS2参数服务器获取配置  
 *  
 * @return      void                          数据类型: void  
 **/  
void PCDtoPointCloud2Publisher::readParameters()  
{  
    /// 读取输入路径参数  
    inputPath = get_parameter("inputPath").as_string();  
    /// 读取文件夹模式参数  
    inputFolderMode = get_parameter("inputFolderMode").as_bool();  
    /// 读取播放模式参数  
    folderOptionsPlayMode = get_parameter("FolderOptions.playMode").as_string();  
    /// 读取分割文件前缀参数  
    folderOptionsSplitFilePrefix = get_parameter("FolderOptions.splitFilePrefix").as_string();  
    /// 读取起始索引参数  
    folderOptionsSplitStartIndex = get_parameter("FolderOptions.splitStartIndex").as_int();  
    /// 读取话题命名空间参数  
    topicNamespace = get_parameter("topicNamespace").as_string();  
    /// 读取话题名称参数  
    topicName = get_parameter("topicName").as_string();  
    /// 读取坐标系ID参数  
    frameId = get_parameter("frameId").as_string();  
    /// 读取播放速率参数  
    playRate = get_parameter("playRate").as_double();  
    /// 读取循环播放参数  
    playLoop = get_parameter("playLoop").as_bool();  
}  

// 参数打印
/**  
 * @brief        显示当前参数配置  
 *  @details     通过ROS2日志输出参数值  
 *  
 * @return      void                          数据类型: void  
 **/  
void PCDtoPointCloud2Publisher::printParameters()  
{  
    /// 输出参数日志标题  
    RCLCPP_INFO(get_logger(), "Parameters:");  
    /// 输出输入路径参数  
    RCLCPP_INFO(get_logger(), "  inputPath: %s", inputPath.c_str());  
    /// 输出文件夹模式参数  
    RCLCPP_INFO(get_logger(), "  inputFolderMode: %s", inputFolderMode ? "true" : "false");  
    /// 输出播放模式参数  
    RCLCPP_INFO(get_logger(), "  FolderOptions.playMode: %s", folderOptionsPlayMode.c_str());  
    /// 输出分割文件前缀参数  
    RCLCPP_INFO(get_logger(), "  FolderOptions.splitFilePrefix: %s", folderOptionsSplitFilePrefix.c_str());  
    /// 输出起始索引参数  
    RCLCPP_INFO(get_logger(), "  FolderOptions.splitStartIndex: %d", folderOptionsSplitStartIndex);  
    /// 输出话题命名空间参数  
    RCLCPP_INFO(get_logger(), "  topicNamespace: %s", topicNamespace.c_str());  
    /// 输出话题名称参数  
    RCLCPP_INFO(get_logger(), "  topicName: %s", topicName.c_str());  
    /// 输出坐标系ID参数  
    RCLCPP_INFO(get_logger(), "  frameId: %s", frameId.c_str());  
    /// 输出播放速率参数  
    RCLCPP_INFO(get_logger(), "  playRate: %f", playRate);  
    /// 输出循环播放参数  
    RCLCPP_INFO(get_logger(), "  playLoop: %s", playLoop ? "true" : "false");  
}  

// 文件加载
/**  
 * @brief        加载单个PCD文件  
 *  @details     验证文件存在性并读取点云数据  
 *   @note        仅支持PCD格式文件  
 *   @throws      std::runtime_error 文件加载失败时抛出  
 *  
 * @param       参数名称: path                          数据类型: const string&  
 *  @details     PCD文件路径（必须存在）  
 *  
 * @return      bool                                  数据类型: bool  
 *  @retval      true 文件加载成功  
 *  @retval      false 文件加载失败  
 **/  
bool PCDtoPointCloud2Publisher::loadSingleFile(const string& path)  
{  
    /// 验证文件存在性  
    if (!std::filesystem::exists(path))  
    {  
        RCLCPP_FATAL(get_logger(), "File not found: %s", path.c_str());  
        return false;  
    }  

    /// 验证路径是否为文件而非目录  
    if (std::filesystem::is_directory(path))  
    {  
        RCLCPP_FATAL(get_logger(), "Path %s is a directory (expecting single file)", path.c_str());  
        return false;  
    }  

    /// 创建临时点云对象  
    PointCloudT::Ptr cloud(new PointCloudT);  
    /// 尝试加载PCD文件  
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(path, *cloud) == -1)  
    {  
        RCLCPP_FATAL(get_logger(), "Failed to load PCD file: %s", path.c_str());  
        return false;  
    }  

    /// 保存加载成功的点云数据  
    pointCloud = cloud;  
    RCLCPP_INFO(get_logger(), "Loaded PCD file: %s", path.c_str());  
    return true;  
}  

// 文件夹加载（占位）
/**  
 * @brief        加载文件夹模式（未实现）  
 *  @details     未来支持连续扫描文件加载  
 *   @note        当前为占位实现，返回false  
 *  
 * @return      bool                                  数据类型: bool  
 *  @retval      false 暂不支持文件夹模式  
 **/  
bool PCDtoPointCloud2Publisher::loadFolder()  
{  
    /// 验证输入路径存在  
    if (!std::filesystem::exists(inputPath))  
    {  
        RCLCPP_FATAL(get_logger(), "Input path %s not found", inputPath.c_str());  
        return false;  
    }  

    /// 验证路径是否为目录  
    if (!std::filesystem::is_directory(inputPath))  
    {  
        RCLCPP_FATAL(get_logger(), "Path %s is not a directory (required for folder mode)", inputPath.c_str());  
        return false;  
    }  

    /// 输出功能未实现的警告  
    RCLCPP_WARN(get_logger(), "Folder mode is not yet implemented. Use parallel mode via launch file.");  
    return false;  
}
/**  
 * @brief        初始化ROS2发布器和定时器  
 *  @details     根据playRate参数配置发布器和定时器  
 *   @note       定时器周期 = 1/playRate（单位：秒）  
 *   @throws     std::invalid_argument 参数无效时抛出  
 *  
 * @return      void                          数据类型: void  
 **/  
void PCDtoPointCloud2Publisher::initializePublisherAndTimer()  
{  
    /// 构造ROS2话题全路径（命名空间/话题名）  
    string fullTopic = topicNamespace + "/" + topicName;  
    /// 创建ROS2点云消息发布器，队列深度设为10  
    publisher = create_publisher<sensor_msgs::msg::PointCloud2>(fullTopic, 10);  
    /// 根据playRate参数计算定时器周期（秒）  
    auto period = chrono::duration<double>(1.0 / playRate);  
    /// 创建固定频率定时器并绑定点云发布回调函数  
    timer = create_wall_timer(period, [this]() { this->publishPointCloud(); });  
}
/**  
 * @brief        发布点云数据  
 *  @details     模拟时间戳并填充ROS消息  
 *   @note        使用playRate参数控制发布速度  
 *   @throws      std::runtime_error 点云数据未加载时抛出  
 *  
 * @return      void                          数据类型: void  
 **/  
void PCDtoPointCloud2Publisher::publishPointCloud()  
{  
    /// 检查点云数据是否已加载  
    if (!pointCloud)  
    {  
        RCLCPP_WARN(get_logger(), "No point cloud loaded, skipping publish");  
        return;  
    }  
    /// 计算节点运行时间（单位：秒）  
    auto elapsed = (now() - start_time_).seconds();  
    /// 根据playRate生成模拟时间戳  
    auto simulatedTime = start_time_ + rclcpp::Duration::from_seconds(elapsed * playRate);  
    /// 创建ROS2 PointCloud2消息实例  
    sensor_msgs::msg::PointCloud2 msg;  
    /// 将PCL点云数据转换为ROS2消息格式  
    pcl::toROSMsg(*pointCloud, msg);  
    /// 设置消息坐标系ID  
    msg.header.frame_id = frameId;  
    /// 赋予模拟时间戳  
    msg.header.stamp = simulatedTime;  
    /// 发布转换后的点云消息  
    publisher->publish(msg);  
    /// 记录发布事件调试信息  
    RCLCPP_DEBUG(get_logger(), "Published at simulated time: %.3f", simulatedTime.seconds());  
}
/**  
 * @brief        处理动态参数更新  
 *  @details     验证并应用新参数值  
 *   @note        仅支持playRate参数的动态更新  
 *   @throws      std::invalid_argument 参数无效时抛出  
 *  
 * @param       参数名称: parameters                          数据类型: const std::vector<rclcpp::Parameter>&  
 *  @details     需要更新的参数列表（仅处理playRate参数）  
 *  
 * @return      rcl_interfaces::msg::SetParametersResult       数据类型: rcl_interfaces::msg::SetParametersResult  
 *  @retval      result.successful = true 更新成功  
 *  @retval      result.successful = false 参数无效或更新失败  
 **/  
rcl_interfaces::msg::SetParametersResult PCDtoPointCloud2Publisher::paramCallback(const std::vector<rclcpp::Parameter>& parameters)  
{  
    /// 初始化参数更新结果为成功  
    rcl_interfaces::msg::SetParametersResult result;  
    result.successful = true;  

    /// 遍历所有传入的参数  
    for (const auto& param : parameters)  
    {  
        /// 检查参数名称是否为playRate  
        if (param.get_name() == "playRate")  
        {  
            /// 解析新播放速率值  
            double newPlayRate = param.as_double();  
            /// 验证参数有效性（必须>0）  
            if (newPlayRate <= 0.0)  
            {  
                result.successful = false;  
                result.reason = "playRate must be >0.0";  // 设置失败原因  
            }  
            else  
            {  
                /// 更新成员变量并重新配置定时器  
                playRate = newPlayRate;  
                RCLCPP_INFO(get_logger(), "playRate updated to %.2f", playRate);  
                updateTimerPeriod();  // 重新计算定时器周期  
            }  
        }  
        else  
        {  
            /// 记录未知参数警告  
            RCLCPP_WARN(get_logger(), "Unknown parameter: %s", param.get_name().c_str());  
        }  
    }  

    /// 检查更新是否失败并记录原因  
    if (!result.successful)  
        RCLCPP_WARN(get_logger(), "Parameter update failed: %s", result.reason.c_str());  

    /// 返回最终更新结果  
    return result;  
}
/**  
 * @brief        更新定时器周期  
 *  @details     根据新playRate重新配置定时器  
 *   @note       需确保在ROS主循环中调用（避免竞态条件）  
 *   @throws     std::runtime_error 定时器创建失败时抛出  
 *  
 * @return      void  
 **/  
void PCDtoPointCloud2Publisher::updateTimerPeriod()  
{  
    /// 检查现有定时器是否存在  
    if (timer)  
    {  
        /// 取消现有定时器任务  
        timer->cancel();  
        /// 重置定时器智能指针（释放资源）  
        timer.reset();  
    }  
    /// 根据新playRate计算定时器周期（单位：秒）  
    auto period = chrono::duration<double>(1.0 / playRate);  
    /// 创建新的固定频率定时器并绑定发布回调  
    timer = create_wall_timer(period, [this]() { this->publishPointCloud(); });  
}
/**  
 * @brief        ROS2节点入口函数  
 *  @details     初始化ROS2环境并启动节点事件循环  
 *   @note       必须通过ROS2 launch文件或终端启动  
 *   @throws     std::runtime_error 节点创建失败时抛出  
 *  
 * @param       参数名称: argc                          数据类型: int  
 *  @details     ROS2命令行参数计数（如：节点名称参数）  
 * @param       参数名称: argv                          数据类型: char**  
 *  @details     ROS2命令行参数数组（如：ros2 run ...）  
 *  
 * @return      int                                    数据类型: int  
 *  @retval      0 程序正常退出  
 *  @retval      -1 节点创建失败  
 **/  
int main(int argc, char* argv[])  
{  
    /// 初始化ROS2通信环境  
    rclcpp::init(argc, argv);  
    /// 创建PCD到ROS2消息转换节点实例  
    auto node = std::make_shared<PCDtoPointCloud2Publisher>();  
    /// 进入ROS2事件循环并开始处理回调  
    rclcpp::spin(node);  
    /// 关闭ROS2通信环境  
    rclcpp::shutdown();  
    /// 程序正常终止  
    return 0;  
}