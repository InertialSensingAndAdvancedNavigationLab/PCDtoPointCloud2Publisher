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
    : Node("PCDtoPointCloud2Publisher"), 
    currentFileIndex(0),  
    isFileListLoaded(false),
    startTime(now())
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
    try {  
        /// 从参数服务器读取所有配置  
        readParameters();  
        /// 输出当前参数配置到日志  
        printParameters();  
        /// 统一处理输入路径  
        if (std::filesystem::is_regular_file(inputPath)) {  
            // 单文件模式：将文件路径添加到fileList  
            fileList.push_back(inputPath);  
            inputFolderMode = false;  
        } else if (std::filesystem::is_directory(inputPath)) {  
            /// 文件夹模式：根据playMode加载文件列表  
            inputFolderMode = true;  
            if (folderOptionsPlayMode == "split") {  
                loadSplitFiles(inputPath);  
            } else if (folderOptionsPlayMode == "sequential") {  
                loadSequentialFiles(inputPath);  
            } else {  
                throw std::invalid_argument("Unsupported play mode");  
            }  
        } else {  
            throw std::invalid_argument("Input path is neither file nor directory");  
        }  
        /// 验证文件列表非空  
        if (fileList.empty()) {  
            RCLCPP_FATAL(get_logger(), "No valid PCD files found");  
            rclcpp::shutdown();  
        } 
        /// 初始化播放逻辑  
        isFileListLoaded = true;  
        initializePublisherAndTimer();  
        /// 捕捉可能的异常，反正我只会关机
    } catch (const std::exception& e) {  
        RCLCPP_FATAL(get_logger(), "Initialization failed: %s", e.what());  
        rclcpp::shutdown();  
    }  
}  
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
/**  
 * @brief        加载单个PCD文件  
 *  @details     验证文件存在性并读取点云数据  
 *   @note        仅支持PCD格式文件  
 *   @throws      std::runtime_error 文件路径无效或加载失败时抛出  
 *  
 * @param       参数名称: path                          数据类型: const std::string&  
 *  @details     需要加载的PCD文件完整路径（必须存在）  
 *  
 * @return      bool                                  数据类型: bool  
 *  @retval      true 文件加载成功且数据有效  
 *  @retval      false 文件不存在、加载失败或数据为空  
 **/  
bool PCDtoPointCloud2Publisher::loadSingleFile(const std::string& path)  
{  
    /// 创建新的点云对象用于数据存储  
    pointCloud = std::make_shared<PointCloudT>();  
    /// 验证文件路径有效性  
    if (!std::filesystem::exists(path))  
    {  
        /// 记录文件不存在错误  
        RCLCPP_ERROR(get_logger(), "File not found: %s", path.c_str());  
        /// 返回加载失败  
        return false;  
    }  
    /// 尝试读取PCD文件内容  
    if (pcl::io::loadPCDFile(path, *pointCloud) == -1)  
    {  
        /// 记录文件加载失败  
        RCLCPP_ERROR(get_logger(), "Failed to load PCD file: %s", path.c_str());  
        /// 返回加载失败  
        return false;  
    }  
    /// 检查点云数据是否为空  
    if (pointCloud->empty())  
    {  
        /// 记录空点云错误  
        RCLCPP_ERROR(get_logger(), "Loaded empty point cloud from: %s", path.c_str());  
        /// 返回加载失败  
        return false;  
    }  
    /// 记录成功加载信息（包含点云规模）  
    RCLCPP_INFO(get_logger(), "Loaded %zu points from %s", pointCloud->size(), path.c_str());  
    /// 返回加载成功  
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
 * @brief        统一发布点云消息（支持顺序和分包模式）  
 *  @details     根据文件列表顺序发布点云，支持预加载下一个文件  
 *   @note        使用异步预加载减少文件切换延迟  
 *   @throws      std::runtime_error 点云数据加载失败时抛出  
 *  
 * @param       参数名称: 无参数                              数据类型: 无  
 * @return      void 类型                                    数据类型: void  
 **/  
void PCDtoPointCloud2Publisher::publishPointCloud()  
{  
    /// 检查当前点云是否已加载或为空  
    if (!pointCloud || pointCloud->empty())  
    {  
        /// 检查是否超出文件列表范围  
        if (currentFileIndex >= fileList.size())  
        {  
            /// 处理循环播放模式  
            if (playLoop)  
            {  
                currentFileIndex = 0; // 重置到第一个文件  
            }  
            else  
            {  
                RCLCPP_INFO(get_logger(), "All files played, exiting...");  
                return; // 退出发布流程  
            }  
        }  
        /// 获取当前文件路径  
        std::string currentFile = fileList[currentFileIndex];  
        /// 加载当前文件内容  
        if (!loadSingleFile(currentFile))  
        {  
            currentFileIndex++; // 跳过无效文件  
            return; // 中止本次发布  
        }  
    }  
    /// 创建ROS2 PointCloud2消息实例  
    sensor_msgs::msg::PointCloud2 msg;  
    /// 将PCL点云数据转换为ROS2消息格式  
    pcl::toROSMsg(*pointCloud, msg);  
    /// 设置消息坐标系ID和时间戳  
    msg.header.frame_id = frameId;  
    msg.header.stamp = calculateSimulatedTime();  
    /// 发布转换后的点云消息  
    publisher->publish(msg);  
    /// 释放当前点云资源  
    pointCloud.reset();  
    /// 更新当前文件索引到下一个  
    currentFileIndex++;  
    /// 记录发布成功调试信息  
    RCLCPP_DEBUG(get_logger(), "Published file %zu/%zu", currentFileIndex, fileList.size());  
}  

/**  
 * @brief        发布当前文件并切换索引  
 *  @details     发布当前文件，处理循环播放和无效文件跳过逻辑  
 *   @note        需要线程安全访问currentFileIndex和fileList  
 *   @throws      std::runtime_error 点云数据加载失败时抛出  
 *  
 * @return      bool 类型                                    数据类型: bool  
 *  @retval      true 继续播放  
 *  @retval      false 播放结束或错误  
 **/  
bool PCDtoPointCloud2Publisher::publishCurrentFile()  
{  
    /// 检查当前点云是否已加载  
    if (!pointCloud)  
    {  
        /// 检查是否超出文件列表范围  
        if (currentFileIndex >= fileList.size())  
        {  
            /// 处理循环播放模式  
            if (playLoop)  
            {  
                currentFileIndex = 0; // 重置索引  
            }  
            else  
            {  
                RCLCPP_INFO(get_logger(), "All files played, exiting...");  
                return false; // 结束播放  
            }  
        }  
        /// 获取当前文件路径  
        std::string currentFile = fileList[currentFileIndex];  
        /// 加载当前文件内容  
        if (!loadSingleFile(currentFile))  
        {  
            currentFileIndex++; // 跳过无效文件  
            return publishCurrentFile(); // 递归处理下一个文件  
        }  
    }  
    /// 创建ROS2 PointCloud2消息实例  
    sensor_msgs::msg::PointCloud2 msg;  
    /// 将PCL点云数据转换为ROS2消息格式  
    pcl::toROSMsg(*pointCloud, msg);  
    /// 应用加速后的时间戳  
    msg.header.stamp = calculateSimulatedTime();  
    /// 发布转换后的点云消息  
    publisher->publish(msg);  
    /// 释放当前点云资源  
    pointCloud.reset();  
    /// 更新到下一个文件索引  
    currentFileIndex++;  
    /// 记录发布成功调试信息  
    RCLCPP_DEBUG(get_logger(), "Published file %zu/%zu", currentFileIndex, fileList.size());  
    /// 成功切换到下一文件  
    return true; 
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
 * @return      void                          数据类型: void  
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
    auto period = std::chrono::duration<double>(1.0 / playRate);  
    /// 记录新周期配置信息  
    RCLCPP_INFO(get_logger(), "New timer period: %.9f seconds (playRate=%.1f)", period.count(), playRate);  
    /// 创建新的固定频率定时器并绑定发布回调  
    timer = create_wall_timer(period, [this]() { this->publishPointCloud(); });  
    /// 检查定时器创建是否成功  
    if (!timer)  
    {  
        RCLCPP_FATAL(get_logger(), "Failed to create timer!");  
        exit(EXIT_FAILURE); // 程序异常终止  
    }  
}  

/**  
 * @brief        按数字后缀加载分包模式文件  
 *  @details     根据文件名前缀和数字后缀筛选并排序文件  
 *   @throws     std::invalid_argument 当文件夹路径无效时抛出  
 *   @note       仅处理符合 "prefix_<数字>.pcd" 格式的文件  
 *               文件名数字后缀必须为整数类型  
 *  
 * @param       参数名称: folderPath                          数据类型: const std::string&  
 *  @details     需要加载的文件夹路径（必须存在）  
 *  
 * @return      void 类型                                    数据类型: void  
 *  @retval      无返回值，通过成员变量 fileList 存储结果  
 **/  
void PCDtoPointCloud2Publisher::loadSplitFiles(const std::string& folderPath)  
{  
    /// 验证文件夹路径有效性  
    if (!std::filesystem::exists(folderPath))  
    {  
        throw std::invalid_argument("Invalid folder path: " + folderPath);  
    }  
    /// 构造符合分包模式的正则表达式  
    std::regex pattern(folderOptionsSplitFilePrefix + "(\\d+)\\.pcd");  
    /// 清空现有文件列表  
    fileList.clear();  
    /// 遍历文件夹中的所有文件  
    for (const auto& entry : std::filesystem::directory_iterator(folderPath))  
    {  
        /// 提取文件名进行格式匹配  
        std::string filename = entry.path().filename().string();  
        if (std::regex_search(filename, pattern))  
        {  
            /// 将匹配的文件路径添加到列表  
            fileList.push_back(entry.path().string());  
        }  
    }  
    /// 按数字后缀升序排序文件列表  
    std::sort(fileList.begin(), fileList.end(), [this, &pattern](const std::string& a, const std::string& b) -> bool  
    {  
        std::smatch matchA, matchB;  
        std::regex_search(a, matchA, pattern);  
        std::regex_search(b, matchB, pattern);  
        return std::stoll(matchA[1]) < std::stoll(matchB[1]);  
    });  
    /// 构造加载信息日志  
    std::ostringstream oss;  
    oss << "Loaded " << fileList.size() << " files in split mode (prefix: " << folderOptionsSplitFilePrefix << "): ";  
    for (size_t i = 0; i < std::min(fileList.size(), size_t(5)); ++i)  
    {  
        oss << fileList[i] << ", ";  
    }  
    if (!fileList.empty())  
    {  
        /// 移除最后一个逗号和空格  
        oss.seekp(-2, oss.cur);  
    }  
    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());  
    /// 初始化分包模式起始索引  
    currentSplitIndex = folderOptionsSplitStartIndex;  
}
/**  
 * @brief        按文件名字典序加载顺序模式文件  
 *  @details     遍历文件夹并按文件名自然排序加载PCD文件  
 *   @throws      std::invalid_argument 当文件夹路径无效时抛出  
 *   @note        仅加载扩展名为.pcd的文件  
 *  
 * @param       参数名称: folderPath                          数据类型: const std::string&  
 *  @details     需要加载的文件夹路径（必须存在）  
 *  
 * @return      bool 类型                                      数据类型: bool  
 *  @retval      true 加载成功，false 文件夹无效或无PCD文件  
 **/  
bool PCDtoPointCloud2Publisher::loadSequentialFiles(const std::string& folderPath)  
{  
    /// 检查文件夹是否存在  
    if (!std::filesystem::exists(folderPath))  
    {  
        throw std::invalid_argument("Invalid folder path: " + folderPath);  
    }  
    /// 清空现有文件列表  
    fileList.clear();  
    /// 遍历文件夹中的所有文件  
    for (const auto& entry : std::filesystem::directory_iterator(folderPath))  
    {  
        /// 过滤仅PCD格式文件  
        if (entry.path().extension() == ".pcd")  
        {  
            /// 将有效文件路径添加到列表  
            fileList.push_back(entry.path().string());  
        }  
    }  
    /// 按字典序排序文件列表  
    std::sort(fileList.begin(), fileList.end());  
    /// 构造加载信息日志  
    std::ostringstream oss;  
    oss << "Loaded " << fileList.size() << " files in sequential mode: ";  
    for (size_t i = 0; i < std::min(fileList.size(), size_t(5)); ++i)  
    {  
        oss << fileList[i] << ", ";  
    }  
    if (!fileList.empty())  
    {  
        /// 移除最后一个逗号和空格  
        oss.seekp(-2, oss.cur);  
    }  
    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());  
    /// 返回文件列表是否非空  
    return !fileList.empty();  
}  

/**  
 * @brief        异步预加载下一个文件  
 *  @details     使用std::async在后台线程加载下一个文件  
 *   @note        需要线程安全访问currentFileIndex和fileList  
 *                通过文件列表锁保护数据访问  
 *  
 * @param       参数名称: 无参数                              数据类型: 无  
 * @return      void 类型                                    数据类型: void  
 **/  
void PCDtoPointCloud2Publisher::preloadNextFile()  
{  
    /// 检查是否需要预加载下一个文件  
    if (currentFileIndex + 1 < fileList.size())  
    {  
        /// 获取下一个文件的索引和路径  
        size_t nextIndex = currentFileIndex + 1;  
        std::string nextFile = fileList[nextIndex];  
        /// 启动异步加载任务  
        preloadFuture = std::async(std::launch::async, [this, nextFile]() -> bool {  
            return loadSingleFile(nextFile);  
        });  
    }  
}  
/**  
 * @brief        切换到预加载的下一个文件  
 *  @details     检查预加载结果并更新当前点云  
 *   @note        必须确保在ROS主循环中调用（避免竞态条件）  
 *  
 * @return      bool 类型                                      数据类型: bool  
 *  @retval      true 预加载成功并切换成功  
 *  @retval      false 预加载失败或无更多文件  
 **/  
bool PCDtoPointCloud2Publisher::switchToNextFile()  
{  
    /// 检查预加载任务是否有效  
    if (preloadFuture.valid())  
    {  
        /// 等待异步加载完成并获取结果  
        bool success = preloadFuture.get();  
        if (success)  
        {  
            /// 更新当前文件索引并保留有效点云  
            currentFileIndex++;  
            return true;  
        }  
        else  
        {  
            /// 跳过无效文件并更新索引  
            currentFileIndex++;  
            return false;  
        }  
    }  
    return false;  
}  
/**  
 * @brief        根据播放速率计算加速后的时间戳  
 *  @details     以节点启动时间为基准，按 playRate 加速时间流逝  
 *   @note        时间戳连续性对回放一致性至关重要  
 *                需确保在发布前调用此函数  
 *  
 * @return      rclcpp::Time 类型                             数据类型: rclcpp::Time  
 *  @retval      加速后的时间戳（基于节点启动时间）  
 **/  
rclcpp::Time PCDtoPointCloud2Publisher::calculateSimulatedTime()  
{  
    /// 记录节点启动基准时间  
    static auto startTime = now();  
    /// 计算当前运行时间（秒）  
    auto elapsed = (now() - startTime).seconds();  
    /// 应用播放速率计算模拟时间  
    auto simulatedTimeSeconds = elapsed * playRate;  
    /// 生成最终时间戳  
    return startTime + rclcpp::Duration::from_seconds(simulatedTimeSeconds);  
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