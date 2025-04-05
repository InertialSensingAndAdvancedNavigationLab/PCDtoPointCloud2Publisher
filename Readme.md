# PCD to PointCloud2 Publisher

ROS2 节点，读取 PCD 文件或文件夹中的点云数据，并发布为 `sensor_msgs/PointCloud2` 消息。功能类似于 `rosbag` 播放，支持文件夹模式和多种播放策略。

---

## 功能实现进度树
- [ ] **核心功能**
  - [x] 单文件播放
  - [ ] 文件夹模式（顺序播放）
  - [ ] 分包模式（按前缀+数字后缀）
  - [ ] 并行模式（多传感器模拟）
  - [ ] 多线程预加载
- [ ] **高级功能**
  - [ ] 时间戳模拟（系统时间或文件时间戳）
  - [ ] 循环播放
  - [ ] 播放速率控制
- [ ] **调试与日志**
  - [ ] 日志文件输出
  - [ ] 参数校验与错误处理

---

## 仓库信息
- **GitHub 地址**: https://github.com/InertialSensingAndAdvancedNavigationLab/PCDtoPointCloud2Publisher
- **ROS2 包名**: `pcd_to_cloud`

---

## 核心功能

### 1. 文件夹模式播放
- 支持按文件名顺序播放（`sequential`）。
- 未来支持分包模式（`split`）和并行模式（`parallel`）。

### 2. 参数化配置
通过 ROS2 动态参数控制以下功能：
- 播放速率（快放/慢放）
- 循环播放
- 坐标系 ID
- 话题命名空间（如 `/sensors/lidar`）

---

## 安装步骤

### 依赖项
安装基础依赖：
sudo apt install ros--pcl-conversions sudo apt install libpcl-dev



### 构建项目
将仓库克隆到 ROS2 工作区的 `src` 文件夹：

    cd your_ros2_ws/src 
    git clone https://github.com/InertialSensingAndAdvancedNavigationLab/PCDtoPointCloud2Publisher.git

编译代码：

    cd your_ros2_ws 
    colcon build --packages-select pcd_to_cloud 
    source install/setup.bash

## 参数说明
在config中，配置yaml文件，使用

    ros2 launch pcd_to_cloud launch.py

### **基础参数**
| 参数名                | 类型   | 默认值       | 描述                                                                 | 实现进度 |
|-----------------------|--------|--------------|----------------------------------------------------------------------|----------|
| inputPath             | string | ""           | PCD 文件路径或文件夹路径（必填）                                   | [x]      |
| inputFolderMode       | bool   | false        | 启用文件夹模式（`true`）或单文件模式（`false`）                    | [x]      |

### **播放控制**
| 参数名               | 类型   | 默认值       | 描述                                                                 | 实现进度 |
|----------------------|--------|--------------|----------------------------------------------------------------------|----------|
| playRate             | double | 1.0          | 播放速率（>1 快放，<1 慢放）                                       | [ ]      |
| playLoop             | bool   | false        | 是否循环播放（单文件循环文件，文件夹循环序列）                     | [ ]      |

### **发布配置**
| 参数名             | 类型   | 默认值       | 描述                                                                 | 实现进度 |
|--------------------|--------|--------------|----------------------------------------------------------------------|----------|
| topicNamespace     | string | "point_cloud" | ROS 命名空间（如 `/sensors/lidar`）                                | [x]      |
| topicName          | string | "data"       | 话题名（与命名空间组合，如 `point_cloud/data`）                     | [x]      |
| frameId            | string | "map"        | 坐标系 ID（如 `laser_frame`）                                      | [x]      |

### **高级选项**
| 参数名                     | 类型   | 默认值       | 描述                                                                 | 实现进度 |
|----------------------------|--------|--------------|----------------------------------------------------------------------|----------|
| enableMultiThreadLoad      | bool   | false        | 启用多线程预加载下一个文件（减少延迟）                           | [ ]      |
| timestampSource            | string | "system"     | 时间戳来源：`system`（系统时间）或 `file`（文件时间戳）           | [ ]      |

---

## 贡献指南

本项目为实验室项目依赖，不一定保证长期更新，所有功能是否实现可能取决于实验是否需求。欢迎各位主动维护！

1. **代码规范**：
   - 使用 C++17 和 ROS2 标准。
   - 参数名遵循 `camelCase` 命名规范。
2. **提交 PR**：
   - 确保通过单元测试（`colcon test`）。
   - 更新 `README.md` 的进度树和参数说明。

---

## 许可证

MIT License（见 LICENSE 文件）。