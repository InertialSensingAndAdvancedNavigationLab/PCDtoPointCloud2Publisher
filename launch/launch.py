# ROS2核心启动模块
import launch
# ROS2节点启动相关类
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # PCDtoPointCloud2Publisher
    winyunq_publisher_node = Node(
        package='pcd_to_cloud',  # @param 包名：节点所在ROS2包
        executable='PCDtoPointCloud2Publisher',  # @param 可执行文件名：编译后的节点可执行文件
        name='PCDtoPointCloud2Publisher',  # @param 节点名称：ROS2节点在图中的标识名
        output='screen',  # @param 输出方式：将日志输出到终端
        emulate_tty=True,  # @param 终端仿真：保留终端颜色输出
        parameters=[  # @param 配置参数：节点运行时参数（示例）
        PathJoinSubstitution([  # 从YAML加载默认参数
                    FindPackageShare('pcd_to_cloud'),
                    'config',
                    'Default.yaml'
                ])
        ]
    )

    # 返回启动描述对象
    return launch.LaunchDescription([
        winyunq_publisher_node  # @details 启动WinyunqPublisher节点
    ])