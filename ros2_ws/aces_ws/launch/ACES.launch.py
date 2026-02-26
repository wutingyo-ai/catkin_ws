from launch import LaunchDescription           # launch文件的描述类
from launch_ros.actions import Node            # 节点启动的描述类
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
def generate_launch_description():             # 自动生成launch文件的函数
    return LaunchDescription([                 # 返回launch文件的描述信息
        
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.10.3',
            description='IP address of the robot'
        ),
        Node(                                  # 配置一个节点的启动
            package='aces_ws',          # 节点所在的功能包
            executable='Image_sub_node', # 节点的可执行文件
        ),
        Node(                                  # 配置一个节点的启动
            package='tm_get_status',          # 节点所在的功能包
            output='screen',
            executable='image_talker', # 节点的可执行文件名
        ),
        Node(                                  # 配置一个节点的启动
            package='aces_ws',          # 节点所在的功能包
            executable='Vision_node', # 节点的可执行文件名
        ),
        
        # Node(
        #     package='tm_driver',
        #     executable='tm_driver',
        #     output='screen',
        #     parameters=[{'robot_ip': LaunchConfiguration('robot_ip')}],
        # ),
    ])
