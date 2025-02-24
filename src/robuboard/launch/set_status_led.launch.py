from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'r',
            default_value='255',
            description='Red value for the status LED'
        ),
        DeclareLaunchArgument(
            'g',
            default_value='255',
            description='Green value for the status LED'
        ),
        DeclareLaunchArgument(
            'b',
            default_value='51',
            description='Blue value for the status LED'
        ),
        Node(
            package='robuboard',
            executable='set_status_led',
            namespace="",
            name='set_status_led',
            # Launch the node with root access (GPIO) in a shell
            prefix=["sudo -E env \"ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY\" \"RMW_FASTRTPS_USE_SHM=$RMW_FASTRTPS_USE_SHM\" \"ROS_DOMAIN_ID=$ROS_DOMAIN_ID\" \"RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION\" \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\"  bash -c "],
            shell=True,
            parameters=[
                {"r": LaunchConfiguration('r')},
                {"g": LaunchConfiguration('g')},
                {"b": LaunchConfiguration('b')},
            ],
        ),
    ])