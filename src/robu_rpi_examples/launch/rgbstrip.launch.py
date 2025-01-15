from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robu_rpi_examples',
            executable='rgbstrip_sub',
            namespace="",
            name='rgbstrip_sub',
            # Launch the node with root access (GPIO) in a shell
            prefix=["sudo -E env \"ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY\" \"RMW_FASTRTPS_USE_SHM=$RMW_FASTRTPS_USE_SHM\" \"ROS_DOMAIN_ID=$ROS_DOMAIN_ID\" \"RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION\" \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\"  bash -c "],
            shell=True,
        ),
    ])