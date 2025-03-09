from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description(): 
    larg_red = DeclareLaunchArgument('r', default_value='255', description='Red value for the status LED')
    larg_green = DeclareLaunchArgument('g', default_value='140', description='Green value for the status LED')
    larg_blue = DeclareLaunchArgument('b',default_value='50',description='Blue value for the status LED')

    # ROS-Parameter als LaunchConfiguration
    param_r = LaunchConfiguration('r')
    param_g = LaunchConfiguration('g')
    param_b = LaunchConfiguration('b')
        
    node_status_led = ExecuteProcess(
        cmd=[
            "sudo",
            "-E env",
            "\"ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY\"",
            "\"RMW_FASTRTPS_USE_SHM=$RMW_FASTRTPS_USE_SHM\"",
            "\"ROS_DOMAIN_ID=$ROS_DOMAIN_ID\"",
            "\"RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION\"",
            "\"PYTHONPATH=$PYTHONPATH\"",
            "\"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\"",
            "\"PATH=$PATH\"",
            "\"USER=$USER\"",
            "bash -c",
            "'",
            TextSubstitution(text=(
                'ros2 run robuboard set_status_led '
                '--ros-args '
                '-p r:=${r} '
                '-p g:=${g} '
                '-p b:=${b}'
            )),
            "'"

        ],
        additional_env={
            'r': param_r,
            'g': param_g,
            'b': param_b,
        },
        shell=True,
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(larg_red)
    ld.add_action(larg_green)
    ld.add_action(larg_blue)
    ld.add_action(node_status_led)

    return ld