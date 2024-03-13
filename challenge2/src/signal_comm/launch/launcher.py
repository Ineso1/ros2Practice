import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
      '/home/ines/Documents/challengesRos/challenge2/src/signal_comm',
      'config',
      'params.yaml'
      )
    talker_node = Node(
        package='signal_comm',
        executable='signal_generator',
        parameters = [config],
        output='screen',
        prefix = ["konsole -e"],
        emulate_tty = True
    )

    listener_node = Node(
        package='signal_comm',
        executable='process',
        output='screen',
        prefix = ["konsole -e"],
        emulate_tty = True
    )

    rqt_graph_node = Node(
        package='rqt_plot',
        executable='rqt_plot',
        output='screen'
    )

    l_d = LaunchDescription([talker_node, listener_node, rqt_graph_node])
    return l_d