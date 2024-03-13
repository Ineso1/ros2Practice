import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
      '/home/ines/Desktop/RetoRos_1/ControllerNode/src/signalCommunication',
      'config',
      'params.yaml'
      )
    talker_node = Node(
        package='signalCommunication',
        executable='signalGenerator',
        parameters = [config],
        output='screen',
        prefix = ["konsole -e"],
        emulate_tty = True
    )
    rqt_graph_node = Node(
        package='rqt_plot',
        executable='rqt_plot',
        output='screen'
    )

    l_d = LaunchDescription([talker_node, rqt_graph_node])
    return l_d