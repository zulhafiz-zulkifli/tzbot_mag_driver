import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = []

    ## Package main node
    # nodes.append(
    #     Node(
    #         package = 'tzbot_mag',
    #         executable = 'tzbot_mag_node',
    #         namespace = 'tzbot'
    #     )
    # )
    
    ## Driver node
    nodes.append(
        Node(
            package = 'tzbot_mag',
            executable = 'tzbot_mag_driver.py',
            namespace = 'tzbot_mag',
            parameters = [os.path.join(get_package_share_directory('tzbot_mag'), 'config','params.yaml')],
            emulate_tty=True
        )
    )

    return LaunchDescription(nodes)