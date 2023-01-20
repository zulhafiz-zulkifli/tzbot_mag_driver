import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = []

    ## Package main node
    nodes.append(
        Node(
            package = 'tzbot_mag',
            executable = 'tzbot_mag_node',
            namespace = ''
        )
    )
    
    ## Driver node
    nodes.append(
        Node(
            package = 'tzbot_mag',
            executable = 'driver.py',
            namespace = ''
        )
    )

    return LaunchDescription(nodes)