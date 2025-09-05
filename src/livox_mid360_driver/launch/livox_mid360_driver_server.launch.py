from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(
        os.path.dirname(__file__),
        '..',
        'config',
        'livox_mid360_driver.yaml'
    )

    node = Node(
        package='livox_mid360_driver',
        executable='server',   # deve combaciare con add_executable in CMakeLists
        name='server',         # deve combaciare col Node("server")
        parameters=[config]    # <-- qui passa il file yaml
        )
    ld.add_action(node)
    return ld