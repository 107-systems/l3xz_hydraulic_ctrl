from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='l3xz_pump_ctrl',
      namespace='l3xz',
      executable='l3xz_pump_ctrl_node',
      name='l3xz_pump_ctrl',
      output='screen',
      emulate_tty=True,
      parameters=[
        {'pump_min_rpm' :  800.0},
        {'pump_max_rpm' : 1800.0},
      ]
    )
  ])
