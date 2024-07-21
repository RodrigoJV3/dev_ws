import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    channel_type = 'serial'
    serial_port = '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-port0'
    serial_baudrate = 115200
    frame_id = 'Lidar_motor_link'
    inverted = False
    angle_compensate = True
    scan_mode = 'Sensitivity'
    
    return LaunchDescription([


        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type':channel_type,
                         'serial_port': serial_port, 
                         'serial_baudrate': serial_baudrate, 
                         'frame_id': frame_id,
                         'scan_mode': scan_mode,
                         'inverted': inverted,
                         'angle_compensate': angle_compensate}],
            output='screen'),

    ])

# ros2 run sllidar_ros2 sllidar_node --ros-args -p channel_type:=serial -p serial_port:=/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0 -p serial_baudrate:=115200 -p frame_id:=Lidar_motor_link -p angle_compensate:=true
#ros2 service call /stop_motor std_srvs/srv/Empty
#ros2 service call /start_motor std_srvs/srv/Empty