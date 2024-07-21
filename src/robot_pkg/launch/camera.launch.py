from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # republish using image_transport
         Node(
             package='image_transport',
             executable='republish',
             name='image_transport',
             output='screen',
             arguments=[
                 'raw',
                 'in:=/image_raw'  # in:=<in_base_topic>
             ],
         ),
        
        # v4l2_camera node
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            output='screen',
            parameters=[
                {'image_size': [320, 240]},
                {'camera_frame_id': 'Camera_optical_link'},
                {'pixel_format': 'YUYV'},
                {'output_encoding': 'rgb8'}
            ]
        ),
    ])