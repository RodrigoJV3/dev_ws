import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        #v4l2 parameters

        #Resolution & CODEC
        ExecuteProcess(
            cmd=['v4l2-ctl', '--set-fmt-video=width=320,height=240,pixelformat=YUYV'],
            shell=True,
            output='screen',
            name='v4l2_set_fmt_video'
        ),
        
        #FPS
        ExecuteProcess(
            cmd=['v4l2-ctl', '--set-parm=10'],
            shell=True,
            output='screen',
            name='v4l2_set_parm'
        ),
        
        #Bitrate
        ExecuteProcess(
            cmd=['v4l2-ctl', '--set-ctrl=video_bitrate=5000000'],
            shell=True,
            output='screen',
            name='v4l2_set_ctrl'
        ),

        #Image transport node with https://index.ros.org/p/ffmpeg_image_transport/github-ros-misc-utilities-ffmpeg_image_transport/#humble
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
        
        #v4l2 node from https://index.ros.org/r/v4l2_camera/
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