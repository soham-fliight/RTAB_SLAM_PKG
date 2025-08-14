# bringup/zoe_rtabmap.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    zoe = Node(
        package='zoe_rgbd_publisher', executable='zoe_rgbd_node', output='screen',
        parameters=[{
            'video_source': 0,                 # webcam index or file path
            'img_size_h': 384, 'img_size_w': 512,
            'depth_scale': 1.0,
            'fx': 525.0, 'fy': 525.0, 'cx': 320.0, 'cy': 240.0,
            'frame_id': 'camera_color_optical_frame',
        }]
    )

    sync = Node(
        package='rtabmap_sync', executable='rgbd_sync', output='screen',
        parameters=[{
            'approx_sync': True,               # exact→False requires identical stamps
            'topic_queue_size': 50,
            'sync_queue_size': 50
        }],
        remappings=[
            ('/rgb/image',       '/camera/color/image_rect'),
            ('/depth/image',     '/camera/aligned_depth_to_color'),
            ('/rgb/camera_info', '/camera/color/camera_info'),
        ]
    )

    odom = Node(
        package='rtabmap_odom', executable='rgbd_odometry', output='screen',
        parameters=[{
            'frame_id': 'camera_color_optical_frame',
            'odom_frame_id': 'odom',
            'publish_tf': True
        }],
        remappings=[('/rgbd_image', '/rgbd_image')]
    )

    rtab = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=['rtabmap_rgbd.yaml']  # see file below
    )

    return LaunchDescription([zoe, sync, odom, rtab])
