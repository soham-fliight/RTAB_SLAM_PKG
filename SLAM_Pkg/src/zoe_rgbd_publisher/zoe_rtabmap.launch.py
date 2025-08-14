from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    zoe = Node(
        package='zoe_rgbd_publisher', executable='zoe_rgbd_node', output='screen',
        parameters=[{
            'video_source': 0,
            'img_size_h': 192, 'img_size_w': 320,
            'depth_scale': 1.0,
            'fx': 525.0, 'fy': 525.0, 'cx': 320.0, 'cy': 240.0,
            'frame_id': 'camera_color_optical_frame',
        }]
    )
    sync = Node(
        package='rtabmap_ros', executable='rgbd_sync', output='screen',
        parameters=[{'approx_sync': False, 'queue_size': 10}],
        remappings=[
            ('/rgb/image',       '/camera/color/image_rect'),
            ('/depth/image',     '/camera/aligned_depth_to_color'),
            ('/rgb/camera_info', '/camera/color/camera_info'),
        ]
    )
    rtab = Node(
        package='rtabmap_ros', executable='rtabmap', output='screen',
        parameters=[{
            'subscribe_rgbd': True,
            'frame_id': 'camera_color_optical_frame',
            'Rtabmap/DetectionRate': '10.0',
            'RGBD/MinDepth': '0.2',
            'RGBD/MaxDepth': '20.0',
        }]
    )
    return LaunchDescription([zoe, sync, rtab])
