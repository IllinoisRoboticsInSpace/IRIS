import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# detect all 36h11 tags
cfg_36h11 = {
    "image_transport": "raw",
    "family": "36h11",
    "size": 0.162,
    "max_hamming": 0,
    "z_up": True,
    "refine-pose": False        # increase the accuracy of the extracted pose
}

arducam = {
    'video_device': '/dev/video0',
    'camera_info_url': '/home/iris/.ros/camera_info/arducam.yaml',
}

def generate_launch_description():
    cam_node = ComposableNode(
        name='camera',
        namespace='v4l2',
        package='v4l2_camera', plugin='v4l2_camera::V4L2Camera',
        parameters=[arducam],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    tag_node = ComposableNode(
        name='apriltag_36h11',
        namespace='apriltag',
        package='apriltag_ros', plugin='AprilTagNode',
        remappings=[
            # This maps the 'raw' images for simplicity of demonstration.
            # In practice, this will have to be the rectified 'rect' images.
            ("/apriltag/image_rect", "/v4l2/image_raw"),
            ("/apriltag/camera_info", "/v4l2/camera_info"),
        ],
        parameters=[cfg_36h11],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    viz_node = ComposableNode(
        name='viz',
        namespace='apriltag',
        package='apriltag_viz', plugin='AprilVizNode',
        remappings=[
            # This maps the 'raw' images for simplicity of demonstration.
            # In practice, this will have to be the rectified 'rect' images.
            # ("/apriltag/image", "/apriltag/image_rect"),
            ("/apriltag/image", "/v4l2/image_raw"),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    container = ComposableNodeContainer(
        name='tag_container',
        namespace='apriltag',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[cam_node, tag_node, viz_node],
        output='screen'
    )

    return launch.LaunchDescription([container])