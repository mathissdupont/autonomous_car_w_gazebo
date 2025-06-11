#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    home = os.getenv('HOME')
    sdf_file_path = os.path.join(home, 'ros2_ws', 'src', 'ahaltech_sim', 'worlds', 'model.sdf')
    urdf_file_path = os.path.expanduser('~/ros2_ws/src/ahaltech_sim/urdf/autonomous_vehicle.urdf')

    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    sonar_bridges = [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'sonar_{name}_bridge',
            arguments=[
                f'/{name}/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked'
            ],
            output='screen'
        ) for name in [
            'front_left_far_sonar',
            'front_left_middle_sonar',
            'front_right_far_sonar',
            'front_right_middle_sonar',
            'back_left_far_sonar',
            'back_left_middle_sonar',
            'back_right_far_sonar',
            'back_right_middle_sonar'
        ]
    ]

    nodes = [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='lidar_bridge',
            arguments=[
                '/center_lidar/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
            ],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='imu_bridge',
            arguments=[
                '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            ],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gps_bridge',
            arguments=[
                '/gps_sensor@sensor_msgs/msg/NavSatFix@ignition.msgs.NavSat',
            ],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_front_bridge',
            arguments=[
                '/front_camera_prius@sensor_msgs/msg/Image@ignition.msgs.Image',
            ],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_back_bridge',
            arguments=[
                '/back_camera_prius@sensor_msgs/msg/Image@ignition.msgs.Image',
            ],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_left_bridge',
            arguments=[
                '/left_camera_prius@sensor_msgs/msg/Image@ignition.msgs.Image',
            ],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_right_bridge',
            arguments=[
                '/right_camera_prius@sensor_msgs/msg/Image@ignition.msgs.Image',
            ],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='laser_fl_bridge',
            arguments=[
                '/front_left_laser/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            ],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='laser_fr_bridge',
            arguments=[
                '/front_right_laser/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            ],
            output='screen'
        ),
        *sonar_bridges,

        # Custom processing nodes
        Node(
            package='ahaltech_sim',
            executable='sensor_cleaner',
            name='sensor_cleaner',
            output='screen'
        ),
        Node(
            package='ahaltech_sim',
            executable='decision_maker.py',
            name='decision_maker',
            output='screen'
        ),

        Node(
            package='ahaltech_sim',
            executable='pid_controller.py',
            name='pid_controller',
            output='screen'
        ),
        # FRONT CAMERA
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
            '--x', '0', '--y', '-0.4', '--z', '1.4',
            '--roll', '0', '--pitch', '0.05', '--yaw', '-1.5707',
            '--frame-id', 'chassis', '--child-frame-id', 'front_camera_prius'
            ],
            output='screen'
        ),

        # BACK CAMERA
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
            '--x', '0', '--y', '1.45', '--z', '1.4',
            '--roll', '0', '--pitch', '0.05', '--yaw', '1.5707',
            '--frame-id', 'chassis', '--child-frame-id', 'back_camera_prius'
            ],
            output='screen'
        ),

        # LEFT CAMERA
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
            '--x', '1', '--y', '-0.7', '--z', '1',
            '--roll', '0', '--pitch', '0.05', '--yaw', '1.0',
            '--frame-id', 'chassis', '--child-frame-id', 'left_camera_prius'
            ],
            output='screen'
        ),

        # RIGHT CAMERA
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
            '--x', '-1', '--y', '-0.7', '--z', '1',
            '--roll', '0', '--pitch', '0.05', '--yaw', '2.1416',
            '--frame-id', 'chassis', '--child-frame-id', 'right_camera_prius'
            ],
            output='screen'
        ),

        # CENTER LIDAR
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
            '--x', '0', '--y', '0.4', '--z', '2.0',
            '--roll', '0', '--pitch', '0', '--yaw', '-1.5707',
            '--frame-id', 'chassis', '--child-frame-id', 'center_lidar'
            ],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='cmd_vel_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'
            ],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='lidar_bridge',
            arguments=[
                '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'
            ],
            output='screen'
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pc2_to_scan',
            parameters=[{
                'target_frame': 'center_lidar',
                'transform_tolerance': 0.01,
                'min_height': -0.5,
                'max_height':  0.5,
                'angle_min': -3.14,
                'angle_max':  3.14,
                'angle_increment': 0.017,
                'scan_time': 0.1,
                'range_min': 0.1,
                'range_max': 30.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'concurrency_level': 1,
            }],
            remappings=[
                ('cloud_in', '/center_lidar/points'),
                ('scan', '/scan')
            ],
            output='screen'
        ),



    ]

    return LaunchDescription(nodes)

