import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable

from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('robot_omni')

    urdf_file = os.path.join(pkg, 'urdf', 'omni_base.urdf')
    # world_file = os.path.join(pkg, 'worlds', 'hospital_aws.world')
    world_file = os.path.join(pkg, 'worlds', 'hospital_full_exam.world')
    bridge_config = os.path.join(pkg, 'config', 'bridge_config.yaml')
    controller_config = os.path.join(pkg, 'config', 'configuration.yaml')
    ekf_config = os.path.join(pkg, 'config', 'ekf.yaml')
    laser_filter_config = os.path.join(pkg, 'config', 'laser_filters.yaml')

    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Khai báo thư mục models của AWS
    models_dir = os.path.join(pkg, 'models')

    # Meshes & Models
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{os.path.dirname(pkg)}:{models_dir}"
    )
    
    # -------------------------
    # Controller YAML passed     # to gz_ros2_control
    # -------------------------
    set_ros_args = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_ARGS',
        value=f'--ros-args --params-file {controller_config}'
    )

    # -------------------------
    # Robot State Publisher
    # -------------------------

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],  
        output='screen'
    )

    # -------------------------
    # Start Gazebo
    # -------------------------

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # -------------------------
    # Spawn robot
    # -------------------------

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'omni_base',
            '-x', '0.0',
            '-y', '12.0',
            '-z', '0.1',
            '-Y', '-1.5708',
        ],
        output='screen'
    )

    delayed_spawn = TimerAction(
        period=11.0,
        actions=[spawn_robot]
    )

    # -------------------------
    # ROS <-> Gazebo Bridge
    # Uses bridge_config.yaml
    # -------------------------

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {'config_file': bridge_config},
            {'use_sim_time': True}
        ],
    )

    delayed_bridge = TimerAction(
        period=13.0,
        actions=[bridge]
    )

    # -------------------------
    # ros2_control Controllers
    # -------------------------

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--ros-args',
            '-p', 'use_sim_time:=true'
        ],
    )

    mobile_base_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'mobile_base_controller',
            '--controller-manager',
            '/controller_manager',
            '--ros-args',
            '-p', 'use_sim_time:=true'
        ],
    )

    delayed_controllers = TimerAction(
        period=20.0,
        actions=[
            joint_state_broadcaster,
            mobile_base_controller,
        ]
    )

    # EKF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}]
    )

    # lidar filter
    front_lidar_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_front_filter',
        parameters=[laser_filter_config],
        remappings=[
            ('/scan', '/scan_front_raw'),       # Topic đầu vào từ Gazebo
            ('/scan_filtered', '/scan_front')   # Topic đầu ra (đã lọc sạch)
        ],
        output='screen'
    )

    # camera depth
    depth_to_pointcloud_node = Node(
        package='depth_image_proc',
        executable='point_cloud_xyz_node',
        name='point_cloud_xyz_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('image_rect', '/camera/depth_image'),  # Ảnh depth đầu vào
            ('camera_info', '/camera/camera_info'), # Thông tin camera đầu vào
            ('points', '/camera/points')            # PointCloud đầu ra cho Nav2
        ]
    )

    # manual_control_process = ExecuteProcess(
    #     cmd=['xterm', '-e', 'python3', '/home/ubuntu24-04/ros2_ws/src/robot_omni/launch/manual_control.py'],
    #     output='screen'
    # )

    # delayed_manual_control = TimerAction(
    #     period=15.0,
    #     actions=[manual_control_process]
    # )

    # -------------------------
    # Launch everything
    # -------------------------

    return LaunchDescription([
        set_gz_resource_path,
        set_ros_args,
        robot_state_publisher,
        gz_sim,
        delayed_spawn,
        delayed_bridge,
        delayed_controllers,
        # delayed_manual_control,
        ekf_node,
        front_lidar_filter_node,
        depth_to_pointcloud_node,
    ])
