import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    drone_id = LaunchConfiguration('drone_id', default=0)
    topic_prefix = LaunchConfiguration('topic_prefix', default='')

    map_size_x = LaunchConfiguration('map_size_x', default=60.0)
    map_size_y = LaunchConfiguration('map_size_y', default=40.0)
    map_size_z = LaunchConfiguration('map_size_z', default=5.0)

    odom_topic = LaunchConfiguration('odom_topic', default='/lio_sam/mapping/odometry')
    cloud_topic = LaunchConfiguration('cloud_topic', default='/unilidar/cloud')
    camera_pose_topic = LaunchConfiguration('camera_pose_topic', default='camera_pose')
    depth_topic = LaunchConfiguration('depth_topic', default='depth_image')

    max_vel = LaunchConfiguration('max_vel', default=3.0)
    max_acc = LaunchConfiguration('max_acc', default=4.0)
    planning_horizon = LaunchConfiguration('planning_horizon', default=8.0)
    flight_type = LaunchConfiguration('flight_type', default=2)
    use_distinctive_trajs = LaunchConfiguration('use_distinctive_trajs', default=True)

    map_size_x_arg = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_arg = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_arg = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    drone_id_arg = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')
    topic_prefix_arg = DeclareLaunchArgument('topic_prefix', default_value=topic_prefix, description='Topic prefix for namespacing; leave empty to remove drone_id prefix')
    odom_topic_arg = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry input for ego_planner')
    cloud_topic_arg = DeclareLaunchArgument('cloud_topic', default_value=cloud_topic, description='Point cloud input for ego_planner')
    camera_pose_topic_arg = DeclareLaunchArgument('camera_pose_topic', default_value=camera_pose_topic, description='Camera pose topic (if used)')
    depth_topic_arg = DeclareLaunchArgument('depth_topic', default_value=depth_topic, description='Depth image topic (if used)')
    max_vel_arg = DeclareLaunchArgument('max_vel', default_value=max_vel, description='Maximum velocity used by planner')
    max_acc_arg = DeclareLaunchArgument('max_acc', default_value=max_acc, description='Maximum acceleration used by planner')
    planning_horizon_arg = DeclareLaunchArgument('planning_horizon', default_value=planning_horizon, description='Planning horizon (seconds)')
    flight_type_arg = DeclareLaunchArgument('flight_type', default_value=flight_type, description='Flight type mode for ego_planner')
    use_distinctive_trajs_arg = DeclareLaunchArgument('use_distinctive_trajs', default_value=use_distinctive_trajs, description='Allow distinctive trajectories')

    advanced_param_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')
        ),
        launch_arguments={
            'drone_id': drone_id,
            'topic_prefix': topic_prefix,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'odometry_topic': odom_topic,
            'camera_pose_topic': camera_pose_topic,
            'depth_topic': depth_topic,
            'cloud_topic': cloud_topic,
            'max_vel': max_vel,
            'max_acc': max_acc,
            'planning_horizon': planning_horizon,
            'use_distinctive_trajs': use_distinctive_trajs,
            'flight_type': flight_type
        }.items()
    )

    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['traj_server_', drone_id],
        output='screen',
        remappings=[
            ('position_cmd', [topic_prefix, 'planning/pos_cmd']),
            ('planning/bspline', [topic_prefix, 'planning/bspline'])
        ],
        parameters=[
            {'traj_server/time_forward': 1.0}
        ]
    )

    ld = LaunchDescription()

    ld.add_action(map_size_x_arg)
    ld.add_action(map_size_y_arg)
    ld.add_action(map_size_z_arg)
    ld.add_action(drone_id_arg)
    ld.add_action(topic_prefix_arg)
    ld.add_action(odom_topic_arg)
    ld.add_action(cloud_topic_arg)
    ld.add_action(camera_pose_topic_arg)
    ld.add_action(depth_topic_arg)
    ld.add_action(max_vel_arg)
    ld.add_action(max_acc_arg)
    ld.add_action(planning_horizon_arg)
    ld.add_action(flight_type_arg)
    ld.add_action(use_distinctive_trajs_arg)

    ld.add_action(advanced_param_include)
    ld.add_action(traj_server_node)

    return ld
