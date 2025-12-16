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
    map_size_z = LaunchConfiguration('map_size_z', default=10.0)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    odom_topic = LaunchConfiguration('odom_topic', default='/lio_sam/mapping/odometry')
    cloud_topic = LaunchConfiguration('cloud_topic', default='/lidar/cloud')
    camera_pose_topic = LaunchConfiguration('camera_pose_topic', default='camera_pose')
    depth_topic = LaunchConfiguration('depth_topic', default='depth_image')

    max_vel = LaunchConfiguration('max_vel', default=3.0)
    max_acc = LaunchConfiguration('max_acc', default=4.0)
    planning_horizon = LaunchConfiguration('planning_horizon', default=8.0)
    target_mode = LaunchConfiguration('target_mode', default=1)
    use_distinctive_trajs = LaunchConfiguration('use_distinctive_trajs', default=True)

    map_size_x_arg = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_arg = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_arg = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    drone_id_arg = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')
    topic_prefix_arg = DeclareLaunchArgument('topic_prefix', default_value=topic_prefix, description='Topic prefix for namespacing; leave empty to remove drone_id prefix')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time, description='Use simulation time')
    odom_topic_arg = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry input for ego_planner')
    cloud_topic_arg = DeclareLaunchArgument('cloud_topic', default_value=cloud_topic, description='Point cloud input for ego_planner')
    camera_pose_topic_arg = DeclareLaunchArgument('camera_pose_topic', default_value=camera_pose_topic, description='Camera pose topic (if used)')
    depth_topic_arg = DeclareLaunchArgument('depth_topic', default_value=depth_topic, description='Depth image topic (if used)')
    max_vel_arg = DeclareLaunchArgument('max_vel', default_value=max_vel, description='Maximum velocity used by planner')
    max_acc_arg = DeclareLaunchArgument('max_acc', default_value=max_acc, description='Maximum acceleration used by planner')
    planning_horizon_arg = DeclareLaunchArgument('planning_horizon', default_value=planning_horizon, description='Planning horizon (seconds)')
    target_mode_arg = DeclareLaunchArgument('target_mode', default_value=target_mode, description='Target mode for ego_planner: 1=single, 2=waypoints')
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
            'use_sim_time': use_sim_time,
            'odometry_topic': odom_topic,
            'camera_pose_topic': camera_pose_topic,
            'depth_topic': depth_topic,
            'cloud_topic': cloud_topic,
            'max_vel': max_vel,
            'max_acc': max_acc,
            'planning_horizon': planning_horizon,
            'use_distinctive_trajs': use_distinctive_trajs,
            'target_mode': target_mode
        }.items()
    )

    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['traj_server_', drone_id],
        output='screen',
        remappings=[
            ('position_cmd', [topic_prefix, 'planning/position_cmd']),
            ('planning/bspline', [topic_prefix, 'planning/bspline']),
            ('plan_vis/position_cmd', [topic_prefix, 'plan_vis/position_cmd'])
        ],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'traj_server/time_forward': 1.0}
        ]
    )

    ld = LaunchDescription()

    ld.add_action(map_size_x_arg)
    ld.add_action(map_size_y_arg)
    ld.add_action(map_size_z_arg)
    ld.add_action(drone_id_arg)
    ld.add_action(topic_prefix_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(odom_topic_arg)
    ld.add_action(cloud_topic_arg)
    ld.add_action(camera_pose_topic_arg)
    ld.add_action(depth_topic_arg)
    ld.add_action(max_vel_arg)
    ld.add_action(max_acc_arg)
    ld.add_action(planning_horizon_arg)
    ld.add_action(target_mode_arg)
    ld.add_action(use_distinctive_trajs_arg)

    ld.add_action(advanced_param_include)
    ld.add_action(traj_server_node)

    return ld
