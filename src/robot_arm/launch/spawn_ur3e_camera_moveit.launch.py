from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    ld = LaunchDescription()
    gazebo_launch_file = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
    )

    moveit_config = (
        MoveItConfigsBuilder("custom_robot", package_name="ur3e_camera_moveit_config")
        .robot_description(file_path="config/ur.urdf.xacro")
        .robot_description_semantic(file_path="config/ur.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description= True, publish_robot_description_semantic=True, publish_planning_scene=True
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )


    x_arg = DeclareLaunchArgument('x', default_value='0', description='X position of the robot')
    y_arg = DeclareLaunchArgument('y', default_value='0', description='Y position of the robot')
    z_arg = DeclareLaunchArgument('z', default_value='0', description='Z position of the robot')
    target_color_arg = DeclareLaunchArgument(
        'target_color',
        default_value='red',
        description='Color to pick: red, blue, or yellow',
    )
    run_pick_and_place_arg = DeclareLaunchArgument(
        'run_pick_and_place',
        default_value='true',
        description='Whether to launch the pick_and_place_node',
    )

    # Include Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
            'debug': 'false',
            'gui': 'true',
            'paused': 'false',
            #'world' : world_file
        }.items()
    )

    rviz_config_path = os.path.join(
        get_package_share_directory("ur3e_camera_moveit_config"),
        "config",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            {"use_sim_time": True},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    # spawn the robot
    table_sdf = os.path.join(get_package_share_directory('robot_arm'), 'models', 'table.sdf')
    red_bin_sdf = os.path.join(get_package_share_directory('robot_arm'), 'models', 'red_bin.sdf')
    blue_bin_sdf = os.path.join(get_package_share_directory('robot_arm'), 'models', 'blue_bin.sdf')
    yellow_bin_sdf = os.path.join(get_package_share_directory('robot_arm'), 'models', 'yellow_bin.sdf')
    red_block_sdf = os.path.join(get_package_share_directory('robot_arm'), 'models', 'red_block.sdf')
    blue_block_sdf = os.path.join(get_package_share_directory('robot_arm'), 'models', 'blue_block.sdf')
    yellow_block_sdf = os.path.join(get_package_share_directory('robot_arm'), 'models', 'yellow_block.sdf')

    spawn_table = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'work_table', '-file', table_sdf, '-x', '0', '-y', '0', '-z', '0'],
        output='screen',
    )

    spawn_red_bin = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'red_bin', '-file', red_bin_sdf, '-x', '-0.31', '-y', '-0.015', '-z', '0.75'],
        output='screen',
    )

    spawn_blue_bin = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'blue_bin', '-file', blue_bin_sdf, '-x', '-0.21', '-y', '-0.015', '-z', '0.75'],
        output='screen',
    )

    spawn_yellow_bin = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'yellow_bin', '-file', yellow_bin_sdf, '-x', '-0.11', '-y', '-0.015', '-z', '0.75'],
        output='screen',
    )

    spawn_red_block = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'red_block', '-file', red_block_sdf, '-x', '-0.39', '-y', '0.145', '-z', '0.765'],
        output='screen',
    )

    spawn_blue_block = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'blue_block', '-file', blue_block_sdf, '-x', '-0.27', '-y', '0.145', '-z', '0.765'],
        output='screen',
    )

    spawn_yellow_block = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'yellow_block', '-file', yellow_block_sdf, '-x', '-0.15', '-y', '0.145', '-z', '0.765'],
        output='screen',
    )

    spawn_the_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'cobot',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z')
        ],
        output='screen',
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    use_sim_time={"use_sim_time": True}
    config_dict = moveit_config.to_dict()
    config_dict.update(use_sim_time)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[config_dict],
        arguments=["--ros-args", "--log-level", "info"],
    )

    planner_node = Node(
        package="my_arm_planning_py",
        executable="rrtconnect_planning",
        output="screen",
        parameters=[
            use_sim_time,
            {"use_rrt_fallback": True},
            {"enable_state_validity_checks": False},
        ],
    )

    stream_camera_node = Node(
        package="opencv_perception",
        executable="stream_camera",
        output="screen",
        parameters=[use_sim_time],
    )

    pick_and_place_node = Node(
        package="robot_arm",
        executable="pick_and_place_node",
        output="screen",
        parameters=[
            use_sim_time,
            {"target_color": LaunchConfiguration("target_color")},
        ],
        condition=IfCondition(LaunchConfiguration("run_pick_and_place")),
    )

    delay_spawn_robot = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[spawn_the_robot],
        )
    )

    delay_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_the_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_spawn_table_objects = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_table,
            on_exit=[
                spawn_red_bin,
                spawn_blue_bin,
                spawn_yellow_bin,
                spawn_red_block,
                spawn_blue_block,
                spawn_yellow_block,
            ],
        )
    )

    delay_arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_trajectory_controller_spawner],
        )
    )

    delay_gripper_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    delay_rviz_node = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[rviz_node],
        )
    )

    delay_perception_and_pick = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_controller_spawner,
            on_exit=[stream_camera_node, pick_and_place_node],
        )
    )

    # Launch Description
    ld.add_action(x_arg)
    ld.add_action(y_arg)
    ld.add_action(z_arg)
    ld.add_action(target_color_arg)
    ld.add_action(run_pick_and_place_arg)
    ld.add_action(gazebo)
    ld.add_action(spawn_table)
    ld.add_action(robot_state_publisher)
    ld.add_action(delay_spawn_robot)
    ld.add_action(delay_spawn_table_objects)
    delay_move_group_and_planner = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_controller_spawner,
            on_exit=[move_group_node, planner_node],
        )
    )
    # delay of the controllers
    ld.add_action(delay_joint_state_broadcaster)
    ld.add_action(delay_arm_controller)
    ld.add_action(delay_gripper_controller)
    ld.add_action(delay_move_group_and_planner)
    ld.add_action(delay_rviz_node)
    ld.add_action(delay_perception_and_pick)
    return ld
