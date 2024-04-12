from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # robot_ip_parameter_name = 'robot_ip'
    # load_gripper_parameter_name = 'load_gripper'
    # use_fake_hardware_parameter_name = 'use_fake_hardware'
    # fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    # use_rviz_parameter_name = 'use_rviz'

    # robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    # load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    # use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    # fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    # use_rviz = LaunchConfiguration(use_rviz_parameter_name)

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     robot_ip_parameter_name,
        #     default_value='172.16.0.2',                 ### originally this line was not here
        #     description='Hostname or IP address of the robot.'),
        # DeclareLaunchArgument(
        #     use_rviz_parameter_name,
        #     default_value='false',                       ### this was originally false
        #     description='Visualize the robot in Rviz'),
        # DeclareLaunchArgument(
        #     use_fake_hardware_parameter_name,
        #     default_value='false',
        #     description='Use fake hardware'),
        # DeclareLaunchArgument(
        #     fake_sensor_commands_parameter_name,
        #     default_value='false',
        #     description="Fake sensor commands. Only valid when '{}' is true".format(
        #         use_fake_hardware_parameter_name)),
        # DeclareLaunchArgument(
        #     load_gripper_parameter_name,
        #     default_value='true',
        #     description='Use Franka Gripper as an end-effector, otherwise, the robot is loaded '
        #                 'without an end-effector.'),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([PathJoinSubstitution(
        #         [FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py'])]),
        #     launch_arguments={robot_ip_parameter_name: robot_ip,
        #                       load_gripper_parameter_name: load_gripper,
        #                       use_fake_hardware_parameter_name: use_fake_hardware,
        #                       fake_sensor_commands_parameter_name: fake_sensor_commands,
        #                       use_rviz_parameter_name: use_rviz
        #                       }.items(),
        # ),


        ############################## THE FOLLOWING ARE MY OWN NODES ##############################

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([PathJoinSubstitution(
        #         [FindPackageShare('panda'), 'launch', 'gz.launch.py'])]),
        #     launch_arguments={use_rviz_parameter_name: use_rviz
        #                       }.items(),
        # ),

        # joint trajectory controller
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=['joint_trajectory_controller'],
        #     output='screen',
        # ),

        # # active Falcon interface
        # Node(
        #     package='cpp_pubsub',
        #     executable='position_talker',
        #     name='position_talker'
        # ),

        # # trajectory recorder node
        # Node(
        #     package='cpp_pubsub',
        #     executable='traj_recorder.py',
        #     name='traj_recorder'
        # )

        # camera viewer node
        Node(
            package='cpp_pubsub',
            executable='camera_viewer.py',
            # name='camera_viewer'
        )

        # Node(
        #     package='cpp_pubsub',
        #     executable='real_controller',
        #     name='real_controller'
        # ),
    ])
