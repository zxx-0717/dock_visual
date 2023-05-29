import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    
    launch_description = LaunchDescription()

    # get pkg path
    camera_pkg_path = get_package_share_directory('astra_camera')
    aruco_pkg_path = get_package_share_directory('aruco_ros')
    dock_pkg_path = get_package_share_directory('dock_visual')

    # create launch configuration variables
    params_file_path = LaunchConfiguration('params_file', default=os.path.join(dock_pkg_path, 'param', 'dock.yaml'))
    motion_control_log_level = LaunchConfiguration('motion_control_log_level')
    test_count = LaunchConfiguration('test_count', default = 1)
    
    # declare launch arguments   
    # test_count_launch_arg = DeclareLaunchArgument('test_count', default_value=TextSubstitution(text="1"))
    log_level_arg = DeclareLaunchArgument('log_level', default_value='info', description='define motion_control node log level')
    params_file_arg = DeclareLaunchArgument('params_filess', default_value=params_file_path)
    
    # configured params file
    param_substitutions = {
        "test_count": test_count,
    }

    configured_params = RewrittenYaml(
        source_file=params_file_path,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # serial Node
    serial_node = Node(
        executable='serial_port_node',
        package='capella_ros_serial',
        name='serial_node',
        namespace=''
    )

    # wifi Node
    wifi_node = Node(
        executable='charge_server_node',
        package='capella_charge_service',
        name='wifi_node'
    )

    # camera(orbbec dabai_dcw) launch file
    camera_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(camera_pkg_path, 'launch', 'dabai_dcw.launch.py'))
    )

    # aruco launch file
    aruco_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(aruco_pkg_path, 'launch', 'single.launch.py'))
    )

    # motion_control Node
    motion_control_node = Node(
        executable='motion_control',
        package='dock_visual',
        name='motion_control',
        namespace='',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', ['motion_control:=', LaunchConfiguration('log_level')]]
    )

    # test docking Node
    test_docking_node = Node(
        executable='test_dock',
        package='dock_visual',
        name='test_dock',
        parameters=[configured_params],
    )

    # launch_description.add_action(test_count_launch_arg)
    launch_description.add_action(log_level_arg)
    launch_description.add_action(params_file_arg)
    launch_description.add_action(serial_node)
    launch_description.add_action(wifi_node)
    launch_description.add_action(camera_launch_file)
    launch_description.add_action(aruco_launch_file)
    launch_description.add_action(motion_control_node)
    launch_description.add_action(test_docking_node)

    return launch_description
