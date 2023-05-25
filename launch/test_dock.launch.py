import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution

def generate_launch_description():
    
    launch_description = LaunchDescription()

    # get pkg path
    camera_pkg_path = get_package_share_directory('astra_camera')
    aruco_pkg_path = get_package_share_directory('aruco_ros')
    
    # declare launch arguments   
    test_count_launch_arg = DeclareLaunchArgument('test_count', default_value=TextSubstitution(text="1"))
    
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
        namespace=''
    )

    # test docking Node
    test_docking_node = Node(
        executable='test_dock_node',
        package='dock_visual',
        name='test_docking',
        namespace='',
        parameters=[
            {'test_count': LaunchConfiguration('test_count'),}
        ]
    )

    launch_description.add_action(test_count_launch_arg)
    launch_description.add_action(serial_node)
    launch_description.add_action(wifi_node)
    launch_description.add_action(camera_launch_file)
    launch_description.add_action(aruco_launch_file)
    launch_description.add_action(motion_control_node)
    launch_description.add_action(test_docking_node)

    return launch_description



    
