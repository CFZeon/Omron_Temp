import os
import sys
import json
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

pp_share = get_package_share_directory('pickplace')
pp_library =  pp_share + '/pickplace/pp_library'

from pp_library import Transform

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    args = []
    length = len(sys.argv)
    if (len(sys.argv) >= 5):
        i = 4
        while i < len(sys.argv):
            args.append(sys.argv[i])
            i = i + 1

    tf = Transform.TransformClass()
    
    # moveit_cpp.yaml is passed by filename for now since it's node specific
    moveit_cpp_yaml_file_name = get_package_share_directory('tmr_moveit_cpp_demo') + "/config/moveit_cpp.yaml"

    # Component yaml files are grouped in separate namespaces
    robot_description_config = load_file('tmr_description', 'urdf/tm5-900.urdf')
    robot_description = {'robot_description' : robot_description_config}

    robot_description_semantic_config = load_file('tmr_moveit_config_tm5-900', 'config/tm5-900.srdf')
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}

    kinematics_yaml = load_yaml('tmr_moveit_config_tm5-900', 'config/kinematics.yaml')
    robot_description_kinematics = { 'robot_description_kinematics' : kinematics_yaml }

    controllers_yaml = load_yaml('tmr_moveit_cpp_demo', 'config/controllers.yaml')
    moveit_controllers = { 'moveit_simple_controller_manager' : controllers_yaml,
                           'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'}

    ompl_planning_pipeline_config = { 'ompl' : {
        'planning_plugin' : 'ompl_interface/OMPLPlanner',
        'request_adapters' : """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""" ,
        'start_state_max_bounds_error' : 0.1 } }
    ompl_planning_yaml = load_yaml('tmr_moveit_config_tm5-900', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    # Component yaml files are grouped in separate namespaces
    #robot_description_config = load_file('tmr_description', 'urdf/tm5-900.urdf')
    #robot_description_config = load_file('tm_models', 'urdf/tm12.urdf')
    #robot_description = {'robot_description' : robot_description_config}

    pp_config = get_package_share_directory('pickplace') + '/config.txt'
    view_pick = []
    view_place = []
    with open(pp_config) as json_file:
        data = json.load(json_file)
        view_pick =  data['view_pick']
        view_place =  data['view_place']
    view_pick = tf.rpy_to_ypr(view_pick)
    view_place = tf.rpy_to_ypr(view_place)
    view_pick = [str(i) for i in view_pick] + ['base', 'view_pick']
    view_place = [str(i) for i in view_place] + ['base', 'view_place']
  

    # RViz
    rviz_config_file = get_package_share_directory('tmr_moveit_cpp_demo') + "/launch/run_moveit_cpp.rviz"
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic]
        )

    # Static TF
    static_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base']
    )

    static_viewpick = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='viewpick_publisher',
        output='log',
        arguments= view_pick
    )
    
    static_viewplace = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='viewplace_publisher',
        output='log',
        arguments= view_place
    )
    
    tm_driver_node = Node(
        package='tm_driver',
        executable='tm_driver',
        #name='tm_driver',
        output='screen',
        arguments=[str(args)[12:-2]],
        #prefix=["bash -c 'sleep 6.0; $0 $@' "]
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
        #prefix="bash -c 'sleep 5.0; $0 $@'"
    )

    # joint driver
    tmr_driver_node = Node(
        package='tmr_driver',
        executable='tmr_driver',
        #name='tmr_driver',
        output='screen',
        arguments=args
    )
        

    # Pickplace Program
    pickplace_node = Node(
        package='pickplace',
        executable='pickplace_moveit',
        output='screen'
    )

    # Marker Publisher
    marker_publisher_node = Node(
        package='pp_marker',
        executable='marker',
        output='screen'
    )
    
    # Destination Publisher
    destination_publisher_node = Node(
        package='pickplace',
        executable='destination_publisher',
        output='screen'
    )

    modbus_server_node = Node(
        package='pickplace',
        executable='modbus_server',
        output='screen',
    )
    
    static_viewpick = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='viewpick_publisher',
        output='log',
        arguments= view_pick
    )
    
    static_viewplace = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='viewplace_publisher',
        output='log',
        arguments= view_place
    )
    
    moveit_server_node = Node(
        package='tmr_moveit_cpp_demo',
        executable='run_moveit_cpp',
        output='screen',
        parameters=[
            moveit_cpp_yaml_file_name,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            moveit_controllers
        ]
    )

    return LaunchDescription([ #tmr_driver_node, 
        pickplace_node, robot_state_publisher, tm_driver_node,
        static_world, rviz_node, marker_publisher_node, destination_publisher_node, modbus_server_node,
        static_viewpick, static_viewplace, moveit_server_node])

