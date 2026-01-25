import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import launch_ros


def generate_launch_description():
    """Launch file for humanoid visualization nodes."""
    
    # Get script directory - use absolute path from home directory
    # Scripts are in ~/shibo_intern_ws/src/SuperOdom/script/
    home_dir = os.path.expanduser('~')
    script_dir = os.path.join(
        home_dir,
        'shibo_intern_ws',
        'src',
        'SuperOdom',
        'script'
    )
    
    lowstate_converter_script = os.path.join(script_dir, 'lowstate_to_joint_states_node.py')
    tf_broadcaster_script = os.path.join(script_dir, 'head_to_pelvis_tf_broadcaster.py')
    
    # Verify scripts exist
    if not os.path.exists(lowstate_converter_script):
        raise FileNotFoundError(f"Script not found: {lowstate_converter_script}")
    if not os.path.exists(tf_broadcaster_script):
        raise FileNotFoundError(f"Script not found: {tf_broadcaster_script}")
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    lowstate_topic_arg = DeclareLaunchArgument(
        'lowstate_topic',
        default_value='/lowstate',
        description='Topic for lowstate messages'
    )
    
    joint_states_topic_arg = DeclareLaunchArgument(
        'joint_states_topic',
        default_value='/joint_states',
        description='Topic for joint states output'
    )
    
    state_estimation_topic_arg = DeclareLaunchArgument(
        'state_estimation_topic',
        default_value='/state_estimation',
        description='Topic for state estimation (head pose)'
    )
    
    # Lowstate to JointState converter node (run as Python script)
    # Use python3.10 explicitly for ROS2 Humble compatibility
    lowstate_converter_node = ExecuteProcess(
        cmd=[
            '/usr/bin/python3.10', lowstate_converter_script,
            '--ros-args',
            '-p', ['use_sim_time:=', LaunchConfiguration('use_sim_time')],
            '-p', ['lowstate_topic:=', LaunchConfiguration('lowstate_topic')],
            '-p', ['joint_states_topic:=', LaunchConfiguration('joint_states_topic')],
            '-p', 'frame_id:=base_link',
        ],
        output='screen',
    )
    
    # Head to Pelvis TF broadcaster node (run as Python script)
    # Use python3.10 explicitly for ROS2 Humble compatibility
    tf_broadcaster_node = ExecuteProcess(
        cmd=[
            '/usr/bin/python3.10', tf_broadcaster_script,
            '--ros-args',
            '-p', ['use_sim_time:=', LaunchConfiguration('use_sim_time')],
            '-p', ['state_estimation_topic:=', LaunchConfiguration('state_estimation_topic')],
            '-p', 'map_frame:=map',
            '-p', 'world_frame:=world',
            '-p', 'head_frame:=head_link',
            '-p', 'pelvis_frame:=pelvis',
        ],
        output='screen',
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        lowstate_topic_arg,
        joint_states_topic_arg,
        state_estimation_topic_arg,
        lowstate_converter_node,
        tf_broadcaster_node,
    ])
