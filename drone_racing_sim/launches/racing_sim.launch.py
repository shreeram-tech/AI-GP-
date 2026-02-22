"""
Launch file for drone racing simulation

Launches:
  - Gazebo simulator
  - Drone model in Gazebo (via spawn_entity)
  - ROS2 nodes: gate spawner, autonomy bridge, validation, visualization, logging
  - RViz2 for visualization
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
import xacro


def generate_launch_description():
    
    # Use Gazebo Fortress (compatible with ROS2 Humble)
    gazebo_resource_path = os.environ.get('GAZEBO_RESOURCE_PATH', '')
    
    # World file
    world_file = os.path.expanduser(
        '~/aigp/drone_racing_sim/worlds/racing_track.world'
    )
    
    # Generate URDF from Xacro for drone
    drone_urdf_file = os.path.expanduser(
        '~/aigp/drone_racing_sim/urdf/quadrotor.urdf.xacro'
    )
    
    # Process URDF with xacro
    doc = xacro.process_file(drone_urdf_file)
    drone_urdf_xml = doc.toxml()
    
    # RViz config file
    rviz_config = os.path.expanduser(
        '~/aigp/drone_racing_sim/configs/rviz_config.rviz'
    )
    
    return LaunchDescription([
        
        # ===== GAZEBO =====
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'ignition-gazebo-physics-plugin', 
                 '-s', 'ignition-gazebo-sensors-system', world_file],
            output='screen',
            name='gazebo'
        ),
        
        #===== WAIT FOR GAZEBO =====
        TimerAction(
            period=3.0,  # Wait 3 seconds for Gazebo to start
            actions=[
                # ===== SPAWN DRONE =====
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'quadrotor',
                        '-x', '0',
                        '-y', '0',
                        '-z', '1',
                        '-topic', '/robot_description'
                    ],
                    output='screen'
                ),
            ]
        ),
        
        # Publish drone URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[drone_urdf_file],
            output='screen'
        ),
        
        # ===== GATE SPAWNER =====
        Node(
            package='drone_racing_core',
            executable='gate_spawner',
            output='screen',
            parameters=[
                {'num_gates': 8},
                {'track_radius': 50.0},
                {'track_height': 10.0},
                {'randomize': False}
            ]
        ),
        
        # ===== AUTONOMY BRIDGE =====
        Node(
            package='drone_racing_core',
            executable='autonomy_bridge',
            output='screen',
            parameters=[
                {'autonomy_config': 'default'},
                {'control_frequency': 50.0}
            ]
        ),
        
        # ===== GATE VALIDATOR =====
        Node(
            package='drone_racing_core',
            executable='gate_validator',
            output='screen',
            parameters=[
                {'num_gates': 8},
                {'enable_strict_sequence': True}
            ]
        ),
        
        # ===== VISUALIZATION =====
        Node(
            package='drone_racing_core',
            executable='visualization',
            output='screen',
            parameters=[
                {'num_gates': 8},
                {'track_radius': 50.0}
            ]
        ),
        
        # ===== METRICS LOGGER =====
        Node(
            package='drone_racing_core',
            executable='metrics_logger',
            output='screen',
            parameters=[
                {'enable_csv': True},
                {'enable_json': True}
            ]
        ),
        
        # ===== RVIZ2 =====
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
            output='screen'
        )
    ])
