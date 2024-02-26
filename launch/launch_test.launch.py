from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.actions import (ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.substitutions import FindExecutable

import os
from ament_index_python.packages import get_package_share_directory
import numpy as np


def generate_launch_description():
     
    wall_follower = Node(
        package="wall_follower",
        executable="wall_follower",
        namespace='wall_follower_ns',
        parameters=[
            {"scan_topic": "/scan",
            "drive_topic": "/drive",
            "velocity": 1.,
            "desired_distance": 1.,
            "side": -1}],
        name='wall_follower',
        remappings=[
            ('/wall_follower_ns/pose', '/pose'),
            ('/wall_follower_ns/map', '/map'),
            ('/wall_follower_ns/base_link', '/base_link'),
            ('/wall_follower_ns/tf', '/tf'),
            ('/wall_follower_ns/tf_static', '/tf_static'),
        ]
    )

    test1 = Node(
        package="wall_follower",
        executable="test_wall_follower",
        namespace='test1',
        parameters=[
            {"scan_topic": "/scan",
            "drive_topic": "/drive",
            "pose_topic": "/pose",
            "velocity": 1.,
            "desired_distance": 1.,
            "side": -1,
            "start_x": -4.,
            "start_y": -5.4,
            "start_z": 0,
            "end_x": 5.,
            "end_y": -5.,
            "name": "short_right_close"}],
        name='test_wall_follower',
        remappings=[
            ('/test1/pose', '/pose'),
            ('/test1/map', '/map'),
            ('/test1/base_link', '/base_link'),
            ('/test1/tf', '/tf'),
            ('/test1/tf_static', '/tf_static'),
        ]
    )

    setup_side2 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'side ',
            '1'
        ]],
        shell=True
    )

    setup_side3 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'side ',
            '-1'
        ]],
        shell=True
    )
    setup_v3 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'velocity ',
            '2.'
        ]],
        shell=True
    )

    setup_side4 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'side ',
            '1'
        ]],
        shell=True
    )

    setup_side5 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'side ',
            '-1'
        ]],
        shell=True
    )
    setup_side6 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'side ',
            '1'
        ]],
        shell=True
    )
    setup_v6 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'velocity ',
            '3.'
        ]],
        shell=True
    )
    setup_d6 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'desired_distance ',
            '0.72'
        ]],
        shell=True
    )

    test2 = Node(
        package="wall_follower",
        executable="test_wall_follower",
        namespace='test2',
        parameters=[
            {"scan_topic": "/scan",
            "drive_topic": "/drive",
            "pose_topic": "/pose",
            "velocity": 1.,
            "desired_distance": 1.,
            "side": 1,
            "start_x": 5.,
            "start_y": -4.4,
            "start_z": np.pi - 0.001,
            "end_x": -4.,
            "end_y": -5.,
            "name": "short_left_far"}],
        name='test_wall_follower',
        remappings=[
            ('/test2/pose', '/pose'),
            ('/test2/map', '/map'),
            ('/test2/base_link', '/base_link'),
            ('/test2/tf', '/tf'),
            ('/test2/tf_static', '/tf_static'),
        ]
    )
    test3 = Node(
        package="wall_follower",
        executable="test_wall_follower",
        namespace='test3',
        parameters=[
            {"scan_topic": "/scan",
            "drive_topic": "/drive",
            "pose_topic": "/pose",
            "velocity": 2.,
            "desired_distance": 1.,
            "side": -1,
            "start_x": -4.,
            "start_y": -5.,
            "start_z":  -np.pi/4,
            "end_x": 5.,
            "end_y": -5.,
            "name": "short_right_angled"}],
        name='test_wall_follower',
        remappings=[
            ('/test3/pose', '/pose'),
            ('/test3/map', '/map'),
            ('/test3/base_link', '/base_link'),
            ('/test3/tf', '/tf'),
            ('/test3/tf_static', '/tf_static'),
        ]
    )
    test4 = Node(
        package="wall_follower",
        executable="test_wall_follower",
        namespace='test4',
        parameters=[
            {"scan_topic": "/scan",
            "drive_topic": "/drive",
            "pose_topic": "/pose",
            "velocity": 2.,
            "desired_distance": 1.,
            "side": 1,
            "start_x": 5.,
            "start_y": -4.,
            "start_z":  3*np.pi/4.,
            "end_x": -4.,
            "end_y": -5.,
            "name": "short_left_far_angled"}],
        name='test_wall_follower',
        remappings=[
            ('/test4/pose', '/pose'),
            ('/test4/map', '/map'),
            ('/test4/base_link', '/base_link'),
            ('/test4/tf', '/tf'),
            ('/test4/tf_static', '/tf_static'),
        ]
    )
    test5 = Node(
        package="wall_follower",
        executable="test_wall_follower",
        namespace='test5',
        parameters=[
            {"scan_topic": "/scan",
            "drive_topic": "/drive",
            "pose_topic": "/pose",
            "velocity": 2.,
            "desired_distance": 1.,
            "side": -1,
            "start_x": -4.,
            "start_y": -5.4,
            "start_z":  -np.pi/6.,
            "end_x": -3.5,
            "end_y": 17.6,
            "name": "long_right"}],
        name='test_wall_follower',
        remappings=[
            ('/test5/pose', '/pose'),
            ('/test5/map', '/map'),
            ('/test5/base_link', '/base_link'),
            ('/test5/tf', '/tf'),
            ('/test5/tf_static', '/tf_static'),
        ]
    )
    test6 = Node(
        package="wall_follower",
        executable="test_wall_follower",
        namespace='test6',
        parameters=[
            {"scan_topic": "/scan",
            "drive_topic": "/drive",
            "pose_topic": "/pose",
            "velocity": 3.,
            "desired_distance": 0.72,
            "side": 1,
            "start_x": -7.,
            "start_y": 10.6,
            "start_z":  0.,
            "end_x": -4.,
            "end_y": -5.,
            "name": "long_left"}],
        name='test_wall_follower',
        remappings=[
            ('/test6/pose', '/pose'),
            ('/test6/map', '/map'),
            ('/test6/base_link', '/base_link'),
            ('/test6/tf', '/tf'),
            ('/test6/tf_static', '/tf_static'),
        ]
    )
    

    return LaunchDescription([
        test1,
        
        RegisterEventHandler(
            OnProcessStart(
                target_action=test1,
                on_start=[
                    LogInfo(msg='Test 1 started, starting wall follower'),

                    TimerAction(
                        period=1.0,
                        actions=[wall_follower],
                    )
                    
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=test1,
                on_exit=[
                    LogInfo(msg='Test 1 finished, Starting Test 2'),
                    # stop_car,
                    setup_side2,
                    test2,
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=test2,
                on_exit=[
                    LogInfo(msg='Test 2 finished, Starting Test 3'),
                    # stop_car,
                    setup_side3,
                    test3,
                    setup_v3
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=test3,
                on_exit=[
                    LogInfo(msg='Test 3 finished, Starting Test 4'),
                    # stop_car,
                    setup_side4,
                    test4
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=test4,
                on_exit=[
                    LogInfo(msg='Test 4 finished, Starting Test 5'),
                    # stop_car,
                    setup_side5,
                    test5
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=test5,
                on_exit=[
                    LogInfo(msg='Test 5 finished, Starting Test 6'),
                    # stop_car,
                    setup_side6,
                    test6,
                    setup_v6,
                    setup_d6
                ]
            )
        ),
    ])
