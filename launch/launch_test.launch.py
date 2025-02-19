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
    ############################################################################
    ### Define test nodes and all their parameters
    ############################################################################
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
    
    
    ############################################################################
    ### Define wall follower node
    ############################################################################
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
    
    
    ############################################################################
    ### Define commands to change parameters of the wall_follower node
    ############################################################################  
    setup_side2_1 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'side ',
            '1'
        ]],
        shell=True
    )
    setup_side2_2 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'side ',
            '1'
        ]],
        shell=True
    )
    setup_side2_3 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'side ',
            '1'
        ]],
        shell=True
    )
    
    setup_side3_1 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'side ',
            '-1'
        ]],
        shell=True
    )
    setup_side3_2 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'side ',
            '-1'
        ]],
        shell=True
    )
    setup_side3_3 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'side ',
            '-1'
        ]],
        shell=True
    )
    
    setup_v3_1 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'velocity ',
            '2.'
        ]],
        shell=True
    )
    setup_v3_2 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'velocity ',
            '2.'
        ]],
        shell=True
    )
    setup_v3_3 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'velocity ',
            '2.'
        ]],
        shell=True
    )
    
    setup_side4_1 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'side ',
            '1'
        ]],
        shell=True
    )
    setup_side4_2 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'side ',
            '1'
        ]],
        shell=True
    )
    setup_side4_3 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'side ',
            '1'
        ]],
        shell=True
    )
    
    setup_side5_1 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'side ',
            '-1'
        ]],
        shell=True
    )
    setup_side5_2 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'side ',
            '-1'
        ]],
        shell=True
    )
    setup_side5_3 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'side ',
            '-1'
        ]],
        shell=True
    )
    
    setup_side6_1 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'side ',
            '1'
        ]],
        shell=True
    )
    setup_side6_2 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'side ',
            '1'
        ]],
        shell=True
    )
    setup_side6_3 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'side ',
            '1'
        ]],
        shell=True
    )
    
    setup_v6_1 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'velocity ',
            '3.'
        ]],
        shell=True
    )
    setup_v6_2 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'velocity ',
            '3.'
        ]],
        shell=True
    )
    setup_v6_3 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'velocity ',
            '3.'
        ]],
        shell=True
    )
    
    setup_d6_1 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'desired_distance ',
            '0.72'
        ]],
        shell=True
    )
    setup_d6_2 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'desired_distance ',
            '0.72'
        ]],
        shell=True
    )
    setup_d6_3 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ', 
            'desired_distance ',
            '0.72'
        ]],
        shell=True
    )
    

    ############################################################################
    ### Define launch description
    ############################################################################
    return LaunchDescription([
        test1,
        
        # Start wall follower at beginning of the test (wall follower node is only spun up once)
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
        
        # Start test 2 after test 1 finishes
        RegisterEventHandler(
            OnProcessExit(
                target_action=test1,
                on_exit=[
                    LogInfo(msg='Test 1 finished, Starting Test 2'),
                    test2,
                    setup_side2_1,
                    TimerAction(  # Wait 1 second, then run setup_side2 again (trying to increase reliability)
                        period=1.0,
                        actions=[setup_side2_2],
                    ),
                    TimerAction(  # Wait 4 seconds, then run setup_side2 again (trying to increase reliability)
                        period=4.0,
                        actions=[setup_side2_3],
                    )
                ]
            )
        ),
        
        # Start test 3 after test 2 finishes
        RegisterEventHandler(
            OnProcessExit(
                target_action=test2,
                on_exit=[
                    LogInfo(msg='Test 2 finished, Starting Test 3'),
                    test3,
                    setup_side3_1,
                    setup_v3_1,
                    TimerAction(  # Wait 1 second, then run setup_side3 again (trying to increase reliability)
                        period=1.0,
                        actions=[setup_side3_2],
                    ),
                    TimerAction(  # Wait 1 second, then run setup_v3 again (trying to increase reliability)
                        period=1.0,
                        actions=[setup_v3_2],
                    ),
                    TimerAction(  # Wait 4 seconds, then run setup_side3 again (trying to increase reliability)
                        period=4.0,
                        actions=[setup_side3_3],
                    ),
                    TimerAction(  # Wait 4 seconds, then run setup_v3 again (trying to increase reliability)
                        period=4.0,
                        actions=[setup_v3_3],
                    )
                ]
            )
        ),
        
        # Start test 4 after test 3 finishes
        RegisterEventHandler(
            OnProcessExit(
                target_action=test3,
                on_exit=[
                    LogInfo(msg='Test 3 finished, Starting Test 4'),
                    test4,
                    setup_side4_1,
                    TimerAction(  # Wait 1 second, then run setup_side4 again (trying to increase reliability)
                        period=1.0,
                        actions=[setup_side4_2],
                    ),
                    TimerAction(  # Wait 4 seconds, then run setup_side4 again (trying to increase reliability)
                        period=4.0,
                        actions=[setup_side4_3],
                    )
                ]
            )
        ),
        
        # Start test 5 after test 4 finishes
        RegisterEventHandler(
            OnProcessExit(
                target_action=test4,
                on_exit=[
                    LogInfo(msg='Test 4 finished, Starting Test 5'),
                    test5,
                    setup_side5_1,
                    TimerAction(  # Wait 1 second, then run setup_side5 again (trying to increase reliability)
                        period=1.0,
                        actions=[setup_side5_2],
                    ),
                    TimerAction(  # Wait 4 seconds, then run setup_side5 again (trying to increase reliability)
                        period=4.0,
                        actions=[setup_side5_3],
                    )
                ]
            )
        ),
        
        # Start test 6 after test 5 finishes
        RegisterEventHandler(
            OnProcessExit(
                target_action=test5,
                on_exit=[
                    LogInfo(msg='Test 5 finished, Starting Test 6'),
                    test6,
                    setup_side6_1,
                    setup_v6_1,
                    setup_d6_1,
                    TimerAction(  # Wait 1 second, then run setup_side6 again (trying to increase reliability)
                        period=1.0,
                        actions=[setup_side6_2],
                    ),
                    TimerAction(  # Wait 1 second, then run setup_v6 again (trying to increase reliability)
                        period=1.0,
                        actions=[setup_v6_2],
                    ),
                    TimerAction(  # Wait 1 second, then run setup_d6 again (trying to increase reliability)
                        period=1.0,
                        actions=[setup_d6_2],
                    ),
                    TimerAction(  # Wait 4 seconds, then run setup_side6 again (trying to increase reliability)
                        period=4.0,
                        actions=[setup_side6_3],
                    ),
                    TimerAction(  # Wait 4 seconds, then run setup_v6 again (trying to increase reliability)
                        period=4.0,
                        actions=[setup_v6_3],
                    ),
                    TimerAction(  # Wait 1 second, then run setup_d6 again (trying to increase reliability)
                        period=4.0,
                        actions=[setup_d6_3],
                    )
                ]
            )
        ),
    ])
