import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'orbbec_camera'
    launch_file_name = 'gemini_330_series.launch.py'
    
    orbbec_launch_file = os.path.join(
        get_package_share_directory(package_name),
        'launch',
        launch_file_name
    )

    # 크롭 노드 스크립트 경로
    crop_script_path = os.path.join(
        os.path.dirname(os.path.realpath(__file__)), 
        'crop_node.py'
    )

    # ================= 설정 값 =================
    # 1280x800 @ 5fps (안정성 확보)
    req_width = '640'
    req_height = '400'
    req_fps = '10'
    # ==========================================

    # ---------------------------------------------------------
    # 1. Front Camera (즉시 실행)
    # ---------------------------------------------------------
    
    cam_front_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(orbbec_launch_file),
        launch_arguments={
            'camera_name': 'cam_front',           # 드라이버 이름 통일
            'serial_number': 'CP82841000G5',  
            'device_num': '1',
            'enable_point_cloud': 'false',
            'enable_colored_point_cloud': 'false',
            'enable_depth': 'false',
            'color_width': req_width,
            'color_height': req_height,
            'color_fps': req_fps,
        }.items()
    )
    
    cam_front_cropper = Node(
        package=None, executable=sys.executable,
        arguments=[crop_script_path],
        namespace='cam_front',                    # [중요] 드라이버와 같은 네임스페이스
        output='screen',
        parameters=[{'target_w': 480, 'target_h': 426}]
    )

    # ---------------------------------------------------------
    # 2. Right Camera (2초 딜레이)
    # ---------------------------------------------------------

    cam_right_driver = TimerAction(
        period=2.0,  # 2초 뒤 실행 (USB 충돌 방지)
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(orbbec_launch_file),
            launch_arguments={
                'camera_name': 'cam_right',         # 드라이버 이름 통일
                'serial_number': 'CP82841000C2',  
                'device_num': '2',
                'enable_point_cloud': 'false',
                'enable_colored_point_cloud': 'false',
                'enable_depth': 'false',
                'color_width': req_width,
                'color_height': req_height,
                'color_fps': req_fps,
            }.items()
        )]
    )

    cam_right_cropper = Node(
        package=None, executable=sys.executable,
        arguments=[crop_script_path],
        namespace='cam_right',                  # [중요] 드라이버와 같은 네임스페이스
        output='screen',
        parameters=[{'target_w': 480, 'target_h': 426}]
    )

    # ---------------------------------------------------------
    # 3. Left Camera (4초 딜레이)
    # ---------------------------------------------------------

    cam_left_driver = TimerAction(
        period=4.0,  # 4초 뒤 실행
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(orbbec_launch_file),
            launch_arguments={
                'camera_name': 'cam_left',          # 드라이버 이름 통일
                'serial_number': 'CP82841000KH',
                'device_num': '3',
                'enable_point_cloud': 'false',
                'enable_colored_point_cloud': 'false',
                'enable_depth': 'false',
                'color_width': req_width,
                'color_height': req_height,
                'color_fps': req_fps,
            }.items()
        )]
    )

    cam_left_cropper = Node(
        package=None, executable=sys.executable,
        arguments=[crop_script_path],
        namespace='cam_left',                   # [중요] 드라이버와 같은 네임스페이스
        output='screen',
        parameters=[{'target_w': 480, 'target_h': 426}]
    )

    return LaunchDescription([
        cam_front_driver, cam_front_cropper,
        cam_right_driver, cam_right_cropper,
        cam_left_driver, cam_left_cropper
    ])
