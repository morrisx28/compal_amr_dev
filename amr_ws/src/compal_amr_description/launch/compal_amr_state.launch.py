# import os
# from launch import LaunchDescription
# from launch_ros.actions import Node


# def generate_launch_description():
#     # --- 路徑：你的 URDF 位置 ---
#     compal_urdf_path = "/home/csl/nav2_li_local_ws/src/compal_amr_description/urdf_file/compalamr/urdf/compal_amr.urdf"

#     # --- 檢查檔案是否存在（可選） ---
#     if not os.path.exists(compal_urdf_path):
#         raise FileNotFoundError(f"URDF not found: {compal_urdf_path}")

#     # --- 啟動 robot_state_publisher ---
#     robot_state_publisher_node = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         name="robot_state_publisher",
#         output="screen",
#         parameters=[{
#             "robot_description": open(compal_urdf_path).read()
#         }]
#     )

#     return LaunchDescription([
#         robot_state_publisher_node
#     ])

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # URDF 檔案路徑
    urdf_path = "/home/csl/nav2_li_local_ws/src/compal_amr_description/urdf_file/compalamr/urdf/compal_amr.urdf"

    # 檢查檔案是否存在
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    # 讀取 URDF 檔案
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    # robot_state_publisher 節點
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{'robot_description': robot_description}],
    )

    return LaunchDescription([robot_state_publisher_node])

