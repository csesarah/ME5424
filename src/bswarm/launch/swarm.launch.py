#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, TimerAction
from launch_ros.actions import PushRosNamespace, Node
from ament_index_python.packages import get_package_share_directory

COUNT = 25

def generate_launch_description():
    count = COUNT
    ld = LaunchDescription()

    # micro-xrce-dds service
    ld.add_action(
        ExecuteProcess(
            cmd=["bash", "-lc", "MicroXRCEAgent udp4 -p 8888"], 
            output="screen"
        )
    )

    # launch drones
    for idx in range(1, count+1):
        x = (idx - 1) % 5
        y = (idx - 1) // 5
        pose = f"{x * 2},{y * 2},0,0,0,0"

        env = os.environ.copy()        
        env["PX4_GZ_MODEL_POSE"] = pose
        env["PX4_SIM_QMODEL"] = "gz_x500"
        if idx > 1:
            env["PX4_GZ_STANDALONE"] = "1"
        env["PX4_SYS_AUTOSTART"] = "4001"

        ld.add_action(
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-lc",
                    f"./src/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i {idx}",
                ],
                output="screen",
                env=env
            )
        )


    return ld
