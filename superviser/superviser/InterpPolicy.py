#
#  Copyright (C) 2026 Intrinsic Innovation LLC
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#


from turtle import position
from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)

from aic_control_interfaces.msg import (
    MotionUpdate,
    TrajectoryGenerationMode,
)

from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Point, Pose, Quaternion, Transform, Vector3, Wrench

from enum import Enum

class InterpPolicy(Policy):
    def __init__(self, parent_node: Node):
        super().__init__(parent_node)
        self.get_logger().info("InterpPolicy.__init__()")

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, parent_node)

        # TF parameters
        parent_node.declare_parameters(
            namespace="",
            parameters=[
                ("tf_gripper", "gripper/tcp"),
                ("tf_base", "base_link"),
                ("tf_task_board", "task_board"),
            ]
        )

        self._tf_gripper = parent_node.get_parameter("tf_gripper").get_parameter_value().string_value
        self._tf_board = parent_node.get_parameter("tf_task_board").get_parameter_value().string_value
        self._tf_base = parent_node.get_parameter("tf_base").get_parameter_value().string_value
        
        self._clearance_height = 0.1

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ) -> bool:
        self._parent_node.get_logger().info(f"Recieved task: {task}")

        target_port_name = task.port_name
        target_module_name = task.target_module_name

        # Gripper from task board
        tf_grtb = self._tf_buffer.lookup_transform(
            self._tf_gripper,
            self._tf_board,
            rclpy.time.Time(),
        )
        
        table_offset = tf_grtb.transform.translation.z
        if table_offset < self._clearance_height:
            pass
        else:
            # Move to centre with clearance height
            self._parent_node.get_logger().info("MOVE STATE")

            # tf2_echo and lookup_transform directions are flipped
            target_port_frame = f"task_board/{target_module_name}/{target_port_name}_link_entrance"
            tf_tbbl = self._tf_buffer.lookup_transform(
                self._tf_base,
                target_port_frame,
                rclpy.time.Time(),
            )

            tf_grbl = self._tf_buffer.lookup_transform(
                self._tf_base,
                self._tf_gripper,
                rclpy.time.Time(),
            )

            x = tf_tbbl.transform.translation.x
            y = tf_tbbl.transform.translation.y
            z = tf_tbbl.transform.translation.z + self._clearance_height

            qx = tf_grbl.transform.rotation.x
            qy = tf_grbl.transform.rotation.y
            qz = tf_grbl.transform.rotation.z
            qw = tf_grbl.transform.rotation.w
            #
            # Convert to a pose
            pose = Pose(
                position=Point(x=x, y=y, z=z),
                orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
            )

            self._parent_node.get_logger().info(f"Moving to: {pose}")

            self.set_pose_target(move_robot, pose, frame_id="base_link")

            # # move along one axis at a time
            # xr = tf_grbl.transform.translation.x 
            # yr = tf_grbl.transform.translation.y 
            # zr = tf_grbl.transform.translation.z 
            #
            # pose1 = Pose(
            #     position=Point(x=x, y=yr, z=zr),
            #     orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
            # )
            #
            # pose2 = Pose(
            #     position=Point(x=x, y=y, z=zr),
            #     orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
            # )
            #
            # pose3 = Pose(
            #     position=Point(x=x, y=y, z=z),
            #     orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
            # )
            #
            # self.set_pose_target(move_robot, pose1)
            # self._parent_node.get_clock().sleep_for(Duration(seconds=2.5))
            #
            # self.set_pose_target(move_robot, pose2)
            # self._parent_node.get_clock().sleep_for(Duration(seconds=2.5))
            #
            # self.set_pose_target(move_robot, pose3)
            # self._parent_node.get_clock().sleep_for(Duration(seconds=2.5))
        return True
    
    # States
    def _raise(self, height: float) -> None:
        pass
