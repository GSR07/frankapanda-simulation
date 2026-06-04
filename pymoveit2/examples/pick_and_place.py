#!/usr/bin/env python3
"""
Pick and place node combining Cartesian and joint-space moves.
Locks the detected color coordinates before starting the motion.

ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=R
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=G
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=B

"""

from threading import Thread
from time import monotonic, sleep

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Empty
from std_msgs.msg import String

from pymoveit2 import MoveIt2, GripperInterface
from pymoveit2.robots import panda

import math


class PickAndPlace(Node):
    def __init__(self):
        super().__init__("pick_and_place")

        # Parameters
        self.declare_parameter("target_color", "R")
        self.declare_parameter("pregrasp_z", 0.30)
        # The hand stays above the 0.10 m box while the fingers straddle it.
        self.declare_parameter("grasp_z", 0.170)
        self.declare_parameter("grasp_width", 0.030)
        self.declare_parameter("grasp_settle_time", 1.0)
        self.declare_parameter("attachment_timeout", 2.0)
        # Trash bin at world (0.6, -0.6), expressed in panda_link0.
        self.declare_parameter("drop_x", -0.6)
        self.declare_parameter("drop_y", 0.0)
        self.declare_parameter("drop_above_z", 0.35)
        self.declare_parameter("drop_z", 0.275)
        self.target_color = self.get_parameter("target_color").value.upper()
        self.pregrasp_z = float(self.get_parameter("pregrasp_z").value)
        self.grasp_z = float(self.get_parameter("grasp_z").value)
        self.grasp_width = float(self.get_parameter("grasp_width").value)
        self.grasp_settle_time = float(
            self.get_parameter("grasp_settle_time").value
        )
        self.attachment_timeout = float(
            self.get_parameter("attachment_timeout").value
        )
        self.drop_x = float(self.get_parameter("drop_x").value)
        self.drop_y = float(self.get_parameter("drop_y").value)
        self.drop_above_z = float(self.get_parameter("drop_above_z").value)
        self.drop_z = float(self.get_parameter("drop_z").value)

        # Flags
        self.already_moved = False
        self.ready = False
        self.target_coords = None  # Stores the locked coordinates

        self.callback_group = ReentrantCallbackGroup()

        # Arm MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

        # Set lower velocity & acceleration for smoother motion
        self.moveit2.max_velocity = 0.05
        self.moveit2.max_acceleration = 0.05

        # Gripper interface
        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=panda.gripper_joint_names(),
            open_gripper_joint_positions=panda.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=(
                panda.CLOSED_GRIPPER_JOINT_POSITIONS
            ),
            gripper_group_name=panda.MOVE_GROUP_GRIPPER,
            callback_group=self.callback_group,
            gripper_command_action_name=(
                "gripper_action_controller/gripper_cmd"
            ),
        )

        # Subscriber
        self.sub = self.create_subscription(
            String,
            "/color_coordinates",
            self.coords_callback,
            10,
            callback_group=self.callback_group,
        )
        color_names = {"R": "red", "G": "green", "B": "blue"}
        self.attachment_states = {color: None for color in color_names}
        self.attach_publishers = {
            color: self.create_publisher(
                Empty, f"/panda_pick/{name}/attach", 10
            )
            for color, name in color_names.items()
        }
        self.state_subscribers = [
            self.create_subscription(
                String,
                f"/panda_pick/{name}/state",
                lambda msg, color=color: self.attachment_state_callback(
                    color, msg
                ),
                10,
                callback_group=self.callback_group,
            )
            for color, name in color_names.items()
        ]
        self.detach_publishers = {
            color: self.create_publisher(
                Empty, f"/panda_pick/{name}/detach", 10
            )
            for color, name in color_names.items()
        }
        self.get_logger().info(
            f"Waiting for {self.target_color} from /color_coordinates..."
        )

        # Predefined joint positions (in radians)
        self.start_joints = [
            0.0, 0.0, 0.0, -0.1, 0.0, 0.0, math.radians(-125.0),
        ]
        self.home_joints = [
            0.0, 0.0, 0.0, math.radians(-90.0), 0.0,
            math.radians(92.0), math.radians(50.0),
        ]
        self.drop_joints = [
            math.radians(-155.0), math.radians(30.0), math.radians(-20.0),
            math.radians(-124.0), math.radians(44.0), math.radians(163.0),
            math.radians(7.0),
        ]

    def start(self):
        # Detachable joints start attached in Gazebo. Release all boxes before
        # the arm starts moving, then attach only the selected box at grasp.
        sleep(0.25)
        for publisher in self.detach_publishers.values():
            publisher.publish(Empty())
        sleep(0.25)
        self.move_to_start()

    def attachment_state_callback(self, color, msg):
        self.attachment_states[color] = msg.data.strip().lower()

    def wait_for_attachment_state(self, color, expected_state):
        deadline = monotonic() + self.attachment_timeout
        while monotonic() < deadline:
            if self.attachment_states[color] == expected_state:
                return True
            sleep(0.05)
        return False

    def wait_for_arm(self, motion_name):
        if not self.moveit2.wait_until_executed():
            raise RuntimeError(f"Arm motion failed: {motion_name}")

    def wait_for_gripper(self, motion_name):
        if not self.gripper.wait_until_executed():
            raise RuntimeError(f"Gripper motion failed: {motion_name}")

    def move_to_start(self):
        self.moveit2.move_to_configuration(self.start_joints)
        self.wait_for_arm("move to start")
        self.ready = True
        self.get_logger().info("Ready to pick a detected target")

    def coords_callback(self, msg):
        if not self.ready or self.already_moved:
            return  # Ignore messages once motion starts

        try:
            color_id, x, y, z = msg.data.split(",")
            color_id = color_id.strip().upper()

            if color_id == self.target_color:
                # Lock coordinates immediately
                self.target_coords = [float(x), float(y), float(z)]
                self.get_logger().info(
                    f"Target {self.target_color} locked at: "
                    f"[{self.target_coords[0]:.3f}, "
                    f"{self.target_coords[1]:.3f}, "
                    f"{self.target_coords[2]:.3f}]"
                )
                self.already_moved = True

                # Detector x/y values are calibrated in panda_link0. Its z is
                # synthetic, so use measured table/object heights.
                pregrasp_position = [
                    self.target_coords[0],
                    self.target_coords[1],
                    self.pregrasp_z,
                ]
                grasp_position = [
                    self.target_coords[0],
                    self.target_coords[1],
                    self.grasp_z,
                ]
                quat_xyzw = [0.0, 1.0, 0.0, 0.0]
                self.get_logger().info(
                    f"Pre-grasp: {pregrasp_position}; grasp: {grasp_position}"
                )

                # --- Pick-and-place sequence ---

                # 1. Move to home joint configuration
                self.moveit2.move_to_configuration(self.home_joints)
                self.wait_for_arm("move to home")

                # 2. Open before entering the object's workspace
                self.gripper.open()
                self.wait_for_gripper("open before grasp")

                # 3. Move above target
                self.moveit2.move_to_pose(
                    position=pregrasp_position,
                    quat_xyzw=quat_xyzw,
                )
                self.wait_for_arm("move to pre-grasp")

                # 4. Descend vertically around the object
                self.moveit2.move_to_pose(
                    position=grasp_position,
                    quat_xyzw=quat_xyzw,
                    cartesian=True,
                    cartesian_fraction_threshold=0.9,
                )
                self.wait_for_arm("descend to grasp")

                # 5. Hold the box at the contact pose before finger closure.
                # Gazebo cannot create the joint once collisions overlap.
                self.attachment_states[self.target_color] = None
                self.attach_publishers[self.target_color].publish(Empty())
                if not self.wait_for_attachment_state(
                    self.target_color, "attached"
                ):
                    raise RuntimeError(
                        "Gazebo did not confirm the object attachment; "
                        "lift cancelled"
                    )

                # 6. Clamp the 0.06 m box at its side edges. Each Panda finger
                # joint is half of the total opening, so 0.030 m gives a
                # 0.060 m gap without driving the fingers through the box.
                self.gripper.move_to_position(self.grasp_width)
                self.wait_for_gripper("clamp object edges")
                sleep(self.grasp_settle_time)
                self.get_logger().info("Grasp confirmed; lifting target")

                self.moveit2.move_to_pose(
                    position=pregrasp_position,
                    quat_xyzw=quat_xyzw,
                    cartesian=True,
                    cartesian_fraction_threshold=0.9,
                )
                self.wait_for_arm("lift object")
                sleep(0.25)

                # 8. Move to home joint configuration before relocating to the bin.
                self.moveit2.move_to_configuration(self.home_joints)
                self.wait_for_arm("carry object to home")

                # 9. Move to a drop pose above the trash bin instead of a hard-coded
                # joint configuration.
                drop_above_position = [
                    self.drop_x,
                    self.drop_y,
                    self.drop_above_z,
                ]
                drop_position = [self.drop_x, self.drop_y, self.drop_z]
                self.moveit2.move_to_pose(
                    position=drop_above_position,
                    quat_xyzw=quat_xyzw,
                )
                self.wait_for_arm("move object above drop pose")

                self.moveit2.move_to_pose(
                    position=drop_position,
                    quat_xyzw=quat_xyzw,
                    cartesian=True,
                    cartesian_fraction_threshold=0.9,
                )
                self.wait_for_arm("lower object into drop pose")

                # 10. Open the fingers, then release the simulated grasp.
                self.gripper.open()
                self.wait_for_gripper("open at drop pose")
                self.attachment_states[self.target_color] = None
                self.detach_publishers[self.target_color].publish(Empty())
                if not self.wait_for_attachment_state(
                    self.target_color, "detached"
                ):
                    raise RuntimeError(
                        "Gazebo did not confirm the object release"
                    )
                sleep(0.5)

                # 11. Return to start joint configuration
                self.moveit2.move_to_configuration(self.start_joints)
                self.wait_for_arm("return to start")

                self.get_logger().info("Pick-and-place sequence complete.")

        except Exception as e:
            self.get_logger().error(f"Pick-and-place failed: {e}")


def main():
    rclpy.init()
    node = PickAndPlace()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        node.start()
        executor_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
