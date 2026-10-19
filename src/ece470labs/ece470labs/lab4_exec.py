#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from ur_msgs.srv import SetIO
from ur_msgs.msg import IOStates
import time
import copy
import numpy as np
from math import pi
import sys
from .lab_func import *


class JointAngles:
    def __init__(self):
        self.name = ["", "", "", "", "", ""]
        self.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


# UR3 home position
home = np.radians([180, 0, 0, -90, 0, 0])


class UR3e(Node):
    def __init__(self):
        super().__init__('ur3e')

        # Publishers
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        ############## Your Code Start Here ##############
        # TODO: define a ROS subscriber for gripper input message
        # and corresponding callback function.
        # ROS2 gripper input topic: /io_and_status_controller/io_states

        ############### Your Code End Here ###############

        # Service clients
        self.io_client = self.create_client(
            SetIO, '/io_and_status_controller/set_io'
        )

        while not self.io_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('IO service not available, waiting...')

        # State variables
        self.current_joint_state = None
        self.analog_in_0_value = 0
        self.current_JointAngles = JointAngles()

        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

        index_inOrder = 0
        for name in self.joint_names:
            index_outofOrder = msg.name.index(name)
            self.current_JointAngles.name[index_inOrder] = name
            self.current_JointAngles.position[index_inOrder] = (
                msg.position[index_outofOrder]
            )
            index_inOrder = index_inOrder + 1

    def io_state_callback(self, msg):
        ############## Your Code Start Here ##############
        """
        TODO: define a ROS topic callback function that
        receives and stores the state of the suction cup.
        Whenever /io_and_status_controller/io_states
        publishes this info, this callback function is called.
        """

        pass

        ############### Your Code End Here ###############

    def set_io(self, pin, state):
        req = SetIO.Request()
        req.fun = 1
        req.pin = pin
        req.state = state

        future = self.io_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        return future.result()

    def move_arm(self, target):
        if self.current_joint_state is None:
            self.get_logger().error("No joint state received!")
            return False

        V_MAX = 1       # rad/s
        A_MAX = 0.8     # rad/s^2
        MIN_DURATION = 1
        MAX_DURATION = 8.0

        deltas = []
        for i in range(6):
            deltas.append(
                abs(self.current_JointAngles.position[i] - target[i])
            )

        max_delta = max(deltas)
        t_acc = V_MAX / A_MAX
        d_acc = 0.5 * A_MAX * (t_acc ** 2)

        if max_delta > 2 * d_acc:
            # Trapezoidal velocity profile
            t_total = 2 * t_acc + (max_delta - 2 * d_acc) / V_MAX
        else:
            # Triangular velocity profile
            t_total = 2 * (max_delta / A_MAX) ** 0.5

        duration = max(MIN_DURATION, min(t_total, MAX_DURATION))

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        # Start immediately when the controller receives it
        trajectory_msg.header.stamp.sec = 0
        trajectory_msg.header.stamp.nanosec = 0

        # Anchor point: current measured joint state at t = 0
        p0 = JointTrajectoryPoint()
        p0.positions = self.current_JointAngles.position
        p0.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        p0.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        p0.time_from_start.sec = 0
        p0.time_from_start.nanosec = 0
        trajectory_msg.points.append(p0)

        # Goal point
        p1 = JointTrajectoryPoint()
        p1.positions = target
        p1.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        p1.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        p1.time_from_start.sec = int(duration)
        p1.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        trajectory_msg.points.append(p1)

        self.trajectory_pub.publish(trajectory_msg)

        self.get_logger().info(
            f'Moving to position: {np.degrees(target)}'
        )

        # Wait for movement completion
        start_time = time.time()

        while time.time() - start_time < duration + 2:
            rclpy.spin_once(self, timeout_sec=0.1)

            deltas = []
            for i in range(6):
                deltas.append(
                    abs(self.current_JointAngles.position[i] - target[i])
                )

            if all(delta < 0.001 for delta in deltas):
                time.sleep(0.25)
                return True

        return False

    def move_block(self, start_tower, start_height, end_tower, end_height):
        global Q

        ############## Your Code Start Here ##############
        # TODO: add code to move block from start tower and height
        # to end tower and height.
        # Hint: Use the Q array to map towers by location and height.

        error = 0

        return error

        ############### Your Code End Here ###############


# UR3e link lengths in mm
L1 = 152
L2 = 120
L3 = 244
L4 = 93
L5 = 213
L6 = 104
L7 = 85
L8 = 92
L9 = 53.5
L10 = 59

# M and S matrices
M = np.array([
    [0, -1, 0, 392],
    [0, 0, -1, 432],
    [1, 0, 0, 215.5],
    [0, 0, 0, 1]
])

S1 = np.array([
    [0, -1, 0, 150],
    [1, 0, 0, 150],
    [0, 0, 0, 0],
    [0, 0, 0, 0]
])

S2 = np.array([
    [0, 0, 1, -162],
    [0, 0, 0, 0],
    [-1, 0, 0, -150],
    [0, 0, 0, 0]
])

S3 = np.array([
    [0, 0, 1, -162],
    [0, 0, 0, 0],
    [-1, 0, 0, 94],
    [0, 0, 0, 0]
])

S4 = np.array([
    [0, 0, 1, -162],
    [0, 0, 0, 0],
    [-1, 0, 0, 307],
    [0, 0, 0, 0]
])

S5 = np.array([
    [0, 0, 0, 0],
    [0, 0, -1, 162],
    [0, 1, 0, -281],
    [0, 0, 0, 0]
])

S6 = np.array([
    [0, 0, 1, -162],
    [0, 0, 0, 0],
    [-1, 0, 0, 392],
    [0, 0, 0, 0]
])


def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):
    """Calculate forward kinematics and controller joint angles."""

    return_value = [None, None, None, None, None, None]

    print("Foward kinematics calculated:\n")

    T = (
        expm(S1 * theta1)
        @ expm(S2 * theta2)
        @ expm(S3 * theta3)
        @ expm(S4 * theta4)
        @ expm(S5 * theta5)
        @ expm(S6 * theta6)
        @ M
    )

    print(str(T) + "\n")

    return_value[0] = theta1 + pi
    return_value[1] = theta2
    return_value[2] = theta3
    return_value[3] = theta4 - (0.5 * pi)
    return_value[4] = theta5
    return_value[5] = theta6

    return return_value


def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
    # Inputs: world position in meters and yaw in degrees.
    # Internal geometry: millimeters and radians.

    yaw = np.radians(yaw_WgripDegree)

    # Convert world coordinates to base coordinates.
    xgrip = xWgrip * 1000 + 150
    ygrip = yWgrip * 1000 - 150
    zgrip = zWgrip * 1000 - 10

    # Wrist-center position.
    xcen = xgrip - L9 * np.cos(yaw)
    ycen = ygrip - L9 * np.sin(yaw)
    zcen = zgrip

    # Base and final wrist angles.
    offset = L2 - L4 + L6

    theta1 = (
        np.arctan2(ycen, xcen)
        - np.arcsin(offset / np.sqrt(xcen**2 + ycen**2))
    )

    theta6 = theta1 + pi / 2 - yaw

    # Endpoint of the shoulder/elbow linkage, in base coordinates.
    x3end = (
        xcen
        - L7 * np.cos(theta1)
        + offset * np.sin(theta1)
    )

    y3end = (
        ycen
        - L7 * np.sin(theta1)
        - offset * np.cos(theta1)
    )

    z3end = zcen + L8 + L10

    # Corrected shoulder-to-endpoint geometry.
    # rho is the signed horizontal distance in the arm's plane.
    rho = x3end * np.cos(theta1) + y3end * np.sin(theta1)

    # Vertical distance above the shoulder.
    h = z3end - L1

    # Both points must be expressed in the base frame.
    r = np.linalg.norm(
        np.array([x3end, y3end, z3end])
        - np.array([0, 0, L1])
    )

    if r < abs(L3 - L5) - 1e-9 or r > L3 + L5 + 1e-9:
        raise ValueError(
            "Target is outside the shoulder/elbow reach."
        )

    # Shoulder triangle angle.
    alpha = np.arccos(np.clip(
        (r**2 + L3**2 - L5**2) / (2 * L3 * r),
        -1.0,
        1.0
    ))

    # Interior elbow angle.
    beta = np.arccos(np.clip(
        (-r**2 + L3**2 + L5**2) / (2 * L3 * L5),
        -1.0,
        1.0
    ))

    # Elevation of the shoulder-to-endpoint line.
    gamma = np.arctan2(h, rho)

    theta2 = -(alpha + gamma)
    theta3 = pi - beta
    theta4 = -(theta2 + theta3)
    theta5 = -pi / 2

    print(
        np.degrees(theta1),
        np.degrees(theta2),
        np.degrees(theta3),
        np.degrees(theta4),
        np.degrees(theta5),
        np.degrees(theta6)
    )

    return lab_fk(
        theta1, theta2, theta3, theta4, theta5, theta6
    )


def main(args=None):
    input(
        "Check if the UR3e is in 'Remote' Mode?\n"
        "Check if the UR3e is initialized and in 'Normal' state.\n"
        "Have you run the ROS2 launch statement?\n"
        "If there was an UR3e emergency stop or error, "
        "Ctrl-C the ros2 launch and rerun.\n"
        "\n"
        "Press <Enter> to Continue."
    )

    rclpy.init(args=args)
    node = UR3e()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    new_dest = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Wait for initial state updates.
    while node.current_joint_state is None:
        executor.spin_once(timeout_sec=0.05)
        node.get_logger().info("Waiting for initial state updates...")
        time.sleep(0.5)

    try:
        if len(sys.argv) != 5:
            print("\n")
            print("Invalid number of input!\n")
            print(
                "ros2 run ECE470lab lab4_exec "
                "xWgrip yWgrip zWgrip yaw_WgripDegree \n"
            )
        else:
            print(
                "\nxWgrip: " + sys.argv[1]
                + ", yWgrip: " + sys.argv[2]
                + ", zWgrip: " + sys.argv[3]
                + ", yaw_WgripDegree: " + sys.argv[4]
                + "\n"
            )

        new_dest = lab_invk(
            float(sys.argv[1]),
            float(sys.argv[2]),
            float(sys.argv[3]),
            float(sys.argv[4])
        )

        if not node.move_arm(new_dest):
            node.get_logger().error("Failed to move to goal")

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
