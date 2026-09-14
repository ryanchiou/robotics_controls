#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from ur_msgs.srv import SetIO
from ur_msgs.msg import IOStates
import time
import numpy as np
from math import pi
import sys
class JointAngles:
    def __init__(self):
        self.name = ["", "", "", "", "", ""]  #could have also done [""] * 6
        self.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# UR3e home position
home = np.radians([138.29, -82.23, 84.87, -90.36, -90.61, 24.48])

# Hanoi tower location 
Q11 = [131.74*pi/180.0, -53.21*pi/180.0, 108.79*pi/180.0, -143.69*pi/180.0, -90.85*pi/180.0, 18.05*pi/180.0]
Q12 = [131.83*pi/180.0, -59.47*pi/180.0, 107.07*pi/180.0, -135.7*pi/180.0, -90.85*pi/180.0, 18.12*pi/180.0]
Q13 = [132.03*pi/180.0, -64.76*pi/180.0, 103.44*pi/180.0, -126.79*pi/180.0, -90.83*pi/180.0, 18.41*pi/180.0]

Q21 = [138.37*pi/180.0, -56.62*pi/180.0, 115.07*pi/180.0, -146.47*pi/180.0, -90.65*pi/180.0, 24.67*pi/180.0]
Q22 = [138.23*pi/180.0, -63.11*pi/180.0, 113.33*pi/180.0, -138.55*pi/180.0, -91.01*pi/180.0, 24.54*pi/180.0]
Q23 = [137.79*pi/180.0, -68.01*pi/180.0, 109.41*pi/180.0, -128.74*pi/180.0, -89.53*pi/180.0, 16.96*pi/180.0]

Q31 = [145.1*pi/180.0, -58.48*pi/180.0, 120.22*pi/180.0, -149.97*pi/180.0, -90.82*pi/180.0, 31.42*pi/180.0]
Q32 = [144.79*pi/180.0, -66.19*pi/180.0, 118.65*pi/180.0, -140.41*pi/180.0, -90.43*pi/180.0, 31.08*pi/180.0]
Q33 = [144.28*pi/180.0, -72.18*pi/180.0, 116.07*pi/180.0, -131.85*pi/180.0, -90.44*pi/180.0, 30.56*pi/180.0]


############## Your Code Start Here ##############
# Q[tower][height], where tower 0/1/2 are the three table locations and
# height 0/1/2 is the position in the stack.  Height 2 is the block sitting on
# the table, height 0 is the top of a full three block stack.
Q = [ [Q11, Q12, Q13], \
      [Q21, Q22, Q23], \
      [Q31, Q32, Q33] ]

# Joint angle offset (shoulder lift) used to hover above a waypoint before
# dropping down onto a block or lifting away from one.
APPROACH_OFFSET = np.radians(12.0)

# Analog Input 0 reads above this value when the suction cup has a block.
SUCTION_THRESHOLD = 1.5
############### Your Code End Here ###############
class UR3e(Node):
    def __init__(self):
        super().__init__('ur3e')

        # Publishers
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)

        # Subscribers
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        ############## Your Code Start Here ##############
        # Gripper input: /io_and_status_controller/io_states carries ur_msgs/msg/IOStates,
        # whose analog_in_states[] array holds the suction feedback on pin 0.
        self.io_state_sub = self.create_subscription(IOStates, '/io_and_status_controller/io_states', self.io_state_callback, 10)
        ############### Your Code End Here ###############

        # Service clients
        self.io_client = self.create_client(SetIO, '/io_and_status_controller/set_io')
        while not self.io_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('IO service not available, waiting...')

        # State variables
        self.current_joint_state = None
        self.analog_in_0_value = 0
        self.current_JointAngles = JointAngles()
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ] # shoulder_pan_joint is the base rotation joint

    def joint_state_callback(self, msg):
        self.current_joint_state = msg  # Currently only used to check if messages have arrived
        index_inOrder = 0
        for name in self.joint_names:
            index_outofOrder = msg.name.index(name)
            self.current_JointAngles.name[index_inOrder] = name
            self.current_JointAngles.position[index_inOrder] = msg.position[index_outofOrder]
            index_inOrder = index_inOrder + 1 


    def io_state_callback(self, msg):
    ############## Your Code Start Here ##############
        """
        Called whenever /io_and_status_controller/io_states publishes.
        msg.analog_in_states is an array of entries that each carry a pin
        number and a state, and the array order is not guaranteed to match
        the pin numbering, so search for the entry whose pin is 0 rather
        than indexing into it directly.
        """
        for analog_in in msg.analog_in_states:
            if analog_in.pin == 0:
                self.analog_in_0_value = analog_in.state
                break

    ############### Your Code End Here ###############

    def gripper_has_block(self):
        """Spin briefly so a fresh IOStates message arrives, then report whether
        Analog Input 0 indicates a block is held by the suction cup."""
        start_time = time.time()
        while time.time() - start_time < 1.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'Analog Input 0 = {self.analog_in_0_value}')
        return self.analog_in_0_value > SUCTION_THRESHOLD

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

        V_MAX = 1#2.09    # rad/s
        A_MAX = 0.8#2.79   # rad/s^2
        MIN_DURATION = 1
        MAX_DURATION = 8.0

        deltas = []
        for i in range(6):
            deltas.append(abs(self.current_JointAngles.position[i] - target[i]))


        max_delta = max(deltas)
        t_acc = V_MAX / A_MAX
        d_acc = 0.5 * A_MAX * (t_acc ** 2)
        if max_delta > 2 * d_acc:
            # trapezoidal velocity profile
            t_total = 2 * t_acc + (max_delta - 2 * d_acc) / V_MAX
        else:
            # triangular velocity profile
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
        p0.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # starting at rest
        p0.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # starting at rest
        p0.time_from_start.sec = 0
        p0.time_from_start.nanosec = 0
        trajectory_msg.points.append(p0)

        # Goal point
        p1 = JointTrajectoryPoint()
        p1.positions = target
        p1.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # end at rest, 2 point trajectory
        p1.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #end at rest.
        p1.time_from_start.sec = int(duration)
        p1.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        trajectory_msg.points.append(p1)

        self.trajectory_pub.publish(trajectory_msg)

        self.get_logger().info(f'Moving to position: {np.degrees(target)}')

        # Wait for movement completion
        start_time = time.time()
        while time.time() - start_time < duration + 2:
            rclpy.spin_once(self, timeout_sec=0.1)

            deltas = []
            for i in range(6):
                deltas.append(abs(self.current_JointAngles.position[i] - target[i]))
            if all(delta < 0.001 for delta in deltas):
                time.sleep(0.25)
                return True
        return False



    def approach_of(self, target):
        """Waypoint directly above target, made by lifting the shoulder."""
        above = list(target)
        above[1] = above[1] - APPROACH_OFFSET
        return above

    def move_block(self, start_tower, start_height, end_tower, end_height):
        global Q
    ############## Your Code Start Here ##############
    # Pick the block at Q[start_tower][start_height] and place it at
    # Q[end_tower][end_height].  Returns 0 on success, 1 on error.

        error = 0

        pick = Q[start_tower][start_height]
        place = Q[end_tower][end_height]

        self.get_logger().info(
            f'Moving block: tower {start_tower + 1} height {start_height + 1}'
            f' -> tower {end_tower + 1} height {end_height + 1}')

        # Hover above the block, then descend onto it
        if not self.move_arm(self.approach_of(pick)):
            self.get_logger().error("Failed to move above pick location")
            return 1
        if not self.move_arm(pick):
            self.get_logger().error("Failed to move to pick location")
            return 1

        # Grab the block and give the suction cup time to seal
        self.set_io(0, 1.0)
        time.sleep(1.0)

        # Suction feedback: if nothing was picked up, there is no block here
        if not self.gripper_has_block():
            self.get_logger().error(
                f'No block detected at tower {start_tower + 1},'
                f' height {start_height + 1}!')
            self.set_io(0, 0.0)
            return 1

        # Lift straight up, cross over, and descend onto the destination
        if not self.move_arm(self.approach_of(pick)):
            self.get_logger().error("Failed to lift block off pick location")
            self.set_io(0, 0.0)
            return 1
        if not self.move_arm(self.approach_of(place)):
            self.get_logger().error("Failed to move above place location")
            self.set_io(0, 0.0)
            return 1
        if not self.move_arm(place):
            self.get_logger().error("Failed to move to place location")
            self.set_io(0, 0.0)
            return 1

        # Release the block and back away before the next move
        self.set_io(0, 0.0)
        time.sleep(0.5)
        if not self.move_arm(self.approach_of(place)):
            self.get_logger().error("Failed to retreat from place location")
            return 1

        return error

    ############### Your Code End Here ###############


def get_tower(prompt):
    """Prompt until the user enters a valid tower, returned as index 0, 1 or 2."""
    while True:
        input_string = input(prompt)
        print("You entered " + input_string + "\n")
        try:
            value = int(input_string)
        except ValueError:
            print("Please just enter 1 2 3, or 0 to quit \n\n")
            continue

        if value == 0:
            print("Quitting... ")
            sys.exit()
        elif value in (1, 2, 3):
            return value - 1
        else:
            print("Please just enter 1 2 3, or 0 to quit \n\n")


def hanoi_moves(n, source, destination, spare):
    """Return the list of (from_tower, to_tower) moves that transfers a stack of
    n blocks from source to destination without ever placing a larger block on
    a smaller one."""
    if n == 0:
        return []
    # Move the top n-1 blocks out of the way, move the bottom block across,
    # then stack the n-1 blocks back on top of it.
    moves = hanoi_moves(n - 1, source, spare, destination)
    moves.append((source, destination))
    moves.extend(hanoi_moves(n - 1, spare, destination, source))
    return moves


def main(args=None):
    input("Check if the UR3e is in 'Remote' Mode?\n\
    Check if the UR3e is initialized and in 'Normal' state.\n\
    Have you run the ROS2 launch statement?\n\
    If there was an UR3e emergency stop or error, Ctrl-C the ros2 launch and rerun.\n\
    \n\
    Press <Enter> to Continue.")
    rclpy.init(args=args)
    node = UR3e()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    ############## Your Code Start Here ##############
    # Wait for initial state updates
    while node.current_joint_state is None:
        executor.spin_once(timeout_sec=0.05)
        node.get_logger().info("Waiting for initial state updates...")
        time.sleep(0.5)

    try:
        # Get the start and destination towers from the user
        start_tower = get_tower("Enter the START tower <1 2 or 3, or 0 to quit> ")
        end_tower = get_tower("Enter the DESTINATION tower <1 2 or 3, or 0 to quit> ")

        if start_tower == end_tower:
            print("Start and destination towers must be different. Quitting...")
            sys.exit()

        # The third tower is the spare used for intermediate moves
        spare_tower = 3 - start_tower - end_tower

        print(f'Moving the tower from location {start_tower + 1}'
              f' to location {end_tower + 1}\n')

        # stacks[t] lists the blocks on tower t, bottom first.  Blocks are
        # numbered 1 (smallest, on top) through 3 (largest, on the bottom),
        # so the start tower begins as [3, 2, 1].
        stacks = [[], [], []]
        stacks[start_tower] = [3, 2, 1]

        node.move_arm(home)

        for src, dst in hanoi_moves(3, start_tower, end_tower, spare_tower):
            # The block being moved is on top of the source stack; its height
            # index is set by how many blocks are already on that tower.
            # A tower holding n blocks has its top block at height index 3 - n.
            src_height = 3 - len(stacks[src])
            dst_height = 3 - (len(stacks[dst]) + 1)

            if node.move_block(src, src_height, dst, dst_height) != 0:
                node.get_logger().error("Tower of Hanoi failed, stopping.")
                node.set_io(0, 0.0)
                node.move_arm(home)
                sys.exit(1)

            stacks[dst].append(stacks[src].pop())

        node.move_arm(home)
        print("Tower of Hanoi complete.\n")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
