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
        self.name = ["", "", "", "", "", ""]  #could have also done [""] * 6
        self.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# UR3 home position
home = np.radians([180, 0, 0, -90, 0, 0])

class UR3e(Node):
    def __init__(self):
        super().__init__('ur3e')

        # Publishers
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)

        # Subscribers
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        ############## Your Code Start Here ##############
        # TODO: define a ROS subscriber for gripper input message and corresponding callback function
        # ROS2 gripper input topic: /io_and_status_controller/io_states

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
        TODO: define a ROS topic callback funtion that 
        receives and stores the state of  the suction cup
        Whenever /io_and_status_controller/io_states 
        publishes this info, this callback function is
        called.
        """
        for analog_in in msg.analog_in_states:
            if analog_in.pin == 0:
                self.analog_in_0_value = analog_in.state
                break

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



    def move_block(self, start_tower, start_height, end_tower, end_height):
        global Q
    ############## Your Code Start Here ##############
    # TODO: add code to move block from start tower and height to end tower and height
    ### Hint: Use the Q array to map out your towers by location and "height".

        error = 0



        return error

    ############### Your Code End Here ###############






def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	M = [[0,-1,0,392],\
        [0,0,-1,432],\
        [1,0,0,215.5],\
        [0,0,0,1]]

    
	S1 = [[0,-1,0,-150],\
        [1,0,0,150],\
        [0,0,0,10],\
        [0,0,0,0]]

	S2 = [[0,0,1,-150],\
        [0,0,0,270],\
        [-1,0,0,162],\
        [0,0,0,0]]

	S3 = [[0,0,1,94],\
        [0,0,0,270],\
        [-1,0,0,162],\
        [0,0,0,0]]

	S4 = [[0,0,1,307],\
        [0,0,0,177],\
        [-1,0,0,162],\
        [0,0,0,0]] 


	S5 = [[0,0,0,307],\
        [0,0,-1,340],\
        [0,1,0,162],\
        [0,0,0,0]]

	S6 = [[0,0,1,392],\
        [0,0,0,432],\
        [-1,0,0,215.5],\
        [0,0,0,0]]                   
    

	# ==============================================================#
	return M, S1, S2, S3, S4, S5, S6




"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#

	T = expm(S1*theta1)*expm(S2*theta2)*expm(S3*theta3)*expm(S4*theta4)*expm(S5*theta5)*expm(S6*theta6)*M
	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + pi
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*pi)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value





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
    new_dest = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    # Wait for initial state updates
    while node.current_joint_state is None:
        executor.spin_once(timeout_sec=0.05)
        node.get_logger().info("Waiting for initial state updates...")
        time.sleep(0.5)

    try:
        if(len(sys.argv) != 7):
            print("\n")
            print("Command should be entered in degrees with format: \n")
            print("ros2 run ECE470lab lab3_exec theta1 theta2 theta3 theta4 theta5 theta6 \n")
        else:
            print("\ntheta1: " + sys.argv[1] + ", theta2: " + sys.argv[2] + \
                ", theta3: " + sys.argv[3] + ", theta4: " + sys.argv[4] + \
                ", theta5: " + sys.argv[5] + ", theta6: " + sys.argv[6] + "\n")

        new_dest = lab_fk(float(sys.argv[1])*pi/180.0, float(sys.argv[2])*pi/180.0, \
                        float(sys.argv[3])*pi/180.0, float(sys.argv[4])*pi/180.0, \
                        float(sys.argv[5])*pi/180.0, float(sys.argv[6])*pi/180.0,)
        
        if not node.move_arm(new_dest):
            node.get_logger().error("Failed to move to goal")
     
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
