#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import math
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from threading import Thread
import matplotlib.pyplot as plt
import time


TARGET_X = 10
TARGET_Y = 10
global_time = time.time()

class ProportionalControlNode(Node):

    def __init__(self):
        super().__init__('proportional_control_node')

        self.time_values = []
        self.error_x_values = []
        self.error_y_values = []
        self.lin_x_values = []
        self.lin_y_values = []
        self.linear_vel_values = []
        self.steer_angle_val = []

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,depth=10)


        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        self.imu_subscriber = self.create_subscription(Imu, '/imu_plugin/out', self.imu_callback, qos_profile)
        self.imu_subscriber
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.linear_velocity_x = 0.0  
        self.linear_velocity_y = 0.0
        self.current_heading = 0.0
        self.robot_yaw = 0.0

    def imu_callback(self, msg):
        dt = 0.008
        self.linear_velocity_x += msg.linear_acceleration.x * dt
        self.linear_velocity_y += msg.linear_acceleration.y * dt
        self.robot_x += self.linear_velocity_x * dt
        self.robot_y += self.linear_velocity_y * dt
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.robot_yaw = euler_from_quaternion(orientation_list)


    def calculate_error(self):
        error_x = TARGET_X - self.robot_x
        error_y = TARGET_Y - self.robot_y

        return error_x, error_y

    def proportional_control(self, error_x, error_y):
        k_p_x = 1.7
        k_p_y = 1.7
        k_p_theta = 0.4

        lin_x = k_p_x * error_x
        lin_y = k_p_y * error_y
        linear_vel = math.sqrt((k_p_x * error_x)**2 + (k_p_y * error_y)**2)

        current_heading = self.robot_yaw
        desired_heading = math.atan2(error_x, error_y)
        heading_error = desired_heading - current_heading
        steer_angle = k_p_theta * heading_error

        max_steer_angle = 0.3
        min_steer_angle = -0.3
        steer_angle = -max(min(steer_angle, max_steer_angle), min_steer_angle)

        return linear_vel, steer_angle
    
    def plot_graphs(self):
        plt.figure(figsize=(12, 6))
        
        plt.subplot(3, 2, 1)
        plt.plot(self.time_values, self.error_x_values)
        plt.xlabel('Time')
        plt.ylabel('Error_x')
        plt.title("Error in x-dir vs Time")
        
        plt.subplot(3, 2, 2)
        plt.plot(self.time_values, self.error_y_values)
        plt.xlabel('Time')
        plt.ylabel('Error_y')
        plt.title("Error in y-dir vs Time")
        
        plt.subplot(3, 2, 3)
        plt.plot(self.time_values, self.lin_x_values)
        plt.xlabel('Time')
        plt.ylabel('lin_x')
        plt.title("linear control output in X vs Time")
        
        plt.subplot(3, 2, 4)
        plt.plot(self.time_values, self.lin_y_values)
        plt.xlabel('Time')
        plt.ylabel('lin_y vs Time')
        plt.title('linear control output in Y vs Time')
        
        plt.subplot(3, 2, 5)
        plt.plot(self.time_values, self.linear_vel_values)
        plt.xlabel('Time')
        plt.ylabel('Linear Vel')
        plt.title('linear control output in Y vs Time')

        plt.subplot(3, 2, 6)
        plt.plot(self.time_values, self.steer_angle_val)
        plt.xlabel('Time')
        plt.ylabel('Steer Angle')
        plt.title('Steer Angle vs Time')

        plt.tight_layout()
        plt.show()

    def run_proportional_control(self):
        global global_time
        while rclpy.ok():
            error_x, error_y = self.calculate_error()
            linear_vel, steer_angle = self.proportional_control(error_x, error_y)
            wheel_velocities = Float64MultiArray()
            joint_positions = Float64MultiArray()
            dist = math.sqrt((TARGET_X-self.robot_x)**2 + (TARGET_Y-self.robot_y)**2)

            self.time_values.append(time.time()-global_time)
            self.error_x_values.append(error_x)
            self.error_y_values.append(error_y)
            self.lin_x_values.append(self.linear_velocity_x)
            self.lin_y_values.append(self.linear_velocity_y)
            self.linear_vel_values.append(linear_vel)
            self.steer_angle_val.append(steer_angle)

            if dist<2:
                wheel_velocities.data = [0.0, 0.0, 0.0, 0.0]
                joint_positions.data = [0.0, 0.0]

                self.joint_position_pub.publish(joint_positions)
                self.wheel_velocities_pub.publish(wheel_velocities)
                print("Destination reached")
                self.plot_graphs()
                break
        
            print('Publishing the wheel velocities: [{},{},{},{}]'.format(linear_vel, -linear_vel, linear_vel, -linear_vel))
            print('Publishing the joint positions: [{},{}]'.format(steer_angle, steer_angle))

            wheel_velocities.data = [linear_vel, -linear_vel, linear_vel, -linear_vel]
            joint_positions.data = [steer_angle, steer_angle]



            self.joint_position_pub.publish(joint_positions)
            self.wheel_velocities_pub.publish(wheel_velocities)


def main(args=None):
    rclpy.init(args=args)
    node = ProportionalControlNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    control_thread = Thread(target=node.run_proportional_control)
    control_thread.start()
    executor.spin()
    control_thread.join()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
