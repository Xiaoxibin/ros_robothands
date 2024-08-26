#!/usr/bin/env python3

import rospy
import std_msgs.msg
# from seed_robotics.msg import *
from seed_robotics.msg import JointListSetSpeedPos, SetShutdownCond, JointSetSpeedPos
import time
import pygame




class JoystickControl:
    def __init__(self):
        rospy.init_node('Joystick_Control', anonymous=True)
        self.pub = rospy.Publisher('R_speed_position', JointListSetSpeedPos, queue_size=10)
        self.shutdown_pub = rospy.Publisher('R_shutdown_condition', SetShutdownCond, queue_size=10)

        self.joint_names = ['r_w_rotation', 'r_w_flexion', 'r_w_adduction', 'r_th_adduction', 'r_th_flexion',
                            'r_ix_flexion', 'r_middle_flexion', 'r_ring_ltl_flexion']

        pygame.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.num_axes = self.joystick.get_numaxes()
        self.num_buttons = self.joystick.get_numbuttons()


        self.target_positions = [2048] * 8
        self.target_speeds = [0] * 8

        # 发送消息来失能温度和过载
        self.disable_shutdown_conditions()

        self.update_message()

    def update_message(self):
        message_to_send = JointListSetSpeedPos()
        message_to_send.joints = []

        for name, position, speed in zip(self.joint_names, self.target_positions, self.target_speeds):
            joint_msg = JointSetSpeedPos()
            joint_msg.name = name
            joint_msg.target_pos = position
            joint_msg.target_speed = speed
            message_to_send.joints.append(joint_msg)

        self.pub.publish(message_to_send)

    def disable_shutdown_conditions(self):
        message_to_send = SetShutdownCond()
        message_to_send.name = 'r_ring_ltl_flexion'  # 设置关节名称
        message_to_send.temperature = False  # 失能温度
        message_to_send.overload = False  # 失能过载
        self.shutdown_pub.publish(message_to_send)

    def run(self):
        try:
            while not rospy.is_shutdown():
                pygame.event.pump()
                # if self.joystick.get_button(6):
                axis_values = [self.joystick.get_axis(i) for i in range(self.num_axes)]

                # 控制最后三个关节的位置，使用手柄第一个轴
                for i in range(5, 8):
                    self.target_positions[i] = int((axis_values[0] + 1) / 2 * 4095)
                    self.target_positions[i] = min(max(self.target_positions[i], 0), 4095)
                for i in range(3, 5):
                    self.target_positions[i] = int((axis_values[i] + 1) / 2 * 4095)
                    self.target_positions[i] = min(max(self.target_positions[i], 0), 4095)
                # # 控制前五个关节的位置，使用手柄的其他轴
                # for i in range(5):
                #     if abs(axis_values[i]) > 0.1:  # 设定死区范围
                #         self.target_positions[i] += int(axis_values[i] * 100)
                #         if self.target_positions[i] > 4095:
                #             self.target_positions[i] = 4095
                #         elif self.target_positions[i] < 0:
                #             self.target_positions[i] = 0

                self.update_message()
                time.sleep(0.05)  # 调整此处延迟以平滑控制
        except KeyboardInterrupt:
            pass

if __name__ == "__main__":
    joystick_control = JoystickControl()
    joystick_control.run()
