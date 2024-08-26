#!/usr/bin/env python3

# This sample is done for a RH8D Right Hand
# You can adapt it to a right Hand by changing the 'l' by a 'r' on joint names
# User sample code High Level Logic : When an object is close to the Hand, the fingers close to grab the object. Once it's grabbed, the finger open themselves 5seconds later
# Several things are done here :
# - Get real-time data for each joint
# - Get real-time data from the IR sensor in the palm of the hand
# - Send the instruction to close the index, the ring and the little fingers
# - Send the instruction to close the thumb
# - Continuously checking if any joint is too stressed, i.e. if its current goes above the current limit hardcoded to 300mA
# - If a joint is too stressed, send an instruction to set its target position to its present position
# - When every joint have their present position equal to their target position -> open the finger and let the object go

import rospy
import std_msgs.msg
from seed_robotics.msg import *
import time

# Hardcoded value of the current limit, if a joint goes above that limit it must stop forcing
CURRENT_LIMIT = 300 #mAmp

# Control class to be able to change the IR sensor value in the callback function
# Including 2 flags to control the main loop
class Control:
    def __init__(self):
        self.IR_sensor_value = 254
        self.start_flag      = False
        self.step2_flag      = False


# Initialize an instance of the Control class
control = Control()

# Initialize variables to contunuisly store to incoming informations
joint_list = [LoneJoint() for i in range (5)] # We will only need informations about 4 joints : r_th_adduction, r_th_flexion, r_ix_flexion, r_middle_flexion, r_ring_ltr_flexion

# Initialize variables and structures to send messages
names_step_1            = ['r_ix_flexion','r_middle_flexion','r_ring_ltl_flexion']                # Name of the joints we want to move first
target_positions_step_1 = [4095, 4095, 4095]                                                    # Maximum position value : closed position
target_speeds_step_1    = [100, 100, 100]                                                           # Speed 0 : Highest speed. Speed 50 : Low speed

# 2
names_step_2            = ['r_ix_flexion','r_middle_flexion']                                                      # Name of the joint to move on the 2nd step of the hand closing
target_positions_step_2 = [0,0]                                                                # Maximum position value : closed position
target_speeds_step_2    = [50,50]                                                                  # Speed 50 : Low speed

# 0
names_step_3            = ['r_ix_flexion','r_middle_flexion'] # Names of all the joints to move. Step 3 will be to open each finger
target_positions_step_3 = [4000,4000]                                                          # Minimum position value : open position
target_speeds_step_3    = [50,50]                                                      # Speed 10 : very low speed


# 4
names_step_4            = ['r_ix_flexion','r_middle_flexion','r_ring_ltl_flexion'] # Names of all the joints to move. Step 3 will be to open each finger
target_positions_step_4 = [10, 10, 10]                                                          # Minimum position value : open position
target_speeds_step_4    = [ 10, 10, 10] 

# left right
names_step_5            = ['r_w_adduction'] # Names of all the joints to move. Step 3 will be to open each finger
target_positions_step_5 = [1024]                                                          # Minimum position value : open position
target_speeds_step_5    = [20] 
# left right
names_step_6            = ['r_w_adduction'] # Names of all the joints to move. Step 3 will be to open each finger
target_positions_step_6 = [3070]                                                          # Minimum position value : open position
target_speeds_step_6    = [20] 

# Define a function to fill the message 'finar_message' to send based on lists of names, target position values and target speed values
def buildSpeedPosMsg(names,target_positions,target_speeds):
    # Initialize a list of JointSetSpeedPos messages, the length of the number of joints we want to send instruction to
    joint_list = [JointSetSpeedPos() for i in range(len(names))]
    # Fill up that list with the informations about name, position and speed that are listed above
    for name, position, speed, joint in zip(names, target_positions, target_speeds, joint_list):
        joint.name = name
        joint.target_pos = position
        joint.target_speed = speed
    # Declare, fill up and return an instance of JointListSetSpeedPos message, ready to be sent
    finar_message = JointListSetSpeedPos()
    finar_message.joints = joint_list
    return finar_message

# Callback function called when a AllJoints message is published.
# Will fill up the joint_list with each LoneJoint messages that are received that interests us. (i.e thumb adduction, thumb flexion, index flexion and ring/little flexion)
def jointsCallback(joints_data):
    for joint in joints_data.joints:
        if joint.name == 'r_th_adduction':
            joint_list[0] = joint
        if joint.name == 'r_th_flexion':
            joint_list[1] = joint
        if joint.name == 'r_ix_flexion':
            joint_list[2] = joint
        if joint.name == 'r_middle_flexion':
            joint_list[3] = joint
        if joint.name == 'r_ring_ltl_flexion':
            joint_list[4] = joint

# Callback function called when a message about the main board is published
# Will update the value of the palm IR sensor
def mainBoardCallback(main_board_data):
    for board in main_board_data.boards:
        if board.name == "r_main_board":
            control.IR_sensor_value = board.palm_IR_sensor
    #print("IR Sensor value = %d" % IR_sensor_value)


# Initialize a ROS Node
rospy.init_node('listener', anonymous = True)
# Subscribe to the Joints Topic to receive AllJoints messages that will be processed by the jointsCallback function
# Note that the Topic name MUST be 'Joints'
rospy.Subscriber("R_Joints", AllJoints, jointsCallback)
# Subscribe to the Main_Boards Topic to receive AllMainBoards messages that will be processed by the mainBoardCallback function
# Note that the Topic name MUST be 'Main_Boards'
rospy.Subscriber('R_Main_Boards', AllMainBoards, mainBoardCallback)
# Initialize a Publisher to the 'speed_position' Topic. This MUST be the name of the topic so that the data can be processed
# The publisher will publish JointListSetSpeedPos messages
pub = rospy.Publisher('R_speed_position', JointListSetSpeedPos, queue_size = 10)

# Define a function that takes a LoneJoint message instance in argument
# It will publish a message to that joint to set its target position to its present position
# The idea is to stop stressing the joint
def stopStressing(joint):
    # Getting the joint's present position
    target_pos = joint.present_position
    # Declare a list of 1 JointSetSpeedPos element
    joints = [JointSetSpeedPos()]
    # Fill the JointSetSpeedPos instance with joint's name, new target position and target_speed
    joints[0].name = joint.name
    joints[0].target_pos = target_pos
    joints[0].target_speed = -1         # If targert speed = -1, then the parameter will be ignored
    # Declare an instance of JointListSetSpeedPos that will be the message to send
    message = JointListSetSpeedPos()
    # Fill that message and publish it
    message.joints = joints
    pub.publish(message)

# Declaring a empty list to store future stressed joints
list_stressed_joints = []

# Sleeping for 1sec to let ROS initialize
time.sleep(1)

# Main Loop
while not rospy.is_shutdown():
    # If start flag is false, then try to do the first step
    if control.start_flag is False:
        # If the IR sensor value is below 20, then there is an object to grab -> do the first step
        
        # 1,2324. 0 2024 
        # first step
        message = buildSpeedPosMsg(names_step_1,target_positions_step_1,target_speeds_step_1)
        print(message)
        pub.publish(message)

        time.sleep(2)

        # 2
        message = buildSpeedPosMsg(names_step_2,target_positions_step_2,target_speeds_step_2)
        print(message)
        pub.publish(message)
        
        time.sleep(2)
        # 0
        message = buildSpeedPosMsg(names_step_3,target_positions_step_3,target_speeds_step_3)
        print(message)
        pub.publish(message)

        time.sleep(2)
        # 2
        message = buildSpeedPosMsg(names_step_2,target_positions_step_2,target_speeds_step_2)
        print(message)
        pub.publish(message)

        time.sleep(2)
        # 4
        message = buildSpeedPosMsg(names_step_4,target_positions_step_4,target_speeds_step_4)
        print(message)
        pub.publish(message)
        time.sleep(2)
        
        # 2
        message = buildSpeedPosMsg(names_step_2,target_positions_step_2,target_speeds_step_2)
        print(message)
        pub.publish(message)

        time.sleep(2)
        # l
        message = buildSpeedPosMsg(names_step_5,target_positions_step_5,target_speeds_step_5)
        print(message)
        pub.publish(message)
        time.sleep(3)
        # r
        message = buildSpeedPosMsg(names_step_6,target_positions_step_6,target_speeds_step_6)
        print(message)
        pub.publish(message)
        time.sleep(3)


        quit()