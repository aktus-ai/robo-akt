#!/usr/bin/env python3

import time

from rtde_control import RTDEControlInterface as RTDEControl

from petridish.robotiq_gripper_control import RobotiqGripper


ROBOT_IP = '192.168.1.21'
ROBOT_IP = '192.168.56.101'
ROBOT_PORT = 50002
RTDE_FREQUENCY = 500   # from 0 to 100 %
        

controller = RTDEControl(ROBOT_IP, RTDE_FREQUENCY)
# controller = RTDEControl(
    # ROBOT_IP, RTDE_FREQUENCY, RTDEControl.FLAG_USE_EXT_UR_CAP, ROBOT_PORT)
print(controller.moveL([-0.247, 0.289, 0.474, 1.986, -2.542, 0.107], .135, .14, True))
time.sleep(5)
controller.stopL(0.5)

# Prepare the gripper
print("Creating gripper...")
gripper = RobotiqGripper(controller)
print("Activating gripper...")
gripper.activate()
print("Closing gripper...")
gripper.close()
print("Opening gripper...")
gripper.open()