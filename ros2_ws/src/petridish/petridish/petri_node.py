


import sys
import random
import time
from typing import List

import socketio
import asyncio

import rclpy
from simple_node import Node
from recordclass import recordclass

from yasmin import State
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from petridish.robotiq_gripper_control import RobotiqGripper


ROBOT_IP = '192.168.1.21'
#ROBOT_IP = '192.168.56.101' # TODO(reverse comment)
ROBOT_PORT = 50002
RTDE_FREQUENCY = 500   # from 0 to 100 %

# Define hardcoded poses.
HOME_POSE = [-0.00325, 0.33498, 0.22984, 0.297, -2.442, -1.714]

DISPENSE_POSE = [-0.03691, 0.46891, 0.18000, 0.297, -2.442, -1.714]

CAM_APPROACH_POSE = [0.12212, 0.36511, 0.17907, 0.861, 1.599, 1.573]
CAM_INSPECT_POSE = [0.20506, 0.43051, 0.19843, 1.107, 1.337, 1.396]

BIN_APPROACH_POSE = [-0.17038, 0.30072, 0.19170, 1.023, -1.500, -1.478]
MOIST_BIN_POSE =  [-0.21200, 0.43950, 0.16420, 0.816, -1.766, -1.665]
UNDETERMINED_BIN_POSE = [-0.20119, 0.26190, 0.16961, 0.950, -1.643, -1.522]
BACTERIA_BIN_POSE =  [-0.25939, 0.12570, 0.16791, 1.161, -1.334, -1.279]

DEFAULT_CART_VEL = .25   # m/s
DEFAULT_CART_ACC = .5    # m / s^2
DEFAULT_MOVE_TIME = 2    # s


Context = recordclass(
    'Context', [
        'rtde_controller', 
        'rtde_receiver', 
        'end_signal',
        'home_pose',
        'dispense_pose',
        'cam_approach_pose',
        'cam_inspect_pose',
        'bin_approach_pose',
        'moist_bin_pose',
        'undetermined_bin_pose',
        'bacteria_bin_pose',
        'gripper',
        'culture_type',
        ])

# define state GoHome
class GoHome(State):
    def __init__(self):
        super().__init__(["end", "continue"])

    def execute(self, context: Context):
        print("Executing state HOME")
        controller = context.rtde_controller
        go_end = False
        try:
            if context.end_signal:
                go_end = True
            else:
                if not controller.moveL(context.home_pose, DEFAULT_CART_VEL, DEFAULT_CART_ACC, True):
                    go_end = True
                    # This sleep is needed for the move to complete
                else:
                    time.sleep(DEFAULT_MOVE_TIME)
        except KeyboardInterrupt:
            context.end_signal = True
        if not go_end:
            return "continue"
        context.end_signal = True
        return "end"


# define state GetDish
class GetDish(State):
    def __init__(self):
        super().__init__(outcomes=["success", "failure"])

    def execute(self, context: Context):
        print("Executing state GetDish")
        controller = context.rtde_controller
        failure = False
        try:
            # 1. Move to ge the plate from the dispenser
            failure = failure or not controller.moveL(context.dispense_pose, DEFAULT_CART_VEL, DEFAULT_CART_ACC, True)
            time.sleep(DEFAULT_MOVE_TIME)
            
            # 2. Close the gripper.
            context.gripper.close() # TODO(reverse comment)
            
            # 3. Return to home pose
            failure = failure or not controller.moveL(context.home_pose, DEFAULT_CART_VEL, DEFAULT_CART_ACC, True)
            time.sleep(DEFAULT_MOVE_TIME)

            # 3. Move to the inspection position to see the petridish
            failure = failure or not controller.moveL(context.cam_approach_pose, DEFAULT_CART_VEL, DEFAULT_CART_ACC, True)
            time.sleep(DEFAULT_MOVE_TIME)
            failure = failure or not controller.moveL(context.cam_inspect_pose, DEFAULT_CART_VEL, DEFAULT_CART_ACC, True)
            time.sleep(DEFAULT_MOVE_TIME)
            #context.culture_type = random.choice(['bacteria', 'moist', 'undetermined'])
            context.culture_type = 'bacteria'

        except KeyboardInterrupt:
            context.end_signal = True
        if failure:
            return "failure"
        return "success"
        
    
# define state PutInBin
class PutInBin(State):
    def __init__(self):
        super().__init__(["end", "success"])

    def execute(self, context: Context):
        print("Executing state PutInBin")
        controller = context.rtde_controller

        if context.end_signal:
            return "end"
        else:
            target_pose = context.undetermined_bin_pose
            culture_type = asyncio.run(get_culture_type())
            # TODO(maryam): Consider using pubsub for this feature.
            if culture_type == "bacteria":
                target_pose = context.bacteria_bin_pose
                print("Moving to bacteria bin.")
            elif culture_type == "moist":
                target_pose = context.moist_bin_pose
                print("Moving to moist bin.")
            else:
                print("Moving to undetermined bin.")

            # 0. Move to pre-approach pose.
            failure = not controller.moveL(context.bin_approach_pose, DEFAULT_CART_VEL, DEFAULT_CART_ACC, True)
            time.sleep(DEFAULT_MOVE_TIME)
            # 1. Move to the selected bin
            failure = failure or not controller.moveL(target_pose, DEFAULT_CART_VEL, DEFAULT_CART_ACC, True)
            time.sleep(DEFAULT_MOVE_TIME)
            # 2. Open the gripper.
            context.gripper.open() # TODO(reverse comment)
            time.sleep(DEFAULT_MOVE_TIME)
            # 3. Return to pre-approach pose.
            failure = not controller.moveL(context.bin_approach_pose, DEFAULT_CART_VEL, DEFAULT_CART_ACC, True)
            time.sleep(DEFAULT_MOVE_TIME)
            # 4. Return to home position.
            failure = not controller.moveL(context.home_pose, DEFAULT_CART_VEL, DEFAULT_CART_ACC, True)
            time.sleep(DEFAULT_MOVE_TIME)
            if failure:
                return "failure"
            return "success"       


class PetridishNode(Node):

    def __init__(self):
        super().__init__("yasmin_node")

        # create a state machine
        sm = StateMachine(outcomes=["EndState"])

        # add states
        sm.add_state("GoHome", GoHome(),
                     transitions={"continue": "GetDish",
                                  "end": "EndState"})
        sm.add_state("GetDish", GetDish(),
                     transitions={"success": "PutInBin",
                                  "failure": "GoHome"})
        sm.add_state("PutInBin", PutInBin(),
                     transitions={"success": "GetDish",
                                  "failure": "GoHome",
                                  "end": "EndState"})

        # pub
        YasminViewerPub(self, "PETRI_DEMO", sm)

        # Make the controller and initialize
        rtde_controller = RTDEControl(ROBOT_IP, RTDE_FREQUENCY)
        # rtde_controller = RTDEControl(
            # ROBOT_IP, RTDE_FREQUENCY, RTDEControl.FLAG_USE_EXT_UR_CAP, ROBOT_PORT)
        rtde_receiver = RTDEReceive(ROBOT_IP, RTDE_FREQUENCY)

        # Prepare the gripper
        gripper = RobotiqGripper(rtde_controller)
        print("Activating gripper...")
        gripper.activate()
        print("Closing gripper...")
        gripper.close()
        print("Opening gripper...")
        gripper.open()

        # Make the context
        context = Context(
            rtde_controller=rtde_controller,
            rtde_receiver=rtde_receiver,
            end_signal=False,
            home_pose=HOME_POSE,
            dispense_pose=DISPENSE_POSE,
            cam_approach_pose=CAM_APPROACH_POSE,
            cam_inspect_pose=CAM_INSPECT_POSE,
            bin_approach_pose=BIN_APPROACH_POSE,
            # We want the poses to be a part of the context (as opposed to hard coded) so in the future iterations we can read them from the camera.
            moist_bin_pose=MOIST_BIN_POSE,
            undetermined_bin_pose=UNDETERMINED_BIN_POSE,
            bacteria_bin_pose=BACTERIA_BIN_POSE,
            gripper=gripper,
            culture_type=get_culture_type(),
            )

        # execute
        outcome = sm.execute(context)
        print(outcome)

        # Clean up the resources
        # Stop the rtde control script
        rtde_controller.stopScript()


def main(args=None):
    print('Hi from petridish.')
    print("petridish_demo")
    rclpy.init(args=args)
    node = PetridishNode()
    node.join_spin()
    rclpy.shutdown()
    
async def get_culture_type():
    async with socketio.AsyncSimpleClient() as sio:
        print("connect:")
        await sio.connect('https://websocket-dot-aktus-393100.uw.r.appspot.com') 
        print("emit join room:") 
        await sio.emit('join_room', 'accenture')
        print("emit from robot")
        await sio.emit('from_robot', {'message': 'get_culture_type', 'room': 'accenture'})
        print("await receive:")
        from_model_message = await sio.receive()
        # TODO(maryam): Make sure the first element of the list is 'model_message'
        print(from_model_message[1])
        culture_type = from_model_message[1]['message']['culture']
        return culture_type


if __name__ == '__main__':
    main()
