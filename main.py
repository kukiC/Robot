#!/usr/bin/env python

import rospy
import moveit_commander
import std_msgs.msg as ros_msg
import threading as thr
from copy import deepcopy as dpcpy
from functools import partial
import baxter_external_devices

from data_looping_poses import *
from data_pose_joint_values import *
from bedbot_tools import array_to_pose, pose_to_array
import bot_ctrl
from gripper_ctrl import GripCtrller



# msg_addrs
# nlp_msg_addr = "/home/y/ws_baxter/src/baxter_bedbot/textmsg/nlp_msg.txt"
# gripper_msg_addr = "/home/y/ws_baxter/src/baxter_bedbot/textmsg/gripper_msg.txt"

# global definition
camera_color2num = {
    "green": 0,
    "blue": 1,
    "red": 2,
    "orange": 3
}

camera_num2color = dict([[value, key] for key, value in camera_color2num.items()])

state_dic = {
    "ready": 0,
    "grab": 1,
    "make_bed": 2,
    "pause_nlp": 3,
    "pause_sonar": 4,
    "moving_base": 5,
    "terminate": 6
}

wait_ids = {
    "nlp": 0,
    "sonar_warn": 1
}

wait_id2waitNum = [3, 4]

nlp_command_dic = {
    "grab": 1,
    "make_bed": 2,
    "pause": 3,
    "continue": 5,
    "terminate": 6,
}

# topic names
state_publish_topic_name = "baxter_state_topic"
sonar_warning_topic_name = "/sonar_state"
cam_request_topic_name ="/camera/request"
cam_coord_topic_name = "/camera/coordinate"
gripperL_ctrl_topic_name = "temp1"
gripperR_ctrl_topic_name = "temp2"
gripper_request_topic_name = "temp3"
gripperL_response_topic_name = "temp4"
gripperR_response_topic_name = "temp5"
nlp_topic_name = "/nlp_state"

# flags
terminate_flag = False
camera_request_wait_flag = False

# parameters
camera_coord_dict = {}
gripperL_grab_response = None   # None: no response | 1: success | 0: unsuccess
gripperR_grab_response = None   # None: no response | 1: success | 0: unsuccess




def routine_grab(activity_manager_obj, state_manager, gripper_ctrl):
    # Clear activities
    activity_manager_obj.immediate_stop()
    activity_manager_obj.unpause_activity(wait_id=wait_ids["nlp"])
    activity_manager_obj.add_activity(["SetState", (state_dic["grab"],)])

    # Tuckle
    activity_manager_obj.add_activity(["Gripper", ("BOTH", "open")])
    activity_manager_obj.add_activity(["MoveJoint", ("L", bedbot_init_joint_value[:7])])
    activity_manager_obj.add_activity(["MoveJoint", ("R", bedbot_init_joint_value[7:])])
    while activity_manager_obj.state() != "Idle":
        rospy.sleep(0.5)
    
    # Ask for points on sheet
    activity_manager_obj.add_activity(["CamRequest", ("blue", "green")])
    rospy.sleep(2)
    while activity_manager_obj.state() != "Idle":
        rospy.sleep(0.5)
    coord1 = list(camera_coord_dict["blue"])
    coord2 = list(camera_coord_dict["green"])

    # Move to point
    if coord1[1] > coord2[1]:
        coordL = coord1
        coordR = coord2
    else:
        coordL = coord2
        coordR = coord1
        poseL_arr = coord2 + degree_90_orientation
        poseR_arr = coord1 + degree_90_orientation
    # x - cali
    coordL[0] += 0.12
    coordR[0] += 0.07
    # # y - cali
    # coordL[1] += 0.05
    # coordR[1] += 0.05
    # z - cali
    # coordL[2] += -0.21
    # coordR[2] += -0.21

    coordL[2] = -0.421
    coordR[2] = -0.421
    poseL_arr = coordL + degree_90_orientation
    poseR_arr = coordR + degree_90_orientation
    poseL_arr[2] += 0.2
    poseR_arr[2] += 0.2
    activity_manager_obj.add_activity(["MoveEndEffector_cart", ("L", poseL_arr)])
    activity_manager_obj.add_activity(["MoveEndEffector_cart", ("R", poseR_arr)])
    while activity_manager_obj.state() != "Idle":
        rospy.sleep(0.5)

    # Reach to points and grab
    poseL_arr[2] -= 0.203
    poseR_arr[2] -= 0.203
    activity_manager_obj.add_activity(["MoveEndEffector_cart", ("L", poseL_arr)])
    activity_manager_obj.add_activity(["Gripper", ("L", "close")])
    activity_manager_obj.add_activity(["MoveEndEffector_cart", ("R", poseR_arr)])
    activity_manager_obj.add_activity(["Gripper", ("R", "close")])
    while activity_manager_obj.state() != "Idle":
        rospy.sleep(0.5)

    # Check whether grab succeeds
    # ..........

    activity_manager_obj.add_activity(["SetState", (state_dic["ready"],)])


def routine_makebed(activity_manager_obj, state_manager, gripper_ctrl):
    activity_manager_obj.immediate_stop()
    activity_manager_obj.unpause_activity(wait_id=wait_ids["nlp"])  # Automatically restore state
    activity_manager_obj.add_activity(["SetState", (state_dic["make_bed"],)])

    # Move to side
    poseL_arr = [0.5, 0.6, -0.1] + degree_90_orientation
    poseR_arr = [0.5, -0.6, -0.1] + degree_90_orientation
    activity_manager_obj.add_activity(["MoveEndEffector_cart", ("L", poseL_arr)])
    activity_manager_obj.add_activity(["MoveEndEffector_cart", ("R", poseR_arr)])
    while activity_manager_obj.state() != "Idle":
        rospy.sleep(0.5)

    # Move front+up step1/2
    poseL_arr[0] += 0.25    # x
    poseR_arr[0] += 0.25    # x
    poseL_arr[2] += 0.15    # z
    poseR_arr[2] += 0.15    # z
    poseL_arr = poseL_arr[0:3] + degree_45_orientation
    poseR_arr = poseR_arr[0:3] + degree_45_orientation
    activity_manager_obj.add_activity(["MoveEndEffector_cart", ("L", poseL_arr)])
    activity_manager_obj.add_activity(["MoveEndEffector_cart", ("R", poseR_arr)])
    while activity_manager_obj.state() != "Idle":
        rospy.sleep(0.5)

    # Move front+up step2/2
    poseL_arr[0] = 1.18    # x
    poseR_arr[0] = 1.18    # x
    poseL_arr[2] = 0.65    # z
    poseR_arr[2] = 0.65    # z
    poseL_arr = poseL_arr[0:3] + degree_0_orientation
    poseR_arr = poseR_arr[0:3] + degree_0_orientation
    activity_manager_obj.add_activity(["MoveEndEffector_cart", ("L", poseL_arr)])
    activity_manager_obj.add_activity(["MoveEndEffector_cart", ("R", poseR_arr)])
    while activity_manager_obj.state() != "Idle":
        rospy.sleep(0.5)
    
    # Move back_down step1/2
    poseL_arr[0] -= 0.3    # x
    poseR_arr[0] -= 0.3    # x
    poseL_arr[2] -= 0.4    # z
    poseR_arr[2] -= 0.4    # z
    poseL_arr = poseL_arr[0:3] + degree_45_orientation
    poseR_arr = poseR_arr[0:3] + degree_45_orientation
    activity_manager_obj.add_activity(["MoveEndEffector_cart", ("L", poseL_arr)])
    activity_manager_obj.add_activity(["MoveEndEffector_cart", ("R", poseR_arr)])
    while activity_manager_obj.state() != "Idle":
        rospy.sleep(0.5)
    
    # Move back_down step2/2
    poseL_arr[0] = 0.42    # x
    poseR_arr[0] = 0.42    # x
    poseL_arr[2] -= 0.3    # z
    poseR_arr[2] -= 0.3    # z
    poseL_arr = poseL_arr[0:3] + degree_90_orientation
    poseR_arr = poseR_arr[0:3] + degree_90_orientation
    activity_manager_obj.add_activity(["MoveEndEffector_cart", ("L", poseL_arr)])
    activity_manager_obj.add_activity(["MoveEndEffector_cart", ("R", poseR_arr)])
    while activity_manager_obj.state() != "Idle":
        rospy.sleep(0.5)
    
    # Open grippers
    activity_manager_obj.add_activity(["Gripper", ("BOTH", "open")])
    activity_manager_obj.add_activity(["SetState", (state_dic["ready"],)])



def nlp_callfunc(activity_manager_obj, state_manager, gripper_ctrl, data):
    # "gripper_ctrl" is to check grabbing status.

    global terminate_flag

    c = data.data


    if c == nlp_command_dic["grab"]:    # Grab
        print("NLP Command {}: Grabbing!".format(c))
        routine_grab_thr = thr.Thread(target=routine_grab, args=(activity_manager_obj, state_manager, gripper_ctrl))
        routine_grab_thr.start()

    if c == nlp_command_dic["make_bed"]:    # Make bed
        print("NLP Command {}: Making bed!".format(c))
        routine_makebed_thr = thr.Thread(target=routine_makebed, args=(activity_manager_obj, state_manager, gripper_ctrl))
        routine_makebed_thr.start()


    elif c == nlp_command_dic["pause"]:  # Pause
        print("NLP Command {}: Pausing!".format(c))
        activity_manager_obj.pause_activity(wait_id=wait_ids["nlp"])
        state_manager.set_state(state_dic["pause_nlp"])
    

    elif c == nlp_command_dic["continue"]:    # Continue
        activity_manager_obj.unpause_activity(wait_id=wait_ids["nlp"])
    
    
    elif c == nlp_command_dic["terminate"]:    # Terminate
        print("NLP Command {}: Terminating!".format(c))
        terminate_flag = True
        activity_manager_obj.immediate_stop()
        state_manager.set_state(state_dic["terminate"])
        while activity_manager_obj.state() != "Idle":
            rospy.sleep(0.5)
        activity_manager_obj.terminate()
    
    else:
        print("NLP Thr: Ignoring command code '{}'".format(c))


def gripL_response_callfunc(data):
    global gripperL_grab_response
    gripperL_grab_response = data.data


def gripR_response_callfunc(data):
    global gripperR_grab_response
    gripperR_grab_response = data.data


def key_press_thrfunc(act_manager):
    # A function thread for termination
    # When X is pressed, terminates the program
    global terminate_flag, camera_request_wait_flag
    
    while terminate_flag is False:
        key = baxter_external_devices.getch()
        if key == "x" or key == "X":
            print("============ Key Interruption...\n")
            camera_request_wait_flag = False
            terminate_flag = True
            act_manager.terminate()
            print "Shutting down roscpp"
            moveit_commander.roscpp_shutdown()
            print "Shutting down os"
            moveit_commander.os._exit(0)
            rospy.sleep(0.5)
            print "Exitting"
            exit(1)
        rospy.sleep(0.005)


class StatePublisher(object):
    def __init__(self, init_state=state_dic["ready"]):
        self.state = init_state
        self.pub = rospy.Publisher(state_publish_topic_name, ros_msg.Int16, queue_size=10)
    
    def set_state(self, state_number):
        if state_number != self.state:
            self.state = state_number
            self.pub.publish(ros_msg.Int16(self.state))


def sonar_warning_callfunc(activity_manager_obj, state_manager, data_msg):
    code = data_msg.data
    
    if code == 1:
        print("Sonar Command {}: Stopping!".format(code))
        activity_manager_obj.pause_activity(wait_id=wait_ids["sonar_warn"])     # Automatically store current state

    elif code == 0:
        print("Sonar Command {}: Continuing!".format(code))
        activity_manager_obj.unpause_activity(wait_id=wait_ids["sonar_warn"])   # Automatically sets the state back

    else:
        print("Sonar Sub: Ignoring command code '{}'".format(code))
    
    # pass


def camera_coord_callfunc(data):
    global camera_coord_dict

    arr = data.data

    existances = [round(arr[4*i])==1 for i in range(len(camera_num2color))]
    coords = [list(arr[4*i+1:4*(i+1)]) for i in range(len(camera_num2color))]

    for i in range(len(camera_num2color)):
        if existances[i] == True:
            camera_coord_dict[camera_num2color[i]] = coords[i]


class ActivityQueue(object):
    def __init__(self, groups, grip_ctrller, state_manager, camera_request_pub, pause_start=False):
        self.groups = groups
        self.grip_ctrller = grip_ctrller
        self.state_manager = state_manager
        self.cam_req_sub = camera_request_pub

        self.activities = []
        # Here are all possible actions:    |   Format: ["ActionType", (arg1, arg2, ...)]
        #   1. ["MoveEndEffector", ("L"/"R", position_data,)]
        #   2. ["MoveEndEffector_cart", ("L"/"R", position_data,)]
        #   3. ["MoveJoint", ("L"/"R", joint_data,)]
        #   4. ["Gripper", ("L"/"R"/"BOTH", "close"/"open")]
        #   5. ["SetState", (state_number,)]
        #   6. ["PauseAct", (seconds,)]
        #   7. ["CamRequest", ("green", "blue",)]

        self.waits = [pause_start, False]    # [nlp_pause, snoar_pause]
        self.state_before_pause = None

        self.thr = thr.Thread(target=self.activity_thrfunc)
        self.thr.start()
    
    def add_activity(self, task, prioritize=False):
        if prioritize is False:
            self.activities.append(task)
        else:
            self.activities = [task] + self.activities
    
    def state(self):    # Returns current activity
        if True in self.waits:
            return "Wait"
        elif len(self.activities) == 0:
            return "Idle"
        else:
            code = self.activities[0][0]
            return code
    
    def skip_task(self):
        if len(self.activities) > 0:
            self.activities.pop(0)
    
    def pause_activity(self, wait_id):      # Automatically changes state
        if sum(self.waits) == 0:
            self.state_before_pause = self.state_manager.state
        self.waits[wait_id] = True
        self.state_manager.set_state(wait_id2waitNum[wait_id])
        self.groups[0].stop()
        self.groups[1].stop()
    
    def unpause_activity(self, wait_id):    # Automatically changes state
        self.waits[wait_id] = False
        if sum(self.waits) == 0:
            self.state_manager.set_state(self.state_before_pause)
            self.state_before_pause = None
    
    def immediate_stop(self):
        self.activities = []
        self.groups[0].stop()
        self.groups[1].stop()
    
    def terminate(self):
        self.immediate_stop()
        self.thr.join()
    
    def activity_thrfunc(self):
        global camera_coord_dict
        while terminate_flag is False:
            # print "ActivityLoop"
            if len(self.activities)==0 or (True in self.waits):
                # print "Stopping/no activity"
                rospy.sleep(0.3)
            else:
                code = self.activities[0][0]
                args = self.activities[0][1]

                if code == "MoveEndEffector":
                    l_or_r = args[0]
                    position_data = args[1]
                    if l_or_r == "L":
                        bot_ctrl.act_move_end_effect(self.groups[0], position_data)
                    elif l_or_r == "R":
                        bot_ctrl.act_move_end_effect(self.groups[1], position_data)
                    else:
                        raise(self.UnkownActionArgs())
                    if not (True in self.waits):  # If activity finished natually instead of been paused...
                        self.skip_task()    # Remove activity after finished

                elif code == "MoveEndEffector_cart":
                    l_or_r = args[0]
                    position_data = args[1]
                    if l_or_r == "L":
                        bot_ctrl.act_move_end_effect_cartision(self.groups[0], position_data)
                    elif l_or_r == "R":
                        bot_ctrl.act_move_end_effect_cartision(self.groups[1], position_data)
                    else:
                        raise(self.UnkownActionArgs())
                    if not (True in self.waits):  # If activity finished natually instead of been paused...
                        self.skip_task()    # Remove activity after finished

                elif code == "MoveJoint":
                    l_or_r = args[0]
                    joint_data = args[1]
                    if l_or_r == "L":
                        bot_ctrl.act_move_joint(self.groups[0], joint_data)
                    elif l_or_r == "R":
                        bot_ctrl.act_move_joint(self.groups[1], joint_data)
                    else:
                        raise(self.UnkownActionArgs())
                    if not (True in self.waits):  # If activity finished natually instead of been paused...
                        self.skip_task()    # Remove activity after finished

                elif code == "Gripper":
                    if args[0] == "L" or args[0] == "BOTH":
                        if args[1] == "close":
                            self.grip_ctrller.close_left()
                        elif args[1] == "open":
                            self.grip_ctrller.open_left()
                    if args[0] == "R" or args[0] == "BOTH":
                        if args[1] == "close":
                            self.grip_ctrller.close_right()
                        elif args[1] == "open":
                            self.grip_ctrller.open_right()
                    self.skip_task()        # Remove activity after finished
                
                elif code == "SetState":
                    state_code = args[0]
                    self.state_manager.set_state(state_code)
                    self.skip_task()        # Remove activity after finished

                elif code == "PauseAct":
                    t = args[0]
                    rospy.sleep(t)
                    self.skip_task()        # Remove activity after finished
                
                elif code == "CamRequest":
                    camera_coord_dict = {}

                    while terminate_flag is False:
                        # Request
                        print "Requesting from camera..."
                        self.cam_req_sub.publish(ros_msg.Int8(1))
                        rospy.sleep(1)
                        
                        # Check existance
                        all_collected = True
                        for arg in args:
                            try:
                                camera_coord_dict[arg]
                            except KeyError:
                                print "Color \"{}\" not found...".format(arg)
                                all_collected = False
                                break
                        print all_collected
                        if all_collected is True:
                            break
                    self.skip_task()

                else:
                    raise(self.UnkownActionType("Unrecognized ActionType \"{}\".".format(code)))
        self.groups[0].stop()
        self.groups[1].stop()
        self.state_manager.set_state(state_dic["terminate"])

    class UnkownActionType(Exception):
        pass
    class UnkownActionArgs(Exception):
        pass


def main():
    global terminate_flag, camera_request_wait_flag, camera_coord_dict

    try:
        """ MoveIt Initialization """
        rospy.init_node('moveit_baxter_master')
        print "============ Initializing moveit commander..."
        joint_state_topic = ['joint_states:=/robot/joint_states']
        moveit_commander.roscpp_initialize(joint_state_topic)
        print "============ Initializing parameters..."
        robot = moveit_commander.RobotCommander()
        # print "============ Available Planning Groups:"
        # print robot.get_group_names()

        """ Publisher Ctrllers """
        camera_request_pub = rospy.Publisher(cam_request_topic_name, ros_msg.Int8, queue_size=10)

        """ Local Classes """
        state_manager = StatePublisher(init_state=0)
        grip_ctrller = GripCtrller(simulation=False)
        # group = moveit_commander.MoveGroupCommander("both_arms")
        groupL = moveit_commander.MoveGroupCommander("left_arm", wait_for_servers=50)
        groupR = moveit_commander.MoveGroupCommander("right_arm", wait_for_servers=50)
        groups = (groupL, groupR)
        act_manager = ActivityQueue(groups, grip_ctrller, state_manager, camera_request_pub, pause_start=False)  # Initialize activity manager

        """ Local Threads """
        keyboard_thr = thr.Thread(target=key_press_thrfunc, args=(act_manager,))  # Start keyboard interrupt thread

        """ Subscriber Ctrllers """
        sonar_warning_sub = rospy.Subscriber(
            sonar_warning_topic_name,
            ros_msg.Int32,
            callback=partial(sonar_warning_callfunc, act_manager, state_manager)
        )
        camera_coord_sub = rospy.Subscriber(cam_coord_topic_name, ros_msg.Float64MultiArray, callback=camera_coord_callfunc)
        nlp_sub = rospy.Subscriber(nlp_topic_name, ros_msg.Int32, callback=partial(nlp_callfunc, act_manager, state_manager, grip_ctrller))

        print "============ Intializing"

        act_manager.add_activity(["MoveJoint", ("L", tuckle_joint_values[:7])])
        act_manager.add_activity(["MoveJoint", ("R", tuckle_joint_values[7:])])
        grip_ctrller.initialize()
        while act_manager.state() != "Idle":
            rospy.sleep(0.3)

        # Start!!!
        print "============ Starting Actions! Press X to terminate"

        keyboard_thr.start()

        while terminate_flag is False:
            rospy.sleep(0.5)


        print "============ Exiting"
        groups[0].stop()
        groups[1].stop()
        return

    except rospy.ROSInterruptException:
        print "Shutting down"
        return
    except KeyboardInterrupt:
        print "Shutting down"
        return

if __name__ == '__main__':
    main()
