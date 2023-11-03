#!/usr/bin/env python3
import rospy
import rospkg
import os
from enum import Enum

from stretch_ik.ik import StretchDexIK
from stretch_ik.utils import rpy_to_rotation_matrix
import hello_helpers.hello_misc as hm

##################################################
# Enums
##################################################

class MODES(Enum):
    IDLE = 0
    POSE_IDLE = 1
    GET_POSE = 2
    MOVE_TO_POSE = 3
    HOME = 4
    STOW = 5
    QUIT = 99

##################################################
# Menus
##################################################

def print_main_menu():
    n = 10
    print("=" * n + " MAIN MENU " + "=" * n)
    print("(M) Move to Pose")
    print("(H) Home")
    print("(S) Stow")
    print("(Q) Quit")

def parse_main_menu(ui: str) -> MODES:
    if ui == "m":
        return MODES.POSE_IDLE
    elif ui == "h":
        return MODES.HOME
    elif ui == "s":
        return MODES.STOW
    elif ui == "q":
        return MODES.QUIT
    else:
        return MODES.IDLE

def print_pose_menu():
    n = 10
    print("=" * n + " POSE MENU " + "=" * n)
    print("(E) Enter Pose")
    print("(G) Go to Pose")
    print("(Q) Back to Main Menu")

def parse_pose_menu(ui: str) -> MODES:
    if ui == "e":
        return MODES.GET_POSE
    elif ui == "g":
        return MODES.MOVE_TO_POSE
    elif ui == "q":
        return MODES.IDLE
    else:
        return MODES.IDLE

##################################################
# UI
##################################################

def get_target_pose_ui() -> list:
    """
    Prompts user for a pose (x, y, z, roll, pitch, yaw)
    
    Returns:
        list: [x, y, z, roll, pitch, yaw]
    """

    x = get_input("x: ")
    y = get_input("y: ")
    z = get_input("z: ")
    roll = get_input("roll: ")
    pitch = get_input("pitch: ")
    yaw = get_input("yaw: ")

    return [
        float(x),
        float(y),
        float(z),
        float(roll),
        float(pitch),
        float(yaw)
    ]

def get_input(prompt_str: str="Input: ") -> str:
    """
    Returns lowercase user input.

    Args:
        prompt_str (str): prompt for user input
    
    Returns:
        str: lowercase user input
    """

    print(' ')
    return input(prompt_str).lower()

##################################################
# Node
##################################################

class DemoNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        self.mode = MODES.IDLE
        self.latest_target_pose = None

        package_path = rospkg.RosPack().get_path('stretch_ik')
        urdf_path = os.path.join(package_path, "urdf/stretch.urdf")
        rospy.loginfo(f"DemoNode::__init__: Loading URDF from {urdf_path}")
        self.ik_engine = StretchDexIK(urdf_path)
    
    def home(self):
        """
        Moves robot to home position.
        """

        self.move_to_pose({
            "joint_lift": 0.6,
            "wrist_extension": 0.1,
            "joint_wrist_yaw": 0.0,
            "joint_wrist_pitch": 0.0,
            "joint_wrist_roll": 0.0
        })

    def stow(self):
        """
        Moves robot to stowed position.
        """

        self.move_to_pose({
            "joint_lift": 0.2,
            "wrist_extension": 0.05,
            "joint_wrist_yaw": 3.14,
            "joint_wrist_pitch": 0.0,
            "joint_wrist_roll": 0.0
        })

    def move(self):
        # Check if pose is set
        if self.latest_target_pose is None:
            rospy.logwarn("DemoNode::move: No target pose set, aborting...")
            return

        # Confirm with user
        print("Move to pose {}?".format(self.latest_target_pose))
        ui = get_input("(Y/N): ")
        if ui != "y":
            return

        # extract target position and orientation, convert to rotation matrix for IK
        target = self.latest_target_pose
        target_pos = target[0:3]
        target_rpy = target[3:6]
        target_ori = rpy_to_rotation_matrix(target_rpy)

        # attempt IK solve
        try:
            ik_soln = self.ik_engine.solve_ik(target_pos=target_pos, target_ori=target_ori)
        except Exception as e:
            print(e)
            return

        # go
        self.move_to_pose(ik_soln)

    def main(self):
        hm.HelloNode.main(self, 'stretch_controller', 'stretch_namespace', wait_for_first_pointcloud=False)
        
        # main loop!
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.mode == MODES.IDLE:
                print_main_menu()
                self.mode = parse_main_menu(get_input())

            elif self.mode == MODES.POSE_IDLE:
                print_pose_menu()
                self.mode = parse_pose_menu(get_input())

            elif self.mode == MODES.GET_POSE:
                self.latest_target_pose = get_target_pose_ui()
                self.mode = MODES.POSE_IDLE

            elif self.mode == MODES.MOVE_TO_POSE:
                self.move()
                self.mode = MODES.IDLE

            elif self.mode == MODES.HOME:
                self.home()
                self.mode = MODES.IDLE

            elif self.mode == MODES.STOW:
                self.stow()
                self.mode = MODES.IDLE

            elif self.mode == MODES.QUIT:
                break

            else:
                rospy.logwarn("DemoNode::main: Invalid mode, resetting to idle...")
                self.mode = MODES.IDLE

            rate.sleep()

##################################################
# Main
##################################################

if __name__ == '__main__':
    try:
        node = DemoNode()
        node.main()
    except rospy.ROSInterruptException:
        pass
