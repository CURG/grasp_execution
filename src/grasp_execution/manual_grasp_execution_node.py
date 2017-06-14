#!/usr/bin/env python
from grasp_execution_node import GraspExecutionNode
import rospy

if __name__ == '__main__':

    ge = GraspExecutionNode("ManualExecutionNode", manual_mode=True)
    rospy.loginfo("ge.robot_interface.home_arm()")
    rospy.loginfo("ge.robot_interface.hand_manager.hand_manager.open_hand()")


    #create place action goal here
    #place_goal = message_utils.build_build_place_goal()
    #ge.robot_interface.group.place(place_goal)
    import IPython
    IPython.embed()
