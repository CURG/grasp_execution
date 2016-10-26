__author__ = 'jvarley'
import roslib
import rospy
import actionlib

from control_msgs.msg import GripperCommand, GripperCommandAction, GripperCommandGoal

#range 0 to 0.1 gripper width in centimeters
def move_hand(positions, blocking=True):
    client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)

    if len(positions) != 1:
        return False, "Wrong joint number", positions

    if not client.wait_for_server():
        success = False
        reason = 'failed to connect to action server'
        return success, reason, positions
    rospy.loginfo("Connected to Finger server")

    client.send_goal(GripperCommandGoal(GripperCommand(position=positions[0], max_effort=100)))

    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        rospy.loginfo("Program interrupted, pre-empting goal")
        client.cancel_all_goals()
        success = False
        reason = 'Program interrupted from keyboard'
        return success, reason, positions
    result = client.get_result()
    positions = [result.position]
    success = True
    reason = None
    return success, reason, positions


def close_hand():
    fingers = [0]
    success, reason, position = move_hand(fingers)
    return success, reason, position


def open_hand():
    fingers = [0.087] # slightly less than actual max of 0.0875
    success, reason, position = move_hand(fingers)
    return success, reason, position

def get_gripper_width():
    msg = rospy.wait_for_message(rospy.get_param('gripper_controller_name', 'l_gripper_controller') + '/state', JointControllerState)
    return [msg.process_value]

