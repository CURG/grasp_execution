# coding: utf-8

import reachability_analyzer.message_utils as mu

place_location = mu.build_place_location()
object_name = "all"
planning_group = ge.robot_interface.group
reload(mu)
place_goal = mu.build_place_goal(place_location, object_name, planning_group)
import moveit_msgs.msg
pg = moveit_msgs.msg.PlaceGoal()
reload(mu)
place_goal = mu.build_place_goal(place_location, object_name, planning_group)






ge.robot_interface.group.place(place_goal)
ge.robot_interface.group.place("arm", place_goal)
ge.robot_interface.group.place(place_goal)
ge.robot_interface.group.place("all", )
place_goal = mu.build_place_goal(place_location, object_name, planning_group)
ge.robot_interface.group.place("all", place_location)
ge.robot_interface.group.place("all", place_location.place_pose)
ge.robot_interface.group.place("arm", place_location.place_pose)
place_location
place_plan_client = actionlib.SimpleActionClient('/place', moveit_msgs.msg.PlaceAction)
import actionlib
place_plan_client = actionlib.SimpleActionClient('/place', moveit_msgs.msg.PlaceAction)
place_plan_client.send_goal(place_goal)
ge.robot_interface.group.attach_object("all")
place_plan_client.send_goal(place_goal)
place_goal
place_goal.place_locations[0].post_place_posture.header.frame_id="/world"
place_goal.place_locations[0].post_place_posture.header.frame_id="/world"
place_goal.place_locations[0].post_place_posture.header.frame_id="/world"
place_goal
place_goal.place_locations[0].place_pose.header.frame_id="/world"
place_plan_client.send_goal(place_goal)
place_goal
place_goal.place_locations[0].place_pose.pose.position.z = .3
place_goal.place_locations[0].place_pose.pose.orientation.w=1
place_plan_client.send_goal(place_goal)
place_goal.place_locations[0].pre_place_approach.frame_id="/world"
place_goal.place_locations[0].pre_place_approach.header.frame_id="/world"
place_goal.place_locations[0].pre_place_approach.header.frame_id="/world"
place_goal.place_locations[0].pre_place_approach.header.frame_id="/world"
place_goal.place_locations[0].pre_place_approach.direction.header.frame_id = "/world"
place_goal
place_plan_client.send_goal(place_goal)
ge.robot_interface.group.place("arm", place_location)
ge.robot_interface.group.place("all", place_location)
place_location
ge.robot_interface.group.get_joints()
ge.robot_interface.group.place("all", place_location)
place_location.place_pose.pose
place_location.place_pose.pose.position.z = 0.1
ge.robot_interface.group.place("all", place_location)
place_location.place_pose.pose.position.x = -.4
ge.robot_interface.group.place("all", place_location)
place_location
place_location.post_place_posture
place_location.post_place_posture.joint_names =  ['l_gripper_finger_joint', 'r_gripper_finger_joint']
place_location.post_place_posture
place_location.post_place_retreat
place_location
ge.robot_interface.group.place("all", place_location)
ge.robot_interface.group.place("all", place_location)
get_ipython().system(u'ls -F --color ')
get_ipython().magic(u'save pick_place_session.py 0-70')
