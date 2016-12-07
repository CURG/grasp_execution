#!/usr/bin/env python

import sys
import importlib
import copy

import rospy
import actionlib
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.msg
import control_msgs.msg

import graspit_msgs.msg
import graspit_msgs.srv

import execution_stages
import robot_interface
import execution_pipeline

from hand_managers import hand_manager
from reachability_analyzer.grasp_reachability_analyzer import GraspReachabilityAnalyzer

from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes


class GraspExecutionNode():

    def __init__(self, node_name='grasp_execution_node', manual_mode=False):

        rospy.init_node(node_name)
        #added by Bo
        self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True)
        self.scene = PlanningSceneInterface("base_link")

        self.grasp_approach_tran_frame = rospy.get_param('/grasp_approach_tran_frame')
        self.trajectory_display_topic = rospy.get_param('trajectory_display_topic')
        self.grasp_listener_topic = rospy.get_param('grasp_listener_topic')
        self.move_group_name = rospy.get_param('/move_group_name')
        self.reachability_planner_id = self.move_group_name + rospy.get_param('grasp_executer/planner_config_name')

        display_trajectory_publisher = rospy.Publisher(self.trajectory_display_topic, moveit_msgs.msg.DisplayTrajectory)

        self.trajectory_action_client = actionlib.SimpleActionClient(rospy.get_param('trajectory_action_name'), control_msgs.msg.FollowJointTrajectoryAction)
        self.hand_manager = hand_manager.GraspManager(importlib.import_module(rospy.get_param('hand_manager')), self.move_group_name)

        moveit_commander.roscpp_initialize(sys.argv)
        group = moveit_commander.MoveGroupCommander(self.move_group_name)

        planner_id = self.reachability_planner_id
        grasp_reachability_analyzer = GraspReachabilityAnalyzer(group, self.grasp_approach_tran_frame, planner_id)

        self.robot_interface = robot_interface.RobotInterface(trajectory_action_client=self.trajectory_action_client,
                                                               display_trajectory_publisher=display_trajectory_publisher,
                                                               hand_manager=self.hand_manager,
                                                               group=group,
                                                               grasp_reachability_analyzer=grasp_reachability_analyzer)

        self.execution_pipeline = execution_pipeline.GraspExecutionPipeline(self.robot_interface,
                                                                            ['MoveToPreGraspPosition',
                                                                             'PreshapeHand', 'Approach',
                                                                             'CloseHand', 'Lift'])
        self.pre_planning_pipeline = execution_pipeline.GraspExecutionPipeline(self.robot_interface,
                                                                            ['HomeArm', 'OpenHand'])

        self.last_grasp_time = 0

        if not manual_mode:
            self._grasp_execution = actionlib.SimpleActionServer("grasp_execution_action",
                                                                 graspit_msgs.msg.GraspExecutionAction,
                                                                 execute_cb=self._grasp_execution_cb,
                                                                 auto_start=False)
            self._grasp_execution.start()

        rospy.loginfo(self.__class__.__name__ + " is initialized")



    #added by Bo
    #Detach previous attached objects
    def update_scene(self):
        # self.scene.removeCollisionObject('all', True)
        # self.scene.removeAttachedObject('all', True)
        rospy.loginfo("updateScene: detach object")



    #added by Bo
    """
    need to change the parameter!!!
    """
    def pick(self, grasp_goal, pick_plan):
        # import IPython
        # IPython.embed()
       
        try: 
            # moveit_grasps = [pick_plan.grasp]
            self.update_scene()
            
            grasp = pick_plan.grasp

            grasp_point = grasp.grasp_posture.points[0]
            grasp_point.effort = [50.0, 50.0]
            grasp_point.positions = [0.0, 0.0]
            grasp_point.time_from_start.secs = 0
            grasp_point.time_from_start.secs = 2
            grasp.grasp_posture.points[0] = grasp_point
            grasp.grasp_pose.frame_id = base_link
            moveit_grasps = [grasp]

            # import IPython
            # IPython.embed()
            success, pick_result = self.pickplace.pick_with_retry(grasp_goal.grasp.object_name,
                                                              moveit_grasps,
                                                              support_name='table'
                                                              scene = self.scene)

            self.pick_result = pick_result
        except:
            import IPython
            IPython.embed()
        return success

    def place(self, grasp_goal, pose_stamped):
        places = list()
        l = PlaceLocation()
        l.place_pose.pose = pose_stamped.pose
        l.place_pose.header.frame_id = pose_stamped.header.frame_id

        """
        can change the place location based on the pick location. Let the pick location be the origin.
        """
        # copy the posture, approach and retreat from the grasp used
        l.post_place_posture = self.pick_result.grasp.pre_grasp_posture
        l.pre_place_approach = self.pick_result.grasp.pre_grasp_approach
        l.post_place_retreat = self.pick_result.grasp.post_grasp_retreat
        places.append(copy.deepcopy(l))
        # create another several places, rotate each by 360/m degrees in yaw direction
        m = 16 # number of possible place poses
        pi = 3.141592653589
        for i in range(0, m-1):
            l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 0, 0, 2 * pi / m)
            places.append(copy.deepcopy(l))

        success, place_result = self.pickplace.place_with_retry(grasp_goal.grasp.object_name,
                                                                places,
                                                                scene=self.scene)
        return success

    def _grasp_execution_cb(self, grasp_goal):
        
        rospy.loginfo("GraspExecutor::process_grasp_msg::" + str(grasp_goal))

        status = graspit_msgs.msg.GraspStatus.SUCCESS
        status_msg = "grasp_succeeded"
        success = True
        # import IPython
        # IPython.embed()
        #Pre Planning Moves, normally home arm and open hand
        if success:
            success, status_msg = self.pre_planning_pipeline.run(grasp_goal.grasp, None, self._grasp_execution)

        #Generate Pick Plan
        if success:
            success, pick_plan = self.robot_interface.generate_pick_plan(grasp_goal.grasp)
            if not success:
                grasp_status_msg = "MoveIt Failed to plan pick"
                status = graspit_msgs.msg.GraspStatus.ROBOTERROR
                rospy.logerr(grasp_status_msg)

        #Execute Plan on actual robot
        if success:
            #at some point bring this in instead, but talk to jake first.
            # self.robot_interface.group.pick(grasp_goal.object_name, [pick_plan.grasp])

            # success, status_msg = self.execution_pipeline.run(grasp_goal.grasp, pick_plan, self._grasp_execution)
            #added by Bo
            """
            need to change the parameteres
            """
          
            success = self.pick(grasp_goal, pick_plan)
        
        if success:
            """
            need to change the parameters
            """
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.z += 0.05
            pose_stamped.header.frame_id = pick_plan.grasp.grasp_pose.header.frame_id
            success = self.place(grasp_goal, pose_stamped)
        #need to return [] for empty response.
        _result = graspit_msgs.msg.GraspExecutionResult()
        _result.success = success
        self._grasp_execution.set_succeeded(_result)
        return []


    

if __name__ == '__main__':

    try:
        ge = GraspExecutionNode()
        loop = rospy.Rate(10)
        while not rospy.is_shutdown():
            loop.sleep()

    except rospy.ROSInterruptException:
        rospy.signal_shutdown()
