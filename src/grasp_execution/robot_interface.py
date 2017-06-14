import rospy
import control_msgs.msg
import moveit_msgs.msg
#import jaco_msgs.srv
import numpy as np
import math
import tf


class RobotInterface:

    def __init__(self,
                 trajectory_action_client,
                 display_trajectory_publisher,
                 hand_manager,
                 group,
                 grasp_reachability_analyzer):

        self.trajectory_action_client = trajectory_action_client
        self.display_trajectory_publisher = display_trajectory_publisher
        self.hand_manager = hand_manager
        self.group = group
        self.grasp_reachability_analyzer = grasp_reachability_analyzer

    def run_trajectory(self, trajectory_msg):
        """
        :type trajectory_msg: control_msgs.msg.FollowJointTrajectoryResult
        """

        if not self.trajectory_action_client.wait_for_server(rospy.Duration(5)):
            rospy.logerr('Failed to find trajectory server')
            return False, 'Failed to find trajectory server', []

        trajectory_action_goal = control_msgs.msg.FollowJointTrajectoryGoal()
        trajectory_action_goal.trajectory = trajectory_msg
        self.trajectory_action_client.send_goal(trajectory_action_goal)

        if not self.trajectory_action_client.wait_for_result():
            rospy.logerr("Failed to execute trajectory")
            return False, "Failed to execute trajectory", []

        trajectory_result = self.trajectory_action_client.get_result()

        success = (trajectory_result.SUCCESSFUL == trajectory_result.error_code)
        if not success:
            return False, "Failed to follow trajectory", trajectory_result

        return success, None, trajectory_result

    def run_pickup_trajectory(self, pickup_result, pickup_phase):
        """
        :type pickup_phase: int
        :type pickup_result: moveit_msgs.msg.PickupResult
        """

        robot_trajectory_msg = pickup_result.trajectory_stages[pickup_phase]
        trajectory_msg = robot_trajectory_msg.joint_trajectory

        success, error_msg, trajectory_result = self.run_trajectory(trajectory_msg)
        return success, error_msg, trajectory_result

    def display_trajectory(self, trajectory_msg):
        """
        :type trajectory_msg: moveit_msgs.msg.trajectory_msg
        """
        display_msg = moveit_msgs.msg.DisplayTrajectory()
        display_msg.trajectory = trajectory_msg
        display_msg.model_id = self.group.get_name()
        self.display_trajectory_publisher.publish(display_msg)

    def generate_pick_plan(self, grasp_msg):
        """
        :type grasp_msg: graspit_msgs.msg.Grasp
        """
        success = False
        pick_plan_result = None

        for i in xrange(10):
            success, pick_plan_result = self.grasp_reachability_analyzer.query_moveit_for_reachability(grasp_msg)
            if success:
                break
            else:
                rospy.loginfo("Planning attempt %i Failed" % i)

        return success, pick_plan_result

    def drop_off(self, execute=True):
        self.grasp_reachability_analyzer.drop_off()
        return

    def home_arm(self, execute=True):

        def get_plan():
            success = True
            self.group.set_planning_time(rospy.get_param('~allowed_planning_time', 10))
            self.group.set_start_state_to_current_state()
            self.group.set_named_target("home")
            plan = self.group.plan()

            if len(plan.joint_trajectory.points) == 0:
                rospy.logerr("Failed to plan path home.")
                success = False

            return success, plan

        def is_already_at_goal(plan):
            current_joint_values = self.group.get_current_joint_values()
            final_joint_values = plan.joint_trajectory.points[-1].positions

            rospy.loginfo("current_joint_values: " + str(current_joint_values))
            rospy.loginfo("final_joint_values: " + str(final_joint_values))

            already_home = True
            threshold = 0.05
            for i, (current, final) in enumerate(zip(current_joint_values, final_joint_values)):
                difference = abs(current-final) % math.pi
                rospy.loginfo("joint_value " + str(i) + " difference: " + str(difference))
                if difference > threshold:
                    already_home = False
                    break

            return already_home

        def is_near_goal(plan):
            current_joint_values = self.group.get_current_joint_values()
            final_joint_values = plan.joint_trajectory.points[-1].positions

            rospy.loginfo("current_joint_values: " + str(current_joint_values))
            rospy.loginfo("final_joint_values: " + str(final_joint_values))

            near_home = True
            threshold = 0.5 
            for i, (current, final) in enumerate(zip(current_joint_values, final_joint_values)):
                if i > 4:
                    break
                difference = abs(current-final) % (math.pi)
                if i == 3:
                    difference2 = abs(difference - (math.pi))
                    if difference2 < difference:
                        difference = difference2
                rospy.loginfo("joint_value " + str(i) + " difference: " + str(difference))
                if difference > threshold:
                    near_home = False
                    break

            return near_home            

        success, plan = get_plan()
        if not execute:
            return success

        if not success:
            return success
        else:
            if is_already_at_goal(plan):
                rospy.loginfo("Arm is already home, no need to home it.")
            elif is_near_goal(plan):
                rospy.loginfo("Arm is close to home, no need to plan path.")
                #gohome = rospy.ServiceProxy('/mico_arm_driver/in/home_arm', jaco_msgs.srv.HomeArm)
                #gohome()
            else:
                rospy.loginfo("Arm is not home, so homing it")
                success = self.group.execute(plan)
                rospy.loginfo("Arm homed with success: " + str(success))

        return success

   