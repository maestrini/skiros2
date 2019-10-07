from skiros2_skill.core.skill import SkillDescription, SkillBase, Sequential
from skiros2_std_skills.action_client_primitive import PrimitiveActionClient
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes
import skiros2_common.tools.logger as log

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from moveit_linear_motion.msg import LinearMotionAction, LinearMotionGoal
from moveit_msgs.srv import GetMotionPlan, GetMotionPlanRequest
from moveit_msgs.msg import JointConstraint, Constraints
from sensor_msgs.msg import JointState

import pickle
import ast

import moveit_commander
from copy import deepcopy
from multiprocessing.dummy import Process

#################################################################################
# Descriptions
#################################################################################

class ExecuteTrajectory(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Arm", Element("rparts:ArmDevice"), ParamTypes.Required)
        self.addParam("Plan", str, ParamTypes.Required)

class PlanCartesian(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Arm", Element("rparts:ArmDevice"), ParamTypes.Required)
        self.addParam("moveit/Target", Element("sumo:Object"), ParamTypes.Required)
        self.addParam("Timeout", 5.0, ParamTypes.Required)
        self.addParam("Plan", str, ParamTypes.Optional)

#################################################################################
# Implementation
#################################################################################


class execute_trajectory(PrimitiveActionClient):
    """
    @brief Client for FollowJointTrajectory action
    """
    def createDescription(self):
        self.setDescription(ExecuteTrajectory(), self.__class__.__name__)

    def onReset(self):
        self._poselists = []
        self._completion = 0.0

    def buildClient(self):
        return actionlib.SimpleActionClient(self.params["Arm"].value.getProperty("skiros:MotionExe").value, FollowJointTrajectoryAction)

    def buildGoal(self):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = pickle.loads(self.params["Plan"].value).joint_trajectory
        for pose in goal.trajectory.points:
            self._poselists.append([round(elem,2) for elem in pose.positions])
        return goal

    def onFeedback(self, msg):
        position = [round(elem,2) for elem in msg.desired.positions]
        if position in self._poselists:
            self._completion = float(self._poselists.index(position))/float(len(self._poselists))*100.0
            return self.step("Progress: {}%".format(round(self._completion)))
        else:
            return self.step("")

    def onDone(self, status, msg):
        if ((status == GoalStatus.PREEMPTED) or (status == GoalStatus.ABORTED)):
            return self.fail("Failed", -1)
        elif status == GoalStatus.SUCCEEDED:
            return self.success("Succeeded")
        elif status == GoalStatus.REJECTED:
            rospy.loginfo(status)
            rospy.loginfo(msg)
            return self.fail("Goal was rejected by action server.", -2)
        else:
            rospy.loginfo(status)
            rospy.loginfo(msg)
            return self.fail("Unknown return code.", -100)

class plan_cartesian(PrimitiveBase):
    """
    @brief moveit planning to cartesian goal
    """
    def createDescription(self):
        self.setDescription(PlanCartesian(), self.__class__.__name__)

    def onStart(self):
        self._to = rospy.Time().now() + rospy.Duration(self.params["Timeout"].value)
        self.plan = None
        if not self.setup_request():
            return False
        self.start_planning()
        return True

    def execute(self):
        if self.plan is not None:
            return self.analyse_answer(self.plan)
        if rospy.Time.now() > self._to:
            return self.fail("'%s' execution timed out." %(self.__class__.__name__), -1)
        return self.step("")

    #--------------------------------------------------

    def thread(self, _):
        self.plan = self._group_cmdr.plan()

    def start_planning(self):
        self.process = Process(target=plan_cartesian.thread, args=(self, False))
        self.process.start()

    def setup_request(self):
        arm = self.params["Arm"].value
        if not arm.hasProperty("skiros:FrameId", not_none=True):
            raise Exception("Missing pose info for arm.")
        self._group_cmdr = moveit_commander.MoveGroupCommander(arm.getProperty("skiros:MoveItGroup").value)
        self._group_cmdr.set_planner_id("PRMkConfigDefault")
        target_pose = self.transform_to_frame(
            deepcopy(self.params["moveit/Target"].value),
            arm.getProperty("skiros:FrameId").value)
        return self.inverse_kinematic(target_pose.getData(":PoseStampedMsg").pose)

    def transform_to_frame(self, element, target_frame):
        if not element.hasProperty("skiros:FrameId", not_none=True):
            raise Exception("Missing pose info for target.")
        reasoner = element._getReasoner("AauSpatialReasoner")
        reasoner._getTransform(element.getProperty("skiros:FrameId").value, target_frame)
        reasoner._transform(element, target_frame)
        return element

    def inverse_kinematic(self, posemsg):
        try:
            self._group_cmdr.set_joint_value_target(posemsg, False)
            return True
        except:
            log.error("inverse_kinematic", "No IK found.")
            return False

    def analyse_answer(self, answer):
        self.params["Plan"].value = pickle.dumps(answer)
        return self.success("Plan succeeded")


class plan_cartesian_fast(PrimitiveActionClient):
    """
    @brief Fast motion planning to cartesian pose: first try interpolation otherwise switch to moveit planning
    """
    def createDescription(self):
        self.setDescription(PlanCartesian(), self.__class__.__name__)


    def transform_to_frame(self, element, target_frame):
        if not element.hasProperty("skiros:FrameId", not_none=True):
            raise Exception("Missing pose info for target.")
        reasoner = element._getReasoner("AauSpatialReasoner")
        reasoner._getTransform(element.getProperty("skiros:FrameId").value, target_frame)
        reasoner._transform(element, target_frame)
        return element

    def buildClient(self):
        return actionlib.SimpleActionClient("{}/linear_motion".format(self.params["Arm"].value.getProperty("skiros:MoveItGroup").value), LinearMotionAction)

    def buildGoal(self):
        goal = LinearMotionGoal()
        goal.planning_space = "tool"
        self.params["moveit/Target"].value.setProperty("skiros:TfTimeStamp", None)
        target = self.params["moveit/Target"].value
        arm_frame = self.params["Arm"].value.getProperty("skiros:LinkedToFrameId").value
        target = self.transform_to_frame(deepcopy(target), arm_frame).getData(":PoseStampedMsg")
        target.header.frame_id = self.params["Arm"].value.getProperty("skiros:LinkedToFrameId").value
        goal.pose_waypoints.append(target)
        return goal

    def onFeedback(self, msg):
        return self.step("{}".format(msg.message))

    def onDone(self, status, msg):
        if ((status == GoalStatus.PREEMPTED) or (status == GoalStatus.ABORTED)):
            return self.fail("Failed", -1)
        elif status == GoalStatus.SUCCEEDED:
            if not msg.success:
                return self.fail("Planning failed. ", -101)
            msg.linear_motion_response.trajectory.joint_trajectory.header.frame_id = self.params["Arm"].value.getProperty("skiros:LinkedToFrameId").value
            points = msg.linear_motion_response.trajectory.joint_trajectory.points
            self.params["Plan"].value = pickle.dumps(msg.linear_motion_response.trajectory)
            return self.success("Plan succeeded: {} waypoints, estimated execution time {} secs.".format(len(points), points[-1].time_from_start))
        elif status == GoalStatus.REJECTED:
            return self.fail("Goal was rejected by action server.", -2)
        else:
            return self.fail("Unknown return code.", -100)
