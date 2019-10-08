from skiros2_skill.core.skill import SkillDescription, ParamOptions
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.primitive import PrimitiveBase
import rospy
import turtlesim.msg as ts
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn as SpawnSrv
import threading, Queue, numpy

import numpy as np
import math

#################################################################################
# Command
#################################################################################

class Command(SkillDescription):
    def createDescription(self):
        self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)
        self.addParam("Linear", float, ParamTypes.Required)
        self.addParam("Angular", float, ParamTypes.Required)

class command(PrimitiveBase):
    def createDescription(self):
        self.setDescription(Command(), self.__class__.__name__)

    def _send_command(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = math.radians(angular)
        self.pose_pub.publish(msg)
        return msg

    def onEnd(self):
        self._send_command(0,0)

    def onStart(self):
        turtle = self.params["Turtle"].value.getProperty("tts:TurtleName").value
        self.pose_pub = rospy.Publisher(turtle + "/cmd_vel", Twist, queue_size=20)
        return True

    def execute(self):
        self._send_command(self.params["Linear"].value, self.params["Angular"].value)
        return self.step("")


#################################################################################
# Spawn
#################################################################################

class Spawn(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Name", str, ParamTypes.Required)
        self.addParam("X", float, ParamTypes.Required)
        self.addParam("Y", float, ParamTypes.Required)
        self.addParam("Rotation", float, ParamTypes.Required)
        
        self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Optional)

class spawn(PrimitiveBase):
    def createDescription(self):
        self.setDescription(Spawn(), self.__class__.__name__)

    def execute(self):
        turtle = self.params["Turtle"].value
        name = self.params["Name"].value
        if turtle.id == "":
            try:
                spawner = rospy.ServiceProxy('spawn', SpawnSrv)
                resp = spawner(self.params["X"].value , self.params["Y"].value, math.radians(self.params["Rotation"].value), name)
            except rospy.ServiceException, e:
                return self.fail("Spawning turtle failed.", -1)

            turtle.label = name
            turtle.setProperty("tts:TurtleName", "/{}".format(name))
            turtle.setData(":Position", [self.params["X"].value, self.params["Y"].value, 0.0])
            turtle.setData(":OrientationEuler", [0.0, 0.0, math.radians(self.params["Rotation"].value)])
            turtle.addRelation("skiros:Scene-0", "skiros:contain", "-1")
            turtle = self._wmi.add_element(turtle)
            self.params["Turtle"].value = turtle

            self.success("Spawned turtle {}".format(name))
            
        return self.success("")


#################################################################################
# Monitor
#################################################################################

class Monitor(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)

class monitor(PrimitiveBase):
    def createDescription(self):
        self.setDescription(Monitor(), self.__class__.__name__)

    def onStart(self):
        name = self.params["Turtle"].value.getProperty("tts:TurtleName").value
        self._pose_sub = rospy.Subscriber(name + "/pose", ts.Pose, self._monitor)
        self._pose = None
        return True

    def _monitor(self, msg):
        self._pose = [msg.x, msg.y, msg.theta]

    def execute(self):
        if self._pose is None:
            return self.step("No pose received")

        x,y,theta = self._pose
        turtle = self.params["Turtle"].value
        turtle.setData(":Position", [x, y, 0.0])
        turtle.setData(":OrientationEuler", [0.0, 0.0, theta])
        self.params["Turtle"].value = turtle
  
        return self.step("")


#################################################################################
# Wait
#################################################################################

class Wait(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Duration", float, ParamTypes.Required)

class wait(PrimitiveBase):
    def createDescription(self):
        self.setDescription(Wait(), self.__class__.__name__)

    def onStart(self):
        self._start = rospy.Time.now()
        return True
        
    def execute(self):
        if rospy.Time.now() < self._start + rospy.Duration(self.params["Duration"].value):
            return self.step("")
        return self.success("")





class PoseController(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)
        self.addParam("Target", Element("sumo:Object"), ParamTypes.Required)
        self.addParam("Linear", float, ParamTypes.Optional)
        self.addParam("Angular", float, ParamTypes.Optional)

class pose_controller(PrimitiveBase):
    def createDescription(self):
        self.setDescription(PoseController(), self.__class__.__name__)
        
    def execute(self):
        turtle = self.params["Turtle"].value
        target = self.params["Target"].value
        
        turtle_pos = np.array(turtle.getData(":Position"))[:2]
        target_pos = np.array(target.getData(":Position"))[:2]
        vec = target_pos - turtle_pos
        distance = np.linalg.norm(vec)
        
        turtle_rot = turtle.getData(":OrientationEuler")[2]

        a = vec / distance
        b = np.array([math.cos(turtle_rot), math.sin(turtle_rot)])
        angle = math.acos(a.dot(b))
        
        self.params["Linear"].value = distance / 4.0
        self.params["Angular"].value = math.degrees(angle) / 2.0
        
        return self.step("Vel: {} / {}".format(self.params["Linear"].value, self.params["Angular"].value))






class Wander(SkillDescription):
    def createDescription(self):
        #=======Params=========
        pass

class Wander2(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Name", str, ParamTypes.Required)

class TurtleFind(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Turtle", Element("sumo:Object"), ParamTypes.Optional)

class TargetFollow(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Target", Element("sumo:Object"), ParamTypes.Required)
        self.addParam("Pgain", 1, ParamTypes.Required)

class TurtleSpawn(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("PosX", int, ParamTypes.Required)
        self.addParam("PosY", int, ParamTypes.Required)
        self.addParam("Rotation", int, ParamTypes.Required)
        self.addParam("Name", str, ParamTypes.Required)

#################################################################################
# Implementations
#################################################################################

class wander_around(PrimitiveBase):
    """
    """
    counter = 0

    def createDescription(self):
        self.setDescription(Wander(), self.__class__.__name__)

    def _sendCmd(self):
        msg = Twist()
        self.counter += 0.1
        msg.linear.x = math.sin(self.counter)
        self.pose_pub.publish(msg)
        return msg

    def onReset(self):
        self.counter = 0

    def onStart(self):
        my_turtle = self.params["Robot"].value.getProperty("tts:TurtleName").value
        self.pose_pub = rospy.Publisher(my_turtle+"/cmd_vel", Twist, queue_size=20)
        return True

    def execute(self):
        self._sendCmd()
        return self.step("")


class wander_around_2(PrimitiveBase):
    """
    Same as wander_around except for the name of the topic that is published (it depends on the "Name" parameters and not on the "Robot" parameter)
    """
    counter = 0

    def createDescription(self):
        self.setDescription(Wander2(), self.__class__.__name__)

    def _sendCmd(self):
        msg = Twist()
        self.counter += 0.1
        msg.linear.x = math.sin(self.counter)
        self.pose_pub.publish(msg)
        return msg

    def onReset(self):
        self.counter = 0

    def onStart(self):
        self.pose_pub = rospy.Publisher(self.params["Name"].getValue()+"/cmd_vel", Twist, queue_size=20)
        return True

    def execute(self):
        self._sendCmd()
        return self.step("")


class turtle_find(PrimitiveBase):
    """
    """
    def createDescription(self):
        self.setDescription(TurtleFind(), self.__class__.__name__)


    def execute(self):
        my_turtle = self.params["Robot"].value.getProperty("tts:TurtleName").value
        other_turtle = self.params["Turtle"].value
        #Retrieve
        tlist = [topic.replace("/pose", "") for topic, type in rospy.get_published_topics() if type=='turtlesim/Pose' and topic.find(my_turtle)==-1]
        if tlist:
            other_turtle.setProperty("tts:TurtleName", tlist[0])
            self.params["Turtle"].value = other_turtle
            print self.params["Turtle"].value.printState(True)
            return self.success("Detected turtle {}".format(tlist[0]))
        #return self.step("")
        return self.fail("", -1)

class target_follow(PrimitiveBase):
    """
    """
    target_pose = numpy.array([0,0,0])
    self_pose = numpy.array([0,0,0])
    pose_sub = None
    pose_pub = None
    p_gain = 0
    counter = 0
    go_forward = False


    def createDescription(self):
        self.setDescription(TargetFollow(), self.__class__.__name__)

    def _otherPoseMonitor(self, msg):
        self.target_pose = numpy.array([msg.x, msg.y, msg.theta])

    def _selfPoseMonitor(self, msg):
        self.self_pose = numpy.array([msg.x, msg.y, msg.theta])

    def _sendCmd(self):
        msg = Twist()
        diff = numpy.array([(self.target_pose[0]-self.self_pose[0]), (self.target_pose[1]-self.self_pose[1])])
        tan = numpy.arctan2(*diff)-numpy.pi/2
        if tan < 0:
            tan = (2*numpy.pi + tan)
        norm = numpy.linalg.norm(diff)
        theta = abs(self.self_pose[2])
        if abs(theta-tan)>0.02 and not self.go_forward:
            msg.angular.z = self.p_gain*(theta-tan)
        else:
            self.go_forward = True
            msg.linear.x = self.p_gain*norm
        #print "{} {}".format(tan, theta)
        self.pose_pub.publish(msg)
        return msg

    def onReset(self):
        self.go_forward = False
        self.target_pose = numpy.array([0,0,0])
        self.self_pose = numpy.array([0,0,0.05])

    def onStart(self):
        my_turtle = self.params["Robot"].value.getProperty("tts:TurtleName").value
        other_turtle = self.params["Target"].value.getProperty("tts:TurtleName").value
        self.opose_sub = rospy.Subscriber(other_turtle+"/pose", ts.Pose, self._otherPoseMonitor)
        self.mpose_sub = rospy.Subscriber(my_turtle+"/pose", ts.Pose, self._selfPoseMonitor)
        self.pose_pub = rospy.Publisher(my_turtle+"/cmd_vel", Twist, queue_size=20)
        return True

    def execute(self):
        self.p_gain = self.params["Pgain"].getValue()

        cmd = self._sendCmd()

        if abs(cmd.linear.x)>0.5 or abs(cmd.angular.z)>0:
            self.counter = 0
            return self.step("Speed: {} {}".format(cmd.linear.x, cmd.angular.z))
        else:
            self.counter += 1
            if self.counter>20:
                return self.success("")
        return self.step("")


class turtle_spawn(PrimitiveBase):
    """
    """
    def createDescription(self):
        self.setDescription(TurtleSpawn(), self.__class__.__name__)

    def onStart(self):
        rospy.wait_for_service('spawn')
        try:
            turtle_spawner = rospy.ServiceProxy('spawn', Spawn)
            resp = turtle_spawner(self.params["PosX"].getValue() , self.params["PosY"].getValue() , self.params["Rotation"].getValue() ,  self.params["Name"].getValue())
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return True

    def execute(self):
        return self.success("turtle spawned")
