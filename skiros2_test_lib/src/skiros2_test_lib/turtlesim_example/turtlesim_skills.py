from skiros2_skill.core.skill import SkillDescription, SkillBase, Serial, ParallelFf, ParallelFs, Selector, Sequential
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element



#################################################################################
# Move
#################################################################################


class Move(SkillDescription):
    def createDescription(self):
        self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)
        self.addParam("Linear", float, ParamTypes.Required)
        self.addParam("Angular", float, ParamTypes.Required)
        self.addParam("Duration", float, ParamTypes.Required)

class move(SkillBase):
    def createDescription(self):
        self.setDescription(Move(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(ParallelFs())
        skill(
            self.skill("Command", "command"),
            self.skill("Wait", "wait")
        )


class Go(SkillDescription):
    def createDescription(self):
        self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)
        self.addParam("Duration", 1.0, ParamTypes.Required)
        self.addParam("Distance", 1.0, ParamTypes.Required)

class go(SkillBase):
    def createDescription(self):
        self.setDescription(Go(), self.__class__.__name__)

    def expand(self, skill):
        velocity = self.params["Distance"].value / self.params["Duration"].value
        skill.setProcessor(ParallelFs())
        skill(
            self.skill("Command", "command", specify={"Linear": velocity, "Angular": 0.0}),
            self.skill("Wait", "wait", specify={"Duration": self.params["Duration"].value})
        )


class Turn(SkillDescription):
    def createDescription(self):
        self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)
        self.addParam("Duration", 1.0, ParamTypes.Required)
        self.addParam("Angle", 10.0, ParamTypes.Required)

class turn(SkillBase):
    def createDescription(self):
        self.setDescription(Turn(), self.__class__.__name__)

    def expand(self, skill):
        velocity = self.params["Angle"].value / self.params["Duration"].value
        skill.setProcessor(ParallelFs())
        skill(
            self.skill("Command", "command", specify={"Angular": velocity, "Linear": 0.0}),
            self.skill("Wait", "wait", specify={"Duration": self.params["Duration"].value})
        )





class AttractTo(SkillDescription):
    def createDescription(self):
        self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)
        self.addParam("Target", Element("sumo:Object"), ParamTypes.Required)

class attract_to(SkillBase):
    def createDescription(self):
        self.setDescription(AttractTo(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(ParallelFs())
        skill(
            self.skill("Monitor", "monitor", specify={"Turtle": self.params["Turtle"].value}),
            self.skill("Monitor", "monitor", specify={"Turtle": self.params["Target"].value}),
            self.skill("PoseController", "pose_controller"),
            self.skill("Command", "command"),
            self.skill("Wait", "wait", specify={"Duration": 10000.0})
        )




#class TurnTowards(SkillDescription):
#    def createDescription(self):
#        self.addParam("Name", str, ParamTypes.Required)
#        self.addParam("Target", str, ParamTypes.Required)
























#################################################################################
# Description
#################################################################################

class TurtleFindAndFollow(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Turtle", Element("sumo:Object"), ParamTypes.Optional)

class TurtleSpawnAndFollow(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("PosX", int, ParamTypes.Required)
        self.addParam("PosY", int, ParamTypes.Required)
        self.addParam("Rotation", int, ParamTypes.Required)
        self.addParam("Name", str, ParamTypes.Required)

class TurtleSpawnAndWander(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("PosX", int, ParamTypes.Required)
        self.addParam("PosY", int, ParamTypes.Required)
        self.addParam("Rotation", int, ParamTypes.Required)
        self.addParam("Name", "turtle", ParamTypes.Required)

#################################################################################
# Implementation
#################################################################################

class patrol_and_follow(SkillBase):
    """
    Tree is:
    ----->:trajectory_coordinator (|Fs|)
    ------->:TrajectoryGenerator
    ------->:TrajectoryConsumer

    """
    def createDescription(self):
        self.setDescription(TurtleFindAndFollow(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(Selector())
        skill.addChild(self.getNode(Serial()))
        skill.last().addChild(self.getSkill(":TurtleFind", ""))
        skill.last().addChild(self.getSkill(":TargetFollow", ""))
        skill.last().last().remap('Target', 'Turtle')
        skill.addChild(self.getSkill(":Wander", ""))

class stay_still_and_follow(SkillBase):
    """
    Tree is:
    ----->:trajectory_coordinator (|Fs|)
    ------->:TrajectoryGenerator
    ------->:TrajectoryConsumer

    """
    def createDescription(self):
        self.setDescription(TurtleFindAndFollow(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(Selector())
        skill.addChild(self.getNode(Serial()))
        skill.last().addChild(self.getSkill(":TurtleFind", ""))
        skill.last().addChild(self.getSkill(":TargetFollow", ""))
        skill.last().last().remap('Target', 'Turtle')
        skill.addChild(self.getSkill(":Wait", ""))


class turtle_spawn_and_follow(SkillBase):
    """
    """
    def createDescription(self):
        self.setDescription(TurtleSpawnAndFollow(), self.__class__.__name__)

    def expand(self, skill):
        skill.addChild(self.getNode(Sequential()))
        skill.last().addChild(self.getSkill(":TurtleSpawn", ""))
        skill.last().addChild(self.getSkill(":TurtleFind", ""))
        skill.last().addChild(self.getSkill(":TargetFollow", ""))
        skill.last().last().remap('Target', 'Turtle')


class turtle_spawn_and_wander(SkillBase):
    """
    """
    def createDescription(self):
        self.setDescription(TurtleSpawnAndWander(), self.__class__.__name__)

    def expand(self, skill):
        skill.addChild(self.getNode(Sequential()))
        skill.last().addChild(self.getSkill(":TurtleSpawn", ""))
        skill.last().addChild(self.getNode(ParallelFf()))
        skill.last().last().addChild(self.getSkill(":Wander2", ""))
        skill.last().last().addChild(self.getSkill(":TurtleFind", ""))
        skill.last().last().addChild(self.getSkill(":TargetFollow", ""))
        skill.last().last().remap('Target', 'Turtle')
