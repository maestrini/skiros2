from skiros2_skill.core.skill import SkillDescription, SkillBase, Serial, ParallelFf, ParallelFs, NoFail, Selector, Sequential
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

class CartesianMovement(SkillDescription):
    """
    @brief Any arm movement that brings the end-effector at the target pose
    """
    def createDescription(self):
        #=======Params=========
        self.addParam("Arm", Element("rparts:ArmDevice"), ParamTypes.Required)
        self.addParam("Target", Element("sumo:Object"), ParamTypes.Required)
        #=======PreConditions=========

class goto(SkillBase):
    """
    @brief Brings the end-effector at the target pose
    """
    def createDescription(self):
        self.setDescription(CartesianMovement(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(Sequential())
        skill(
            self.skill("PlanCartesian", "plan_cartesian_fast", remap={"moveit/Target": "Target"}),
            self.skill("ExecuteTrajectory", "execute_trajectory"),
        )
