#include <ros/ros.h>
#include "moveit_linear_motion/ActionServer.h"
#include "moveit_linear_motion/LinearMotionAction.h"
#include "moveit_linear_motion/LinearMotion.h"


using LinearMotionAction = moveit_linear_motion::LinearMotionAction;

class LinearMotionServer : public ActionServer<LinearMotionAction>
{
public:
  LinearMotionServer(std::string move_group) : ActionServer<LinearMotionAction>(move_group + "/linear_motion")
  {
    linear_motion.reset(new LinearMotion(move_group));
  }

  STATUS execute(const typename Goal::ConstPtr & goal) override
  {
    sendFeedback("Goal received.");
    Result result;
    linear_motion->generate_trajectory(goal, result);
    return success(result);
  }

private:
  std::shared_ptr<LinearMotion> linear_motion;
};

int main (int argc, char** argv)
{
  ros::init(argc, argv, "linear_motion_node");
  ros::NodeHandle pn("~");
  std::string move_group = "arm";
  pn.getParam("move_group", move_group);
  LinearMotionServer server(move_group);
  server.init();
  ros::AsyncSpinner spinner(1); //Necessary hack for moveit getModel to work.
  spinner.start();
  ros::spin();
  return 0;
}
