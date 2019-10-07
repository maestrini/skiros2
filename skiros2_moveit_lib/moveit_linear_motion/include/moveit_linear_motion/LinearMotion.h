#ifndef LINEARMOTION_H
#define LINEARMOTION_H

#include "ros/ros.h"
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>

#include "moveit_linear_motion/LinearMotionAction.h"
using Goal = typename moveit_linear_motion::LinearMotionAction::_action_goal_type::_goal_type;
using Result = typename moveit_linear_motion::LinearMotionAction::_action_result_type::_result_type;

class LinearMotion
{
public:

   LinearMotion(std::string group_name);

   bool generate_trajectory(const Goal::ConstPtr & req1, Result &res);

private:

   void getCurrentJointConfig();

   bool isValid(std::vector<sensor_msgs::JointState>  jointConfig,moveit_msgs::Constraints motionConstarins);

   bool getFKsolution(std::vector<sensor_msgs::JointState> jointConf,
                      std::vector<geometry_msgs::PoseStamped> &fkSolution);
   bool getIKsolution(std::vector<geometry_msgs::PoseStamped>  toolPose,
                       std::string interpolationSpace,
                       std::vector<sensor_msgs::JointState> &ikSoloution,
                      std::vector<double> startingState);

   bool interpolateJointSpace(std::vector<sensor_msgs::JointState> wayPoints,
                             moveit_msgs::Constraints motionConstrains,
                             moveit_msgs::RobotTrajectory &generatedTrajectory);
   bool interpolateToolSpace(std::vector<sensor_msgs::JointState> wayPoints,
                             moveit_msgs::Constraints motionConstrains,
                             moveit_msgs::RobotTrajectory &generatedTrajectory, bool virtual_interpolation);
   bool addVelAcc(moveit_msgs::RobotTrajectory &generatedTrajectory);
   bool useProbabilisticPlanner(moveit_msgs::Constraints motionConstrains,moveit_msgs::RobotTrajectory &generatedTrajectory);
   void getActiveJoints(std::vector<sensor_msgs::JointState>  jointConfig,std::vector<sensor_msgs::JointState>&  activeJointConfig);
   double getJointsDistance(std::vector<double>  current_joints,std::vector<double>  joints_from_ik);
   void visualizeTrajectory(moveit_msgs::RobotTrajectory &generatedTrajectory);


   planning_scene_monitor::CurrentStateMonitorPtr current_state_monitor_;


   std::shared_ptr<robot_model_loader::RobotModelLoader> _robot_model_loader;
   robot_model::RobotModelPtr _kinematic_model;
   moveit::planning_interface::MoveGroupInterface* _planningGroup;
   planning_scene::PlanningScene* _planningScene;
   moveit_msgs::MoveItErrorCodes _errorCode;
   std::vector<double> _startingState;
   trajectory_msgs::JointTrajectory _generatedTrajectory;
   std::vector<std::string> _jointNames;
   std::string _groupName;
   ros::NodeHandle _nh;
   moveit_msgs::PositionIKRequest _inverse_kin_request;
   moveit_msgs::PositionIKRequest _forward_kin_request;
   sensor_msgs::JointState _jointConfig;
   std::string _planningFrame;
   moveit_msgs::RobotState _armState;
   double _trajectory_steps_joint;
   double _trajectory_steps_tool;
   moveit_msgs::MotionPlanResponse _plan_response;
   ros::Publisher _display_publisher;
   std::shared_ptr<robot_state::RobotState> _kinematic_state;
   robot_state::JointModelGroup* _joint_model_group;
   std::vector<std::string> _model_joint_names;
   std::vector<double> _joint_model_values;
   double _speed_scale;

};
#endif
