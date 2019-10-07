#include "moveit_linear_motion/LinearMotion.h"

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include "moveit_msgs/CollisionObject.h"
#include "geometry_msgs/Pose.h"
#include <ros/package.h>
#include <moveit/common_planning_interface_objects/common_objects.h>


using namespace moveit;

LinearMotion::LinearMotion(std::string group_name)
{
     _robot_model_loader.reset(new robot_model_loader::RobotModelLoader("robot_description"));
     _kinematic_model = _robot_model_loader->getModel();
     _planningGroup=new moveit::planning_interface::MoveGroupInterface (group_name);

     _kinematic_state.reset(new robot_state::RobotState(_kinematic_model));

     current_state_monitor_ = moveit::planning_interface::getSharedStateMonitor(_kinematic_model, moveit::planning_interface::getSharedTF());
     current_state_monitor_->startStateMonitor();


     _joint_model_group = _kinematic_model->getJointModelGroup(group_name);
     _model_joint_names = _joint_model_group->getJointModelNames();
     _kinematic_state->copyJointGroupPositions(_joint_model_group, _joint_model_values);

     // Initialize planning scene

    _planningScene = new planning_scene::PlanningScene(_kinematic_model);
    _jointNames    = _planningGroup->getActiveJoints();
    _groupName     = group_name;


    // Initialize the  kinematics msg
    _inverse_kin_request.group_name       = _groupName;
    _inverse_kin_request.avoid_collisions = true;
    _inverse_kin_request.timeout  =ros::Duration(0.1);
    _inverse_kin_request.attempts =20;


    //Get planning frame
    _planningFrame=_planningGroup->getPlanningFrame();
    ROS_INFO_STREAM("Planning frame " << _planningFrame);

    //Set the interpolation step
    _trajectory_steps_joint=20;
     _trajectory_steps_tool=5;

    //Initialize trajectory msg

    _generatedTrajectory.joint_names=_jointNames;
    _generatedTrajectory.header.frame_id=_planningFrame;

    // Trajectory speed scale
    _speed_scale=0.001; // Large value results to slow move, small value to large move


    //Initialize joint state msg
    _jointConfig.name = _jointNames;
    _jointConfig.header.frame_id =_planningFrame;


     _armState.joint_state=_jointConfig;

    //Initialize response msgs
    _plan_response.group_name=_groupName;
    _plan_response.trajectory_start.joint_state=_jointConfig;

    _display_publisher = _nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 5, true);
}

void LinearMotion::getCurrentJointConfig(){

    _startingState=_planningGroup->getCurrentJointValues();
    ROS_INFO("[LinearMotion] Got Current Joint Config");

 }

bool LinearMotion::isValid(std::vector<sensor_msgs::JointState>  jointConfig, moveit_msgs::Constraints motionConstarins){

    sensor_msgs::JointState evaluatedJointConfig;
    evaluatedJointConfig=_jointConfig;

    bool collision_free;
    for (int i=0; i<jointConfig.size();i++){

     evaluatedJointConfig.position = jointConfig[i].position;
     _armState.joint_state=evaluatedJointConfig;

     collision_free=_planningScene->isStateValid(_armState,motionConstarins,_groupName);
     }
   return collision_free;

  }
double LinearMotion::getJointsDistance(std::vector<double>  current_joints,std::vector<double>  joints_from_ik){
    double sum=0;
    for(int i=0;i<current_joints.size();i++)
    {
    sum+=std::pow(current_joints[i]-joints_from_ik[i],2);
    }
    return std::sqrt(sum);
}
bool LinearMotion::getIKsolution(std::vector<geometry_msgs::PoseStamped> toolPose,
                                 std::string interpolationSpace,
                                 std::vector<sensor_msgs::JointState> &ikSolution,
                                 std::vector<double> startingState){

     ROS_INFO("[LinearMotion] Getting IK");
     Eigen::Affine3d end_effector_state;
     ikSolution.resize(toolPose.size());
     //std::vector< double > consistency_limits(_jointNames.size(),3);
    /* if (useConsistencyLimits){
        std::fill (consistency_limits.begin(),consistency_limits.begin()+4,0.5);
        }
     else{
         std::fill (consistency_limits.begin(),consistency_limits.begin()+1,2);
         consistency_limits[2]=1.5;
         consistency_limits[3]=0.5;
         consistency_limits[4]=1;
         }*/

     for(int i=0;i<toolPose.size();i++)
     {
        std::vector<sensor_msgs::JointState> temp_ikSolution(3);
        ikSolution[i].header.frame_id=_planningFrame;

        tf::poseMsgToEigen (toolPose[i].pose, end_effector_state);
        double distance,min_distance=20;
        int  ik_attempts=0;
        /*robot_state::JointModelGroup* temp_model_group=_joint_model_group;
        robot_state::RobotState* temp_kinematic_state=_kinematic_state;*/
        _kinematic_state->setJointGroupPositions(_joint_model_group, startingState);

        while ( ik_attempts<3)
        {
             // _kinematic_state->setToIKSolverFrame(end_effector_state,"base_link_robot");
            bool found_ik = _kinematic_state->setFromIK(_joint_model_group, end_effector_state,_planningGroup->getEndEffectorLink(),10,0.1);
            if(found_ik )
            {

                _kinematic_state->copyJointGroupPositions(_joint_model_group, temp_ikSolution[ik_attempts].position);

                distance=getJointsDistance(startingState,temp_ikSolution[ik_attempts].position);



                if(distance<min_distance)
                {
                    min_distance=distance;
                    ikSolution[i].position=temp_ikSolution[ik_attempts].position;

                    if (interpolationSpace=="tool") _kinematic_state->setJointGroupPositions(_joint_model_group,  temp_ikSolution[ik_attempts].position); return true;
                    _kinematic_state->copyJointGroupPositions(_joint_model_group,ikSolution[i].position);
                    _kinematic_state->setToRandomPositionsNearBy(_joint_model_group,*_kinematic_state,2);
                    ik_attempts++;
                    // ROS_INFO("[LinearMotion] An IK solution was found.");
                }
                else{
                ik_attempts++;
                   _kinematic_state->setToRandomPositionsNearBy(_joint_model_group,*_kinematic_state,6);
                }

                /*ikSolution[i].position=temp_ikSolution[ik_attempts].position;*/

            }
            else{
                ROS_WARN("[LinearMotion] Could not find an IK solution, repeating ...");

                ik_attempts++;
                 _kinematic_state->setToRandomPositionsNearBy(_joint_model_group,*_kinematic_state,6);

            }
    }
    if (!ikSolution[i].position.empty()) return true;
    else return false;

  }
}

bool LinearMotion::getFKsolution(std::vector<sensor_msgs::JointState> jointConf,
                                 std::vector<geometry_msgs::PoseStamped> &fkSolution  ){


    ROS_INFO("[LinearMotion] Getting FK");
    fkSolution.resize(jointConf.size());

    for (int i=0;i<jointConf.size();i++)
    {

        std::vector<double> group_position;

        group_position.resize(_model_joint_names.size(),0);

       _kinematic_state->setJointGroupPositions(_joint_model_group, jointConf[i].position);

       const Eigen::Affine3d &end_effector_state = _kinematic_state->getGlobalLinkTransform (_planningGroup->getEndEffectorLink());

       tf::poseEigenToMsg (end_effector_state, fkSolution[i].pose);

       fkSolution[i].header.frame_id=_planningFrame;
    }

    return true;
}


bool LinearMotion::interpolateToolSpace(std::vector<sensor_msgs::JointState> wayPoints,
                                        moveit_msgs::Constraints motionConstrains,
                                        moveit_msgs::RobotTrajectory &generatedTrajectory,bool virtual_interpolation){

    if (virtual_interpolation==true)
    {
        _trajectory_steps_tool=30;

    }
    else
    {
         _trajectory_steps_tool=5;
    }


    //Initialize starting joint state
    std::vector<sensor_msgs::JointState> tempJointSTate(1);
    tempJointSTate[0].position=_startingState;

    //get the FK solutuion for the target joint config
    std::vector<geometry_msgs::PoseStamped> posePath,interpolatedPose;
    posePath.resize(wayPoints.size());
    interpolatedPose.resize(1);

    getFKsolution(tempJointSTate, interpolatedPose);

    //Initialize the trajectory points
    std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points(wayPoints.size()*_trajectory_steps_tool+1);
    trajectory_points[0].positions.resize(_jointNames.size());
    trajectory_points[0].positions=_startingState;



    //Initialize the IK solutions that will derive for each pose
    std::vector<sensor_msgs::JointState> ikSolution (1);

    geometry_msgs::PoseStamped startingPose,tempPose;
    startingPose=interpolatedPose[0];


    tf::Quaternion startQuart(interpolatedPose[0].pose.orientation.x,interpolatedPose[0].pose.orientation.y,interpolatedPose[0].pose.orientation.z,interpolatedPose[0].pose.orientation.w);
    tf::Quaternion tempQuart=startQuart;

    //get the FK solutuion for the target joint config
    getFKsolution(wayPoints,posePath);




    for(int i=0;i<wayPoints.size();i++)
    {
        std::vector<double> stepSize(3);

         //Find interpolation step size for position
         double xPointStep,yPointStep,zPointStep;
         xPointStep=(posePath[i].pose.position.x-startingPose.pose.position.x)/_trajectory_steps_tool;
         yPointStep=(posePath[i].pose.position.y-startingPose.pose.position.y)/_trajectory_steps_tool;
         zPointStep=(posePath[i].pose.position.z-startingPose.pose.position.z)/_trajectory_steps_tool;

         stepSize[0]=xPointStep;
         stepSize[1]=yPointStep;
         stepSize[2]=zPointStep;

         double counter=1;
         tempPose=startingPose;


         tf::Quaternion targetQuart(posePath[i].pose.orientation.x,posePath[i].pose.orientation.y,posePath[i].pose.orientation.z,posePath[i].pose.orientation.w);



         while(counter<(wayPoints.size()*_trajectory_steps_tool)+1)
         {
          //Interpolating position
          trajectory_points[counter].positions.resize(_jointNames.size());
          tempPose.pose.position.x+=xPointStep;
          tempPose.pose.position.y+=yPointStep;
          tempPose.pose.position.z+=zPointStep;

          //Interpolating orientation
          tempQuart=tempQuart.slerp(targetQuart,counter/_trajectory_steps_tool);

          tf::quaternionTFToMsg (tempQuart, tempPose.pose.orientation);

          interpolatedPose[0]=tempPose;

          //Get IK solution of interpolated pose
          bool success =  getIKsolution(interpolatedPose, "tool", ikSolution, tempJointSTate[0].position);
           if (success)
           {


           trajectory_points[counter].positions=ikSolution[i].position;
            tempJointSTate[0].position=ikSolution[i].position;
           }
           else
           {
            return false;

           }

         counter++;
         }

     //trajectory_points[wayPoints.size()*_trajectory_steps_tool].positions=wayPoints[0].position;

    }
    _generatedTrajectory.points=trajectory_points;
    generatedTrajectory.joint_trajectory=_generatedTrajectory;

    visualizeTrajectory(generatedTrajectory);
    //Check if the found trajectory is valid (fisible,collision free, satisfies constrains)
    bool isTrajectoryValid=_planningScene->isPathValid(_armState,generatedTrajectory,motionConstrains,_groupName);


    return isTrajectoryValid;
}

bool LinearMotion::interpolateJointSpace(std::vector<sensor_msgs::JointState> wayPoints,
                                        moveit_msgs::Constraints motionConstrains,
                                        moveit_msgs::RobotTrajectory &generatedTrajectory)
{
    //Initialize trajectory points
    std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points(wayPoints.size()*_trajectory_steps_joint+1);
    trajectory_points[0].positions=_startingState;

    bool isTrajectoryValid;



    std::vector<double> initialPoint=_startingState;

    for(int i=0;i<wayPoints.size();i++)
    {


     std::vector<double> stepSize;
        //For every joint
        for (int j=0;j<wayPoints[i].position.size();j++)
        {
           //Calculate the interpolation step
           stepSize.push_back((wayPoints[i].position[j]-initialPoint[j])/ _trajectory_steps_joint);
           int counter=1;
           while(counter< _trajectory_steps_joint)
           {

            trajectory_points[counter].positions.resize(_startingState.size());
            trajectory_points[counter].positions[j]=trajectory_points[counter-1].positions[j]+stepSize[j];

           counter++;
           }

       }
         trajectory_points[wayPoints.size()* _trajectory_steps_joint].positions=wayPoints[i].position;
       initialPoint=wayPoints[i].position;
       stepSize.clear();

   }


   _generatedTrajectory.points=trajectory_points;
   generatedTrajectory.joint_trajectory=_generatedTrajectory;

   visualizeTrajectory(generatedTrajectory);
   //Check if the found trajectory is valid (fisible,collision free, satisfies constrains)
   isTrajectoryValid=_planningScene->isPathValid(_armState,generatedTrajectory,motionConstrains,_groupName);


   return isTrajectoryValid;

}

bool LinearMotion::addVelAcc(moveit_msgs::RobotTrajectory &generatedTrajectory){

   //Add velocity and acceleration to the interpolated traj
     _planningGroup->startStateMonitor();
   robot_trajectory::RobotTrajectory rt(_planningGroup->getCurrentState()->getRobotModel(),_groupName);
   trajectory_processing::IterativeParabolicTimeParameterization iptp;

   rt.setRobotTrajectoryMsg(*_planningGroup->getCurrentState(), generatedTrajectory);
   bool success = iptp.computeTimeStamps(rt);
   rt.getRobotTrajectoryMsg(generatedTrajectory);

   return success;

}

bool LinearMotion::useProbabilisticPlanner( moveit_msgs::Constraints motionConstrains,moveit_msgs::RobotTrajectory &generatedTrajectory)
{

    int errorCode;
    moveit::planning_interface::MoveGroupInterface::Plan created_plan;
      _planningGroup->startStateMonitor();
    _planningGroup->setPlannerId("PRMkConfigDefault");
    _planningGroup->setStartState(_armState);
    _planningGroup->setPathConstraints(motionConstrains);

    _errorCode=_planningGroup->plan(created_plan);
    if (_errorCode.val==1){
    generatedTrajectory=created_plan.trajectory_;
    return true;
    }
    else if(_errorCode.val==0){

        ROS_ERROR("MoveIt probabily crashed. Check if pose violates constraints"  );
        return false;}
    else{
        ROS_WARN("Planning failed with error code: %i",_errorCode.val);
        return false;
    }
}

void LinearMotion::visualizeTrajectory(moveit_msgs::RobotTrajectory &generatedTrajectory)
{


    moveit_msgs::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory_start = _armState;
    display_trajectory.trajectory.push_back(generatedTrajectory);
    _display_publisher.publish(display_trajectory);


}

bool LinearMotion::generate_trajectory(const Goal::ConstPtr & req1, Result & res){
    auto req = *req1;
    bool successIK,validWaypoints,interpolationSuccess,stampSuccess;
    moveit_msgs::RobotTrajectory found_trajectory;
    moveit_msgs::Constraints motion_constrains=req.motion_constrains;
    std::vector<geometry_msgs::PoseStamped> pose_waypoints=req.pose_waypoints;
    std::vector<sensor_msgs::JointState> jointWaypoints=req.jointWaypoints;
    _kinematic_state=current_state_monitor_->getCurrentState();

    _planningGroup->startStateMonitor();

    res.success = true;


    ROS_INFO("[LinearMotion] Generating Trajectory");



    // Check if planning frames coicide
    // if the initial state is not given get the current state
    if (req.initial_joint_config.position.empty()){
        getCurrentJointConfig();
    }
    else{
        _startingState=req.initial_joint_config.position;
    }

    _armState.joint_state.position=_startingState;

    _plan_response.trajectory_start.joint_state.position=_startingState;

    //Check if targets have the same reference system with kinematic model

    if (req.jointWaypoints.empty() && req.pose_waypoints.empty()){
        ROS_ERROR("Trajectory target not given.");
        res.success = false;
        return true;
    }
    // Check if pose targets are given and get their IK solution
    else if (!req.pose_waypoints.empty()){
        if (pose_waypoints[0].header.frame_id != _planningFrame)
        {   Eigen::Affine3d trf,poseEig;
            trf=_kinematic_state->getFrameTransform(pose_waypoints[0].header.frame_id);
             tf::poseMsgToEigen (pose_waypoints[0].pose, poseEig);
             poseEig=trf*poseEig;
             tf::poseEigenToMsg(poseEig,pose_waypoints[0].pose);
             pose_waypoints[0].header.frame_id = _planningFrame;

        }
        moveit_msgs::RobotTrajectory ik_seed;
        std::vector<double> seed_state;
        jointWaypoints.resize(pose_waypoints.size());
        ROS_INFO("[LinearMotion] Getting IK solution of target pose");
        successIK=getIKsolution(pose_waypoints, "joint",jointWaypoints,_startingState);

         if(!successIK){
            ROS_WARN("[LinearMotion] No IK solution for target pose. Using probabilistic planner instead..");
            _planningGroup->setPoseTarget(pose_waypoints.back());
            bool success=useProbabilisticPlanner(motion_constrains, found_trajectory);
            if (success)
            {
            _plan_response.trajectory=found_trajectory;
            res.linear_motion_response=_plan_response;
            visualizeTrajectory( found_trajectory);
            return success;
            }
            else{
                res.success = false;
                return true;
            }
         }
         else {
             bool virtual_inter=interpolateToolSpace( jointWaypoints, motion_constrains,ik_seed,true);
             if (virtual_inter){
              seed_state=ik_seed.joint_trajectory.points[_trajectory_steps_tool].positions;
             successIK=getIKsolution(pose_waypoints, "joint",jointWaypoints,seed_state);}}
    }
    // Check if joint conf targets are given and check if they are collision free
    else if (!req.jointWaypoints.empty()){
      validWaypoints=isValid(jointWaypoints, motion_constrains);
      if(!validWaypoints){
          ROS_WARN("[LinearMotion] The joints' targets are invalid. Using probabilistic planner instead...");

          _planningGroup->setJointValueTarget(jointWaypoints.back().position);
          bool success=useProbabilisticPlanner(motion_constrains, found_trajectory);
          if (success)
          {
          _plan_response.trajectory=found_trajectory;
          res.linear_motion_response=_plan_response;
          visualizeTrajectory( found_trajectory);
          return success;
          }
          else
          {
              res.success = false;
              return true;
          }
      }
    }

   //Check if the linear interpolation takes place in joint or tool space
   if (req.planning_space=="joint"){
       interpolationSuccess=interpolateJointSpace(jointWaypoints,motion_constrains,found_trajectory);
       ROS_INFO_STREAM("The JOINT SPACE interpolation was" << (interpolationSuccess ? " succesfull" : " not succesfull"));
       }
   else if (req.planning_space=="tool"){
       interpolationSuccess=interpolateToolSpace(jointWaypoints, motion_constrains,found_trajectory,false);
        ROS_INFO_STREAM("The TOOL SPACE interpolation was" << (interpolationSuccess ? " succesfull" : " not succesfull"));
        }
   else{
        ROS_WARN("[LinearMotion] Unknown planning space. Use PRM planner instead");
        _planningGroup->setJointValueTarget(jointWaypoints.back().position);
        bool success=useProbabilisticPlanner(motion_constrains, found_trajectory);
        if (success)
        {
        _plan_response.trajectory=found_trajectory;
        res.linear_motion_response=_plan_response;
        visualizeTrajectory( found_trajectory);
        return success;
        }
        else
        {
            res.success = false;
            return true;
        }
        }

    //Add pos and vel to trajectory
    stampSuccess=addVelAcc(found_trajectory);
     //stampSuccess=true;

  if(!interpolationSuccess || !stampSuccess){
     ROS_WARN("[LinearMotion] Unsuccesfull interpolation. Using probabilistic planner instead ...");
     _planningGroup->setJointValueTarget(jointWaypoints.back().position);
     bool success=useProbabilisticPlanner(motion_constrains, found_trajectory);
     if (success)
     {
     _plan_response.trajectory=found_trajectory;
     res.linear_motion_response=_plan_response;
     visualizeTrajectory( found_trajectory);
     return success;
     }
     else
     {
         res.success = false;
         return true;
     }
     }

    _plan_response.trajectory=found_trajectory;

    ROS_INFO_STREAM("[LinearMotion] Execution time is: " << found_trajectory.joint_trajectory.points.back().time_from_start);
    visualizeTrajectory( found_trajectory);
    res.linear_motion_response=_plan_response;
    return true;
}
