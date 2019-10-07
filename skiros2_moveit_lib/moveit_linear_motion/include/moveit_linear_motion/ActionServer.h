#ifndef ACTION_SERVER_H
#define ACTION_SERVER_H

#include <string>
#include <vector>
#include <functional>

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <std_srvs/Empty.h>


template <class Action>
class ActionServer
{
public:
  using Goal = typename Action::_action_goal_type::_goal_type;
  using Feedback = typename Action::_action_feedback_type::_feedback_type;
  using Result = typename Action::_action_result_type::_result_type;
  using Status = typename Action::_action_result_type::_status_type;

  enum STATUS {
    PREEMPTED = Status::PREEMPTED,
    SUCCEEDED = Status::SUCCEEDED,
    ABORTED = Status::ABORTED,
  };

protected:
  std::string name_;

  actionlib::SimpleActionServer<Action> server_;
  Result result_;
  std::string result_msg_ = "";

  ros::ServiceServer config_;

  void goalCB() {
    if (server_.isActive()) {
      ROS_ERROR("Current goal is still running.");
    } else {
      typename Goal::ConstPtr goal = server_.acceptNewGoal();
      STATUS status = execute(goal);
      switch (status) {
        case STATUS::PREEMPTED:
          server_.setPreempted(result_, result_msg_);
          break;
        case STATUS::ABORTED:
          server_.setAborted(result_, result_msg_);
          break;
        case STATUS::SUCCEEDED:
          server_.setSucceeded(result_, result_msg_);
          break;
      }
    }
  }

  void preemptCB() {
    ROS_INFO_STREAM(name_ << ": Preempted");
    server_.setPreempted();
  }

  bool configCB(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    // ROS_INFO_STREAM(name_ << ": Configuring...");
    this->onConfig();
    return true;
  }

public:
  ActionServer(const std::string & name = "ActionServer") : server_(name, false) {
    this->name_ = ros::this_node::getName();
  }
  virtual ~ActionServer() = 0;

  virtual void onReset() {}
  virtual void onConfig() { this->onReset(); }

  void init() {
    server_.registerGoalCallback(std::bind(&ActionServer::goalCB, this));
    server_.registerPreemptCallback(std::bind(&ActionServer::preemptCB, this));
    ros::NodeHandle nh("~");
    config_ = nh.advertiseService("config", &ActionServer::configCB, this);
    server_.start();
    ROS_INFO_STREAM("Started action server " << name_);
  }

  void sendFeedback(const std::string & msg) {
    Feedback feedback;
    feedback.message = msg;
    ROS_INFO_STREAM("Send feedback: " << msg);
    server_.publishFeedback(feedback);
  }

  void setResult(const Result & result, const std::string & msg) {
    result_ = result;
    result_msg_ = msg;
  }

  STATUS preempt(const std::string & msg = "PREEMPTED") {
    setResult(Result(), msg);
    return STATUS::PREEMPTED;
  }

  STATUS abort(const std::string & msg = "ABORTED") {
    setResult(Result(), msg);
    return STATUS::ABORTED;
  }

  STATUS success(const Result & result, const std::string & msg = "SUCCEEDED") {
    setResult(result, msg);
    return STATUS::SUCCEEDED;
  }

  virtual STATUS execute(const typename Goal::ConstPtr & goal) = 0;
};

template <class Action>
ActionServer<Action>::~ActionServer() {}


#endif // ACTION_SERVER_H
