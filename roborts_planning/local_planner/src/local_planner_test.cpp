//
// Created by kehan on 2019/10/27.
//

#include <thread>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <actionlib/client/simple_action_client.h>

#include "roborts_msgs/LocalPlannerAction.h"
#include "state/error_code.h"

using roborts_common::ErrorCode;

class LocalPlannerTest {
 public:
  LocalPlannerTest() :
      local_planner_actionlib_client_("local_planner_node/local_planner_node_action", true) {
    ros::NodeHandle global_planner_nh("global_planner_node");
    global_path_sub_ =
        global_planner_nh.subscribe<nav_msgs::Path>("path", 1, &LocalPlannerTest::GlobalPathCallback, this);

    local_planner_actionlib_client_.waitForServer();
  }

  ~LocalPlannerTest() = default;

  void GlobalPathCallback(const nav_msgs::Path::ConstPtr &path) {
    ROS_INFO("Get new path.");
    local_planner_goal_.route = *path;
    local_planner_actionlib_client_.sendGoal(local_planner_goal_,
                                             boost::bind(&LocalPlannerTest::DoneCallback, this, _1, _2),
                                             boost::bind(&LocalPlannerTest::ActiveCallback, this),
                                             boost::bind(&LocalPlannerTest::FeedbackCallback, this, _1)
    );
  }

  void DoneCallback(const actionlib::SimpleClientGoalState &state,
                    const roborts_msgs::LocalPlannerResultConstPtr &result) {
    ROS_INFO("The goal is done with %s!", state.toString().c_str());
  }

  void ActiveCallback() {
    ROS_INFO("Action server has recived the goal, the goal is active!");
  }

  void FeedbackCallback(const roborts_msgs::LocalPlannerFeedbackConstPtr &feedback) {
    if (feedback->error_code != ErrorCode::OK) {
      ROS_INFO("%s", feedback->error_msg.c_str());
    }
  }
 private:
  ros::Subscriber global_path_sub_;
  roborts_msgs::LocalPlannerGoal local_planner_goal_;
  actionlib::SimpleActionClient<roborts_msgs::LocalPlannerAction> local_planner_actionlib_client_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "local_planner_test");
  LocalPlannerTest local_planner_test;
  ros::spin();
  return 0;
}

