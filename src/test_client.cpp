#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <amigo_actions/AmigoSpindleCommandAction.h>

int main (int argc, char **argv) {
  ros::init(argc, argv, "test_client");

  if (argc != 2) {
	  ROS_ERROR("Arguments: SPINDLE_HEIGHT");
	  return 1;
  }

  double spindle_height = atof(argv[1]);

  // create the action client
  actionlib::SimpleActionClient<amigo_actions::AmigoSpindleCommandAction> ac("spindle_server");
  ac.waitForServer();

  // send a goal to the action
  amigo_actions::AmigoSpindleCommandGoal goal;
  goal.spindle_height = spindle_height;

  actionlib::SimpleClientGoalState state = ac.sendGoalAndWait(goal);
  ROS_WARN("Current state: %s", state.toString().c_str());

  ros::shutdown();

  return 0;
}
