/*
 * spindle_server.cpp
 *
 *  Created on: Feb 3, 2012
 *      Author: sdries
 */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <amigo_msgs/spindle_setpoint.h>   // reference message type
#include <std_msgs/Float64.h>              // measurement message type

#include <amigo_actions/AmigoSpindleCommandAction.h>

double EPSILON = 0.005; // tolerance for determining whether goal is reached (in meters)

actionlib::SimpleActionServer<amigo_actions::AmigoSpindleCommandAction>* as_;

ros::Publisher spindle_pub_;

amigo_msgs::spindle_setpoint spindle_ref_;

bool meas_received;
std_msgs::Float64 spindle_meas_;

void spindleMeasurementCb(const std_msgs::Float64ConstPtr& meas) {
	if (as_->isActive()) {
		spindle_meas_ = *meas;
		meas_received = true;
	}
}

void publishRef(const amigo_actions::AmigoSpindleCommandGoalConstPtr& spindle_goal) {
	spindle_ref_.pos = spindle_goal->spindle_height;
	spindle_ref_.vel = 0;
	spindle_ref_.acc = 0;
	spindle_ref_.stop = 0;
    spindle_pub_.publish(spindle_ref_);
}

void goalCb(const amigo_actions::AmigoSpindleCommandGoalConstPtr& spindle_goal) {
    printf("Goal callback\n");
	ros::NodeHandle n;

	meas_received = false;
	ros::Subscriber spindle_sub = n.subscribe("/spindle_position", 10, &spindleMeasurementCb);

	publishRef(spindle_goal);

    ros::Rate r(10);
    while(n.ok() && as_->isActive()) {

    	if (as_->isNewGoalAvailable()) {
    		ROS_INFO("New goal received.");
    		publishRef(as_->acceptNewGoal());
    	}

    	if (meas_received && fabs(spindle_meas_.data - spindle_ref_.pos) < EPSILON) {
    		amigo_actions::AmigoSpindleCommandResult result;
    		result.spindle_height = spindle_meas_.data;
    		as_->setSucceeded(result, "Goal height reached.");
    	} else {
    		amigo_actions::AmigoSpindleCommandFeedback feedback;
			feedback.spindle_height = spindle_meas_.data;
			as_->publishFeedback(feedback);
    	}

		r.sleep();
    }

    spindle_sub.shutdown();

    printf("Goal callback - end\n");
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "spindle_server");
	ros::NodeHandle nh("~");

	as_ = new actionlib::SimpleActionServer<amigo_actions::AmigoSpindleCommandAction>(nh, nh.getNamespace(), &goalCb, false);

	//nh.getParam("spindle_measurement_topic", spindle_measurement_topic_);

	//std::string spindle_ref_topic;
	//nh.getParam("spindle_reference_topic", gripper_ref_topic);
	spindle_pub_ = nh.advertise<amigo_msgs::spindle_setpoint>("/spindle_controller/spindle_coordinates", 100);

	as_->start();

	ROS_INFO("Action server is active and spinning...");

	ros::spin();

	return 0;
}

