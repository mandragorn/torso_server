/*
 * spindle_server.cpp
 *
 *  Created on: Feb 3, 2012
 *      Author: sdries
 */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <sensor_msgs/JointState.h>

#include <amigo_actions/AmigoSpindleCommandAction.h>

double EPSILON = 0.005; // tolerance for determining whether goal is reached (in meters)

actionlib::SimpleActionServer<amigo_actions::AmigoSpindleCommandAction>* as_;

ros::Publisher spindle_pub_;

sensor_msgs::JointState torso_ref_;

bool meas_received;
sensor_msgs::JointState spindle_meas_;

void spindleMeasurementCb(const sensor_msgs::JointStateConstPtr& meas) {
    spindle_meas_ = *meas;
    meas_received = true;
}

void publishRef(const amigo_actions::AmigoSpindleCommandGoalConstPtr& spindle_goal) {
    torso_ref_ = sensor_msgs::JointState();
    torso_ref_.header.stamp = ros::Time::now();
    torso_ref_.name.push_back("torso_joint");
    torso_ref_.position.push_back(spindle_goal->spindle_height);
    spindle_pub_.publish(torso_ref_);
}

void goalCb(const amigo_actions::AmigoSpindleCommandGoalConstPtr& spindle_goal) {
	ros::NodeHandle n;

	meas_received = false;
    ros::Subscriber spindle_sub = n.subscribe("/amigo/torso/measurements", 10, &spindleMeasurementCb);

	publishRef(spindle_goal);

    ros::Rate r(10);
    while(n.ok() && as_->isActive()) {

    	if (as_->isNewGoalAvailable()) {
    		ROS_INFO("New goal received.");
    		publishRef(as_->acceptNewGoal());
    	}

        if (meas_received) {
            if (fabs(spindle_meas_.position[0] - torso_ref_.position[0]) < EPSILON) {
                amigo_actions::AmigoSpindleCommandResult result;
                result.spindle_height = spindle_meas_.position[0];
                as_->setSucceeded(result, "Goal height reached.");
            } else {
                amigo_actions::AmigoSpindleCommandFeedback feedback;
                feedback.spindle_height = spindle_meas_.position[0];
                as_->publishFeedback(feedback);
            }
        }

		r.sleep();
    }

    spindle_sub.shutdown();
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "spindle_server");
	ros::NodeHandle nh("~");

	as_ = new actionlib::SimpleActionServer<amigo_actions::AmigoSpindleCommandAction>(nh, nh.getNamespace(), &goalCb, false);

    spindle_pub_ = nh.advertise<sensor_msgs::JointState>("/amigo/torso/references", 100);

	as_->start();

	ROS_INFO("Action server is active and spinning...");

	ros::spin();

	return 0;
}

