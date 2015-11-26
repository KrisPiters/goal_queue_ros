/*
 * Author: Kris Piters on 24/11/2015
 */
#include <vector>
#include <string>

#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

std::vector<geometry_msgs::PoseStamped> g_goals;

void goals_cb(const geometry_msgs::PoseStamped::ConstPtr& newGoal)
{
	ROS_INFO("New goal added.");
	g_goals.push_back(*newGoal);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "goal_queue");

	ROS_INFO("Goal Queue Started.");
	
	int goal_itr = 0;
	bool ac_online = false;

	ros::NodeHandle n;
	
	ros::Subscriber goals_sub = n.subscribe("/goal_queue_goals", 20, goals_cb);
	
	MoveBaseClient ac("move_base", true);

	if(ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Contacted move_base action server");
		ac_online = true;
	}

	else
		ROS_INFO("Unable to contact move_base action server.");

	ros::Rate r(1);

	while(ros::ok() && ac_online)
	{
		if (g_goals.size() > 0)
		{
			move_base_msgs::MoveBaseGoal goal;

			if(goal_itr >= g_goals.size())
				goal_itr = 0;

			goal.target_pose = g_goals[goal_itr];
			goal.target_pose.header.stamp = ros::Time::now();

			ROS_INFO("Sending Goal nr: %d / %lu", goal_itr, g_goals.size() - 1);
			ac.sendGoal(goal);
			ac.waitForResult();

			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				ROS_INFO("Goal reached succesfully.");
  		else
				ROS_INFO("Reaching the goal has failed.");

			if(goal_itr++ == g_goals.size())
    		goal_itr = 0;

			ROS_INFO("Moving to goal nr: %d / %lu", goal_itr, g_goals.size() - 1);
		}
		ros::spinOnce();
		r.sleep();
	}

	ROS_INFO("Exiting Goal Queue...");

	return 0;
}
