/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Fontys Hogescholen Eindhoven.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Fontys Hogescholen Eindhoven nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Kris Piters on 22/11/2013
*********************************************************************/

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

	ROS_INFO("Exiting Rosbee Goal Queue...");

	return 0;
}
