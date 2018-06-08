/*
 * Author: Kris Piters on 24/11/2015
 */
#include <vector>
#include <string>

#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

std::vector<geometry_msgs::PoseStamped> g_goals;
std::vector<geometry_msgs::Point> g_marker_points;
ros::Publisher g_goal_marker_pub;

void publish_markers(void)
{
	visualization_msgs::Marker line_strip;
	line_strip.header.frame_id = "map";
	line_strip.header.stamp = ros::Time::now();
	line_strip.action = visualization_msgs::Marker::ADD;
	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	line_strip.scale.x = 0.1;
	line_strip.color.r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	line_strip.color.g = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	line_strip.color.b = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	line_strip.color.a = 1.0;
	
	line_strip.points = g_marker_points;
	line_strip.points.push_back(g_marker_points[0]);

	g_goal_marker_pub.publish(line_strip);

}

void goals_cb(const geometry_msgs::PoseStamped::ConstPtr& newGoal)
{
	ROS_INFO("New goal added.");
	g_goals.push_back(*newGoal);
	
	// Add the new goal to the marker and publish
	geometry_msgs::PoseStamped ps = *newGoal;
	g_marker_points.push_back(ps.pose.position);
	publish_markers();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "goal_queue");

	ROS_INFO("Goal Queue Started.");
	
	int goal_itr = 0;
	bool ac_online = false;

	ros::NodeHandle n;
	
	ros::Subscriber goals_sub = n.subscribe("goal_queue_goal", 20, goals_cb);
	
	g_goal_marker_pub = n.advertise<visualization_msgs::Marker>("goal_queue_markers", 1);
	
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

			ROS_INFO("Sending Goal nr: %d / %lu", goal_itr + 1, g_goals.size());
			ac.sendGoal(goal);
			ac.waitForResult();

			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				ROS_INFO("Goal reached succesfully.");
			else
				ROS_INFO("Reaching the goal has failed.");

			if(goal_itr++ == g_goals.size())
				goal_itr = 0;

			ROS_INFO("Moving to goal nr: %d / %lu", goal_itr + 1, g_goals.size());
		}
		ros::spinOnce();
		r.sleep();
	}

	ROS_INFO("Exiting Goal Queue...");

	return 0;
}
