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
visualization_msgs::Marker g_line_strip;

int g_goal_itr;
bool g_pub_next_goal;

void update_markers(void)
{
	g_line_strip.header.frame_id = "map";
	g_line_strip.header.stamp = ros::Time::now();
	g_line_strip.action = visualization_msgs::Marker::ADD;
	g_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	g_line_strip.scale.x = 0.1;
	g_line_strip.color.r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	g_line_strip.color.g = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	g_line_strip.color.b = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	g_line_strip.color.a = 1.0;
	g_line_strip.lifetime.sec = 2.0; 	
	g_line_strip.points = g_marker_points;
	g_line_strip.points.push_back(g_marker_points[0]);	
}

void goals_cb(const geometry_msgs::PoseStamped::ConstPtr& newGoal)
{
	ROS_INFO("New goal added.");
	g_goals.push_back(*newGoal);
	
	// Add the new goal to the marker and publish
	geometry_msgs::PoseStamped ps = *newGoal;
	
	// TODO tranform pose to fixed frame (default map)
	g_marker_points.push_back(ps.pose.position);
	update_markers();
}

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const move_base_msgs::MoveBaseResult::ConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  //ROS_INFO("Answer: %i", result->sequence.back());
  //ros::shutdown();

	if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Goal reached succesfully.");
	else
		ROS_INFO("Reaching the goal has failed.");

	/*if(g_goal_itr++ >= g_goals.size())
		g_goal_itr = 0;
	*/
	g_goal_itr++;
	g_pub_next_goal = true;
	
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
	ROS_INFO("Moving to goal nr: %d / %lu", g_goal_itr + 1, g_goals.size());
}

// Called every time feedback is received for the goal
void feedbackCb(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback)
{
  //ROS_INFO("Got Feedback");// of length %lu", feedback->base_position.size());
}

int main(int argc, char** argv)
{
	bool ac_online = false;
	g_pub_next_goal = true;
	g_goal_itr = 0;
	
	ros::init(argc, argv, "goal_queue");

	ROS_INFO("Goal Queue Started.");
	
	
	

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
			g_goal_marker_pub.publish(g_line_strip);

			if(g_pub_next_goal)
			{
				
				if(g_goal_itr >= g_goals.size())
					g_goal_itr = 0;
				
				move_base_msgs::MoveBaseGoal goal;
				goal.target_pose = g_goals[g_goal_itr];
				goal.target_pose.header.stamp = ros::Time::now();
				ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
				//ROS_INFO("Moving to goal nr: %d / %lu", g_goal_itr + 1, g_goals.size());
				g_pub_next_goal = false;
			}
			
			
			/* // Old blocking code
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
			*/
			
		}
		ros::spinOnce();
		r.sleep();
	}

	ROS_INFO("Exiting Goal Queue...");

	return 0;
}
