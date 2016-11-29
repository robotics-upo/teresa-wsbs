#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>




int main(int argc, char** argv)
{
	ros::init(argc, argv, "wsbs_stats");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	std::string planner_bag, controller_bah;
	pn.param<std::string>("planner_bag",planner_bag,"");
	pn.param<std::string>("controller_bag",controller_bag,"");

	return 0;
}
