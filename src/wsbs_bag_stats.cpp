#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <lightsfm/vector2d.hpp>
#include <lightsfm/angle.hpp>
#include <upo_msgs/PersonPoseArrayUPO.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <tf/transform_listener.h>
#include <teresa_wsbs/Info.h>
#include <animated_marker_msgs/AnimatedMarkerArray.h>	
#include <fstream>

#define foreach BOOST_FOREACH

std::vector<double> distance_to_target;
std::vector<double> target_force;
utils::Vector2d robot_position;
utils::Vector2d target_position;
std::vector<utils::Vector2d> people_position;
std::vector<utils::Vector2d> people_velocity;
std::ofstream out_stream;

bool calculateMetrics(int trajectory_index, double time)
{
	bool valid_trajectory = distance_to_target.size()>0 && target_force.size()>0;
	if (!valid_trajectory) {
		return false;
	}

	out_stream<<trajectory_index<<" "<<time;
	if (distance_to_target.size()>0) {
		double d=0;
		for (unsigned i=0;i<distance_to_target.size();i++) {
			d+=distance_to_target[i];
		}
		d /= (double) distance_to_target.size();
		ROS_INFO("Average distance to target: %f m",d);
		out_stream<<" "<<d;
		distance_to_target.clear();
	} else {
		out_stream<<" Inf";
	}
	if (target_force.size()>0) {
		double f=0;
		for (unsigned i=0;i<target_force.size();i++) {
			f+=target_force[i];
		}
		f /= (double) target_force.size();
		out_stream<<" "<<f;
		ROS_INFO("Average Target Group Force: %f",f);
		target_force.clear();
	} else {
		out_stream<<" Inf";
	}
	out_stream<<"\n";
	return true;
}


void readPeople(const upo_msgs::PersonPoseArrayUPO::ConstPtr& people_ptr)
{
	// TODO: Update people_position and people_velocity

}

void readTarget(animated_marker_msgs::AnimatedMarkerArray::ConstPtr& marker_array_ptr)
{
	target_position.set(marker_array_ptr->markers[0].pose.position.x,marker_array_ptr->markers[0].pose.position.y);
	distance_to_target.push_back((robot_position-target_position).norm());
}


void readOdom(const nav_msgs::Odometry::ConstPtr& odom)
{
	robot_position.set(odom->pose.pose.position.x,odom->pose.pose.position.y);
}

void readInfo(const teresa_wsbs::Info::ConstPtr& info)
{
	if (!info->target_detected || info->status!=5) {
		return;
	}
	utils::Vector2d f(info->target_group_force.x,info->target_group_force.y);
	target_force.push_back(f.norm());
	
	

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "wsbs_stats");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	std::string bag_name, people_topic;
	std::string out_file;
	bool trajectory_running=false;
	unsigned trajectory_counter=0;
	double lambda;
	double initial_time;
	pn.param<std::string>("bag_name",bag_name,"/home/ignacio/baseline.bag"); // Bag name
	pn.param<std::string>("people_topic",people_topic,"/people/navigation");
	pn.param<std::string>("out_file",out_file,"/home/ignacio/results.txt");
	pn.param<double>("lambda",lambda,0.4);

	std::vector<std::string> topics;
	topics.push_back(people_topic);
	topics.push_back("/odom");
	topics.push_back("/wsbs/controller/info");
	topics.push_back("/clicked_point");
	topics.push_back("/wsbs/markers/target");
	
  	out_stream.open (out_file);
	out_stream<<"Trajectory index	Time	Average Distance to Target	Average Target Social Force\n";
	rosbag::Bag bag;
	ROS_INFO("Opening bag %s...",bag_name.c_str());
	bag.open(bag_name,rosbag::bagmode::Read);
	ROS_INFO("Processing bag %s...",bag_name.c_str());	

	rosbag::View view(bag, rosbag::TopicQuery(topics));

	foreach(rosbag::MessageInstance const m, view) {
		//ROS_INFO("Reading %s at time %f s. ",m.getTopic().c_str(),m.getTime().toSec());
		if (m.getTopic().compare(people_topic)==0) {
			upo_msgs::PersonPoseArrayUPO::ConstPtr people_ptr = m.instantiate<upo_msgs::PersonPoseArrayUPO>();
			readPeople(people_ptr);
		} else if (m.getTopic().compare("/odom")==0) {
			nav_msgs::Odometry::ConstPtr odom_ptr = m.instantiate<nav_msgs::Odometry>();
			readOdom(odom_ptr);
		} else if (m.getTopic().compare("/wsbs/controller/info")==0) {
			teresa_wsbs::Info::ConstPtr info_ptr = m.instantiate<teresa_wsbs::Info>();
			readInfo(info_ptr);
		} else if (m.getTopic().compare("/wsbs/markers/target")==0) {
			animated_marker_msgs::AnimatedMarkerArray::ConstPtr marker_array_ptr = m.instantiate<animated_marker_msgs::AnimatedMarkerArray>();
			readTarget(marker_array_ptr);
		} else if (m.getTopic().compare("/clicked_point")==0) {
			trajectory_running = !trajectory_running;
			if (trajectory_running) {
				trajectory_counter++;
				initial_time = m.getTime().toSec();
				ROS_INFO("Trajectory N. %d begins at time %f s",trajectory_counter,m.getTime().toSec());		
			} else {
				ROS_INFO("Trajectory N. %d finishes at time %f s",trajectory_counter,m.getTime().toSec());
				bool valid =calculateMetrics(trajectory_counter, m.getTime().toSec()-initial_time);
				if (!valid) {
					ROS_INFO("Invalid trajectory");
					trajectory_counter--;
				}
			}
			
		}

	}
	out_stream.close();
	return 0;
}
