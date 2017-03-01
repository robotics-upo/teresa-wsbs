#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <lightsfm/vector2d.hpp>
#include <lightsfm/angle.hpp>
#include <lightpomcp/Random.hpp>
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
utils::Angle target_yaw;
std::vector<utils::Vector2d> people_position;
std::vector<utils::Angle> people_yaw;
std::vector<double> confort;
std::ofstream out_stream;
double lambda;
double robot_area_radius;
int confort_particles;


// return true if x is in inside the intimate distance(*factor) of person p with yaw
bool intimateDistance(const utils::Vector2d& x, const utils::Vector2d& p, const utils::Angle& yaw, double factor=1.0)
{
	utils::Vector2d n = (p - x).normalized();
	utils::Vector2d e(yaw.cos(),yaw.sin());
	double cos_phi = -n.dot(e);
	double w = lambda + (1-lambda)*(1+cos_phi)*0.5;
	w *= factor;
	double dis = (p-x).norm();
	return dis < w;	
}


double calculateConfort()
{
	utils::Vector2d x;
	double squared_robot_area_radius = robot_area_radius*robot_area_radius;
	double score=0;
	for (int i = 0; i< confort_particles; i++) {
		// Select a random particle in the robot area
		do {
			x.set(utils::RANDOM()*robot_area_radius, utils::RANDOM()*robot_area_radius);
		} while (x[0]*x[0] + x[1]*x[1] > squared_robot_area_radius);
		unsigned quadrant = utils::RANDOM(4);
		if (quadrant==2 || quadrant==3) {
			x.setX(-x[0]);
		}
		if (quadrant==1 || quadrant==2) {
			x.setY(-x[1]); 
		}
		x+=robot_position;
		bool tooCloseToSomebody=false;
		for (unsigned j=0;j<people_position.size();j++) {
			if (intimateDistance(x,people_position[j],people_yaw[j])) {
				tooCloseToSomebody=true;
				break;
			}	
		}
		if (tooCloseToSomebody) {
			continue;
		}
		if (intimateDistance(x,target_position,target_yaw,3.0)) {
			score += 1.0;	
		} else if ((x - target_position).norm() < 3.0) {
			score += 0.5;
		} 
	}
	score /= (double)confort_particles;
	return score;

}


bool calculateMetrics(int trajectory_index, double time)
{
	bool valid_trajectory = distance_to_target.size()>0 && target_force.size()>0;
	if (!valid_trajectory) {
		return false;
	}

	out_stream<<trajectory_index<<"\t"<<time;
	if (distance_to_target.size()>0) {
		double d=0;
		for (unsigned i=0;i<distance_to_target.size();i++) {
			d+=distance_to_target[i];
		}
		d /= (double) distance_to_target.size();
		ROS_INFO("Average distance to target: %f m",d);
		out_stream<<"\t"<<d;
		distance_to_target.clear();
	} else {
		out_stream<<"\tInf";
	}
	if (target_force.size()>0) {
		double f=0;
		for (unsigned i=0;i<target_force.size();i++) {
			f+=target_force[i];
		}
		f /= (double) target_force.size();
		out_stream<<"\t"<<f;
		ROS_INFO("Average Target Group Force: %f",f);
		target_force.clear();
	} else {
		out_stream<<"\tInf";
	}
	if (confort.size()>0) {
		double average_confort=0;
		for (unsigned i=0;i<confort.size();i++) {
			average_confort+=confort[i];
		}
		average_confort /= (double) confort.size();
		out_stream<<"\t"<<average_confort;
		ROS_INFO("Average confort: %f",average_confort);
		confort.clear();
	} else {
		out_stream<<"\t0";
	}
	out_stream<<"\n";
	return true;
}


void readPeople(const upo_msgs::PersonPoseArrayUPO::ConstPtr& people_ptr)
{
	people_position.resize(people_ptr->personPoses.size());
	people_yaw.resize(people_ptr->personPoses.size());
	for (unsigned i=0; i< people_ptr->personPoses.size(); i++) {
		people_position[i].set(people_ptr->personPoses[i].position.x,people_ptr->personPoses[i].position.y);
		people_yaw[i] = utils::Angle::fromRadian(tf::getYaw(people_ptr->personPoses[i].orientation));
	}
}

void readTarget(animated_marker_msgs::AnimatedMarkerArray::ConstPtr& marker_array_ptr)
{
	target_position.set(marker_array_ptr->markers[0].pose.position.x,marker_array_ptr->markers[0].pose.position.y);
	target_yaw = utils::Angle::fromRadian(tf::getYaw(marker_array_ptr->markers[0].pose.orientation));
	target_yaw += utils::Angle::fromRadian(M_PI*0.5);	
	distance_to_target.push_back((robot_position-target_position).norm());
	confort.push_back(calculateConfort());
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
	bool use_clicked_points=false;
	
	double initial_time;
	pn.param<std::string>("bag_name",bag_name,"/home/ignacio/baseline.bag"); // Bag name
	pn.param<std::string>("people_topic",people_topic,"/people/navigation");
	pn.param<std::string>("out_file",out_file,"/home/ignacio/results.txt");
	pn.param<double>("robot_area_radius",robot_area_radius,0.5); 
	pn.param<int>("confort_particles",confort_particles,1000);
	pn.param<bool>("use_clicked_points",use_clicked_points,false);	
	pn.param<double>("lambda",lambda,0.4);
	if (argc!=1) {
		bag_name = argv[1];
	}
	std::vector<std::string> topics;
	topics.push_back(people_topic);
	topics.push_back("/odom");
	topics.push_back("/wsbs/controller/info");
	topics.push_back("/clicked_point");
	topics.push_back("/wsbs/markers/target");
	
  	out_stream.open (out_file);
	out_stream<<"Trajectory index\tTime\tAverage Distance to Target\tAverage Target Social Force\tConfort\n";
	rosbag::Bag bag;
	ROS_INFO("Opening bag %s...",bag_name.c_str());
	bag.open(bag_name,rosbag::bagmode::Read);
	ROS_INFO("Processing bag %s...",bag_name.c_str());	

	rosbag::View view(bag, rosbag::TopicQuery(topics));
	ros::Time time;
	foreach(rosbag::MessageInstance const m, view) {
		time = m.getTime();
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
			animated_marker_msgs::AnimatedMarkerArray::ConstPtr marker_array_ptr =
						m.instantiate<animated_marker_msgs::AnimatedMarkerArray>();
			readTarget(marker_array_ptr);
		} else if ((use_clicked_points && m.getTopic().compare("/clicked_point")==0) || (!use_clicked_points && !trajectory_running)) {
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
	if (!use_clicked_points) {
		ROS_INFO("Trajectory N. %d finishes at time %f s",trajectory_counter,time.toSec());
		bool valid =calculateMetrics(trajectory_counter, time.toSec()-initial_time);
		if (!valid) {
			ROS_INFO("Invalid trajectory");
			trajectory_counter--;
		}
	}
	out_stream.close();
	return 0;
}
