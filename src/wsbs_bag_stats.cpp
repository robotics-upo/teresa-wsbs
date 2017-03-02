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
#include <plab/PLabInfo.h>

#define foreach BOOST_FOREACH

std::vector<double> distance_to_target;
std::vector<utils::Vector2d> target_force;
std::vector<utils::Vector2d> target_trajectory;
utils::Vector2d robot_position;
utils::Vector2d target_position;
utils::Angle target_yaw;
std::vector<utils::Vector2d> people_position;
std::vector<utils::Angle> people_yaw;
std::vector<double> confort;
std::vector<unsigned> target_detected;
std::ofstream out_stream;
double lambda;
double robot_area_radius;
int confort_particles;
bool available_groundtruth=true;
std::string bag_name;

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
	bool valid_trajectory = distance_to_target.size()>0 && target_detected.size()>0 && confort.size()>0;
	if (!valid_trajectory) {
		distance_to_target.clear();
		target_force.clear();
		target_detected.clear();
		confort.clear();
		target_trajectory.clear();
		
		return false;
	}

	out_stream<<bag_name<<"\t"<<trajectory_index<<"\t"<<time;
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
	if (target_force.size()>1 && target_trajectory.size()>1) {
		double work=0;
		for (unsigned i=0;i<std::min(target_trajectory.size()-1,target_force.size()-1);i++) {
			utils::Vector2d d = target_trajectory[i+1] - target_trajectory[i];
			//work += target_force[i].norm() * d.norm() * d.angleTo(target_force[i]).cos();
			work += target_force[i].dot(target_trajectory[i+1] - target_trajectory[i]);
		}
		out_stream<<"\t"<<work;
		ROS_INFO("Target Work: %f",work);
		target_force.clear();
		target_trajectory.clear();
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
	if (target_detected.size()>0) {
		double average_target_detected=0;
		for (unsigned i=0;i<target_detected.size();i++) {
			average_target_detected+=target_detected[i];
		}
		average_target_detected /= (double) target_detected.size();
		out_stream<<"\t"<<average_target_detected;
		ROS_INFO("Target detected: %f",average_target_detected);
		target_detected.clear(); 

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


void readGroundtruth(animated_marker_msgs::AnimatedMarkerArray::ConstPtr& marker_array_ptr)
{
	people_position.resize(marker_array_ptr->markers.size());
	people_yaw.resize(marker_array_ptr->markers.size());
	for (unsigned i = 0; i<marker_array_ptr->markers.size(); i++) {
		people_position[i].set(marker_array_ptr->markers[i].pose.position.x,marker_array_ptr->markers[i].pose.position.y);
		people_yaw[i] = utils::Angle::fromRadian(tf::getYaw(marker_array_ptr->markers[i].pose.orientation));
		people_yaw[i] += utils::Angle::fromRadian(M_PI*0.5);	
		if (marker_array_ptr->markers[i].color.r > 0.5) {
			target_position = people_position[i];
			target_yaw = people_yaw[i];
		}	
	}
	distance_to_target.push_back((robot_position-target_position).norm());
	confort.push_back(calculateConfort());
}


void readTarget(animated_marker_msgs::AnimatedMarkerArray::ConstPtr& marker_array_ptr)
{
	target_position.set(marker_array_ptr->markers[0].pose.position.x,marker_array_ptr->markers[0].pose.position.y);
	target_yaw = utils::Angle::fromRadian(tf::getYaw(marker_array_ptr->markers[0].pose.orientation));
	target_yaw += utils::Angle::fromRadian(M_PI*0.5);
	if (!available_groundtruth) {
		target_trajectory.push_back(target_position);
	}	
	distance_to_target.push_back((robot_position-target_position).norm());
	confort.push_back(calculateConfort());
}


void readOdom(const nav_msgs::Odometry::ConstPtr& odom)
{
	robot_position.set(odom->pose.pose.position.x,odom->pose.pose.position.y);
}

void readInfo(const teresa_wsbs::Info::ConstPtr& info)
{
	if (info->status!=5 && info->status!=6) {
		return;
	}
	
	if (info->status == 6) {
		target_detected.push_back(0);
	} else {
		if (available_groundtruth) {
			utils::Vector2d target0(info->target_pose.x, info->target_pose.y);
			if ((target_position - target0).norm() < 0.3) {
				target_detected.push_back(1);
			} else {
				target_detected.push_back(0);
			}
		} else {
			target_detected.push_back(1);
		}
	}

	if (!available_groundtruth) {
		utils::Vector2d f(info->target_group_force.x, info->target_group_force.y);
		target_force.push_back(f);
	}
	
}	

void readTargetForcesGroundtruth(plab::PLabInfo::ConstPtr& info)
{
	utils::Vector2d pos(info->target_pose.x,info->target_pose.y);
	//utils::Vector2d f(info->target_group_force.x,info->target_group_force.y);
	utils::Vector2d f(info->target_global_force.x,info->target_global_force.y);	
	target_trajectory.push_back(pos);
	target_force.push_back(f);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "wsbs_stats");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	std::string  people_topic;
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
	pn.param<bool>("use_clicked_points",use_clicked_points,true);  // false	
	pn.param<bool>("available_groundtruth",available_groundtruth,false); // true
	pn.param<double>("lambda",lambda,0.59);
	if (argc!=1) {
		bag_name = argv[1];
	}
	std::vector<std::string> topics;
	
	topics.push_back("/odom");
	topics.push_back("/wsbs/controller/info");
	topics.push_back("/clicked_point");

	if (available_groundtruth) {
		topics.push_back("/plab/markers/people");
		topics.push_back("/plab/info");
	} else {
		topics.push_back(people_topic);
		topics.push_back("/wsbs/markers/target");
	}

  	out_stream.open (out_file, std::ios::app);
	//out_stream<<"File name trajectory index\tTime\tAverage Distance to Target\tTarget Work\tConfortTarget detected\n";
	rosbag::Bag bag;
	ROS_INFO("Opening bag %s...",bag_name.c_str());
	bag.open(bag_name,rosbag::bagmode::Read);
	ROS_INFO("Processing bag %s...",bag_name.c_str());	

	rosbag::View view(bag, rosbag::TopicQuery(topics));
	ros::Time time;
	foreach(rosbag::MessageInstance const m, view) {
		time = m.getTime();
		if (m.getTopic().compare(people_topic)==0) {
			upo_msgs::PersonPoseArrayUPO::ConstPtr people_ptr = m.instantiate<upo_msgs::PersonPoseArrayUPO>();
			readPeople(people_ptr);
		} else if (m.getTopic().compare("/odom")==0) {
			nav_msgs::Odometry::ConstPtr odom_ptr = m.instantiate<nav_msgs::Odometry>();
			readOdom(odom_ptr);
		} else if (m.getTopic().compare("/wsbs/controller/info")==0) {
			//std::cout<<"A"<<std::endl;
			teresa_wsbs::Info::ConstPtr info_ptr = m.instantiate<teresa_wsbs::Info>();
			//std::cout<<"B"<<std::endl;
			readInfo(info_ptr);
			//std::cout<<"C"<<std::endl;
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
		} else if (m.getTopic().compare("/plab/markers/people")==0) {
				animated_marker_msgs::AnimatedMarkerArray::ConstPtr marker_array_ptr =
						m.instantiate<animated_marker_msgs::AnimatedMarkerArray>();
				readGroundtruth(marker_array_ptr);
		} else if (m.getTopic().compare("/plab/info")==0) {
				plab::PLabInfo::ConstPtr info = m.instantiate<plab::PLabInfo>();
				m.instantiate<plab::PLabInfo>();
				readTargetForcesGroundtruth(info);
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
