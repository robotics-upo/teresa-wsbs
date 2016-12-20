/***********************************************************************/
/**                                                                    */
/** wsbs_planner.cpp                                                   */
/**                                                                    */
/** Copyright (c) 2016, Service Robotics Lab.                          */ 
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** Ignacio Perez-Hurtado (maintainer)                                 */
/** Jesus Capitan                                                      */
/** Fernando Caballero                                                 */
/** Luis Merino                                                        */
/**                                                                    */   
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/***********************************************************************/


#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <tinyxml.h>
#include <lightsfm/sfm.hpp>
#include <lightsfm/rosmap.hpp>
#include <lightpomcp/Pomcp.hpp>
#include <teresa_wsbs/start.h>
#include <teresa_wsbs/stop.h>
#include <teresa_wsbs/select_mode.h>
#include <teresa_wsbs/model.hpp>
#include <vector>
#include <map>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include <upo_msgs/PersonPoseArrayUPO.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>

namespace wsbs
{

#define	WAITING_FOR_START    0 
#define	WAITING_FOR_ODOM     1 
#define	WAITING_FOR_LASER    2 
#define	WAITING_FOR_XTION    3
#define	WAITING_FOR_PEOPLE   4
#define	RUNNING              5 
#define	TARGET_LOST          6
#define	FINISHED             7
#define	ABORTED		     8


class Planner
{
public:
	Planner(ros::NodeHandle& n, ros::NodeHandle& pn);
	~Planner();

private:
	bool start(teresa_wsbs::start::Request &req, teresa_wsbs::start::Response &res);
	bool stop(teresa_wsbs::stop::Request &req, teresa_wsbs::stop::Response &res);
	void statusReceived(const std_msgs::UInt8::ConstPtr& status);
	void odomReceived(const nav_msgs::Odometry::ConstPtr& odom);
	void peopleReceived(const upo_msgs::PersonPoseArrayUPO::ConstPtr& people);
	model::Observation& getObservation(model::Observation& observation);
	bool transformPoint(double& x, double& y, const std::string& sourceFrameId, const std::string& targetFrameId) const;
	bool transformPose(double& x, double& y, double& theta, const std::string& sourceFrameId, const std::string& targetFrameId) const;

	utils::Vector2d publishGoals(const pomcp::VectorBelief<model::State>& belief) const;

	void readGoals(TiXmlNode *pParent);
	
	std::vector<utils::Vector2d> goals;
	
	ros::ServiceClient controller_start;
	ros::ServiceClient controller_stop;
	ros::ServiceClient controller_mode;
	
	unsigned targetId;
	bool running, firstOdom, firstPeople, target_hidden;
	model::Simulator *simulator;
	
	tf::TransformListener tf_listener;
	double cell_size,running_time;

	ros::Publisher goal_markers_pub;
	ros::Publisher target_pose_pub;
	
};


inline
Planner::Planner(ros::NodeHandle& n, ros::NodeHandle& pn)
: targetId(0),
  running(false),
  firstOdom(false),
  firstPeople(false),
  target_hidden(true),
  simulator(NULL),
  tf_listener(ros::Duration(10))
{
	std::string goals_file, odom_id, people_id,paths_file;
	double freq, discount, timeout,threshold,exploration_constant,tracking_range,goal_radius;
	
	pn.param<std::string>("goals_file",goals_file,"");
	pn.param<std::string>("paths_file",paths_file,"");
	pn.param<double>("freq",freq,2.0); //0.5
	pn.param<double>("discount",discount,0.75);
	pn.param<double>("cell_size",cell_size,1.0);

	pn.param<std::string>("odom_id",odom_id,"/odom");
	pn.param<std::string>("people_id",people_id,"/people/navigation");
	pn.param<double>("timeout",timeout,1.0); //1.0 0.2
	pn.param<double>("threshold",threshold,0.01);
	pn.param<double>("exploration_constant",exploration_constant,1);
	pn.param<double>("tracking_range",tracking_range,4);
	pn.param<double>("goal_radius",goal_radius,1.0);
	pn.param<double>("running_time",running_time,2.0); // 2.0 0.5
	

	TiXmlDocument xml_doc(goals_file);
	if (xml_doc.LoadFile()) {
		readGoals(&xml_doc);
		if (goals.empty()) {
			ROS_FATAL("No goals");
			ROS_BREAK();
		}
	} else {
		ROS_FATAL("Cannot read goals");
		ROS_BREAK();
	}

	//model::FilePathProvider pathProvider(paths_file,0.5);
	model::AStarPathProvider pathProvider;

	ros::ServiceServer start_srv = n.advertiseService("/wsbs/start", &Planner::start,this);
	ros::ServiceServer stop_srv  = n.advertiseService("/wsbs/stop", &Planner::stop,this);
	
	controller_start = n.serviceClient<teresa_wsbs::start>("/wsbs/controller/start");	
	controller_stop = n.serviceClient<teresa_wsbs::stop>("/wsbs/controller/stop");	
	controller_mode = n.serviceClient<teresa_wsbs::select_mode>("/wsbs/select_mode");

	ros::Subscriber controller_status_sub = n.subscribe<std_msgs::UInt8>("/wsbs/status", 1, &Planner::statusReceived,this);

	ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>(odom_id, 1, &Planner::odomReceived,this);
	ros::Subscriber people_sub = n.subscribe<upo_msgs::PersonPoseArrayUPO>(people_id, 1, &Planner::peopleReceived,this);
	goal_markers_pub = pn.advertise<visualization_msgs::MarkerArray>("/wsbs/markers/goals", 1);
	target_pose_pub = pn.advertise<upo_msgs::PersonPoseUPO>("/wsbs/planner/target",1);
	
	
	ros::Rate r(freq);
	
	//teresa_wsbs::start srv;
	//srv.request.target_id=1;
	//bool success= controller_start.call(srv);
	//targetId = 1;
	//while(!success) {
	//	success= controller_start.call(srv);
	//}
	



	simulator = new model::Simulator(discount,cell_size,goals,pathProvider,tracking_range,goal_radius,running_time);
	pomcp::PomcpPlanner<model::State,model::Observation,model::Action> planner(*simulator,timeout,threshold,exploration_constant);
	model::Observation obs;
	unsigned action;
	utils::Vector2d likelyGoal;
	teresa_wsbs::select_mode mode_srv;
	bool reset=true;
	unsigned size,depth;
	while(n.ok()) {
		if (running && firstOdom && firstPeople) {
			
			action = planner.getAction();
			likelyGoal = publishGoals(planner.getCurrentBelief());
			utils::Vector2d target_pos,target_vel;
			for (auto it = planner.getCurrentBelief().getParticles().begin(); it != planner.getCurrentBelief().getParticles().end(); ++it) {
				target_pos += (it->target_pos);
				target_vel += (it->target_vel);
			}
			target_pos /= planner.getCurrentBelief().size();
			target_vel /= planner.getCurrentBelief().size();
			planner.computeInfo(size,depth);
			std::cout<<"DEPTH: "<<depth<<" SIZE: "<<size<<std::endl;
			if (!reset) {
				mode_srv.request.controller_mode = action;
				double x = likelyGoal.getX();
				double y = likelyGoal.getY();
				//transformPoint(x,y,"map","odom");
				mode_srv.request.goal_x = x;
				mode_srv.request.goal_y = y;
				x = target_pos.getX();
				y = target_pos.getY();
				transformPoint(x,y,"map","odom");
				mode_srv.request.target_pos_x = x;
				mode_srv.request.target_pos_y = y;
				mode_srv.request.target_vel_x = target_vel.getX();
				mode_srv.request.target_vel_y = target_vel.getY();
				controller_mode.call(mode_srv);	
			}					
			r.sleep();	
			ros::spinOnce();
			getObservation(obs);
			std::cout<<obs<<std::endl;			
			
			reset = planner.moveTo(action,obs);
			/*std::cout<<"--- "<< planner.getCurrentBelief().size()<<" --"<<std::endl;
			for (auto it = planner.getCurrentBelief().data().begin(); it != planner.getCurrentBelief().data().end(); ++it) {
				std::cout<<(it->first.target_pos)<<" "<<(it->first.robot_pos)<<(it->second)<<std::endl;
			}*/
			

			std::cout<<"RESET: "<<reset<<std::endl;
			
		} else {
			r.sleep();	
			ros::spinOnce();
		}
		
			
	}
}







utils::Vector2d Planner::publishGoals(const pomcp::VectorBelief<model::State>& belief) const
{
	utils::Vector2d likelyGoal;
	utils::Vector2d targetPos;
	utils::Vector2d targetVel;	


	std::map<utils::Vector2d, double> goals;
	for (auto it = belief.getParticles().cbegin(); it!= belief.getParticles().cend(); ++it) {
		if (goals.count(it->goal)==0) {
			goals[it->goal]  = 1.0/(double)(belief.size());
		} else {
			goals[it->goal] += 1.0/(double)(belief.size());
		}
		targetPos += it->target_pos;
		targetVel += it->target_vel;
	}

	if (goals.empty()) {
		for (unsigned i = 0 ; i< Planner::goals.size(); i++) {
			goals[Planner::goals[i]] = 1.0 / (double) Planner::goals.size(); 
		}
	} else {
		targetPos /= (double) belief.getParticles().size();
		targetVel /= (double) belief.getParticles().size();
		upo_msgs::PersonPoseUPO target;
		target.header.frame_id = "map";
          	target.header.stamp = ros::Time::now();
		target.id = 0;
		target.vel = targetVel.norm();
		target.position.x = targetPos.getX();
		target.position.y = targetPos.getY();
		target.position.z = 0;
		target.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, targetVel.angle().toRadian());	
		target_pose_pub.publish(target);
		
	}
	
	double max=0;
	int index=0;
	visualization_msgs::MarkerArray goal_markers; 
	for (auto it = goals.begin(); it != goals.end(); ++it) {
		visualization_msgs::Marker marker;
		marker.header.frame_id = "map";
          	marker.header.stamp = ros::Time::now();
		marker.ns = "goal_markers";
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.id = index++;
		marker.action = visualization_msgs::Marker::ADD;
		marker.color.a = 1.0;			
		marker.color.r = 0.0;
            	marker.color.g = 0.0;
            	marker.color.b = 1.0;
		marker.lifetime = ros::Duration(4);
		marker.scale.x = it->second;
            	marker.scale.y = it->second;
            	marker.scale.z = it->second;
            	marker.pose.position.x = it->first.getX();
		marker.pose.position.y = it->first.getY();
		marker.pose.position.z = 1.0;
		goal_markers.markers.push_back(marker);
		if (it->second > max) {
			max = it->second;
			likelyGoal = it->first;
		}
	}
	goal_markers_pub.publish(goal_markers);	
	
	
	return likelyGoal;

}


bool Planner::transformPose(double& x, double& y, double& theta, const std::string& sourceFrameId, const std::string& targetFrameId) const
{
	tf::Stamped<tf::Pose> pose,tfPose;
	pose.setData(tf::Pose(tf::createQuaternionFromRPY(0,0,theta), tf::Vector3(x,y,0)));
	pose.frame_id_ = sourceFrameId;
	pose.stamp_ = ros::Time(0);
	try
	{
		tf_listener.transformPose(targetFrameId, pose, tfPose);
	} catch(std::exception &e) {
		ROS_ERROR("%s",e.what());
		return false;
	}
	x = tfPose.getOrigin().getX();
	y = tfPose.getOrigin().getY();
	tf::Matrix3x3 m(tfPose.getRotation());
	double roll,pitch;
	m.getRPY(roll, pitch, theta);
	return true;
}



bool Planner::transformPoint(double& x, double& y, const std::string& sourceFrameId, const std::string& targetFrameId) const
{
	tf::Stamped<tf::Pose> pose,tfPose;
	pose.setData(tf::Pose(tf::createQuaternionFromRPY(0,0,0), tf::Vector3(x,y,0)));
	pose.frame_id_ = sourceFrameId;
	pose.stamp_ = ros::Time(0);
	try
	{
		tf_listener.transformPose(targetFrameId, pose, tfPose);
	} catch(std::exception &e) {
		ROS_ERROR("%s",e.what());
		return false;
	}
	x = tfPose.getOrigin().getX();
	y = tfPose.getOrigin().getY();
	return true;
}


Planner::~Planner()
{
	delete simulator;
}


model::Observation& Planner::getObservation(model::Observation& observation)
{
	observation.robot_pos_grid_x = (int)std::round(simulator->robot_pos.getX()/cell_size);
	observation.robot_pos_grid_y = (int)std::round(simulator->robot_pos.getY()/cell_size);
	observation.target_pos_grid_x = (int)std::round(simulator->target_pos.getX()/cell_size);
	observation.target_pos_grid_y = (int)std::round(simulator->target_pos.getY()/cell_size);
	observation.target_hidden = target_hidden;
	return observation;
}


void Planner::odomReceived(const nav_msgs::Odometry::ConstPtr& odom)
{
	double x,y;
	x = odom->pose.pose.position.x;
	y = odom->pose.pose.position.y;
	double yaw =tf::getYaw(odom->pose.pose.orientation);	
	
	if (transformPose(x, y, yaw, odom->header.frame_id, "map")) {
		simulator->robot_pos.set(x,y);
		simulator->robot_vel.set(odom->twist.twist.linear.x * std::cos(yaw), odom->twist.twist.linear.x * std::sin(yaw));
		//simulator->robot_pos+=simulator->robot_vel * running_time;
		firstOdom=true;
	}
}

void Planner::peopleReceived(const upo_msgs::PersonPoseArrayUPO::ConstPtr& people)
{
	target_hidden=true;
	
	for (unsigned i=0; i< people->personPoses.size(); i++) {
		if (people->personPoses[i].id == targetId) {
			double x = people->personPoses[i].position.x;
			double y = people->personPoses[i].position.y;
			double yaw = tf::getYaw(people->personPoses[i].orientation);
			if (transformPose(x, y, yaw, people->personPoses[i].header.frame_id, "map")) {
				simulator->target_pos.set(x,y);
				simulator->target_vel.set(people->personPoses[i].vel * std::cos(yaw), people->personPoses[i].vel * std::sin(yaw));
				//simulator->target_pos+=simulator->target_vel * running_time;
				target_hidden=false;
			}
		}
	}	
	firstPeople=true;
}

void Planner::statusReceived(const std_msgs::UInt8::ConstPtr& status)
{
	if (status->data == RUNNING || status->data == TARGET_LOST) {
		running = true;
	} else {
		running = false;
	}
}


bool Planner::start(teresa_wsbs::start::Request &req, teresa_wsbs::start::Response &res) 
{
	teresa_wsbs::start srv;
	srv.request = req;
	bool success= controller_start.call(srv);
	res = srv.response;
	if (success && res.error_code==0) {
		targetId = req.target_id;
	} 
	return success;
}


bool Planner::stop(teresa_wsbs::stop::Request &req, teresa_wsbs::stop::Response &res)
{
	teresa_wsbs::stop srv;
	srv.request = req;
	bool success= controller_stop.call(srv);
	res = srv.response;
	return success;
}


inline
void Planner::readGoals(TiXmlNode *pParent)
{
	if ( !pParent ) return;

	if ( pParent->Type() == TiXmlNode::TINYXML_ELEMENT) {
		TiXmlElement* pElement = pParent->ToElement();
		TiXmlAttribute* pAttrib=pElement->FirstAttribute();
		if (strcmp(pParent->Value(),"goal")==0) {
			utils::Vector2d goal;
			double x,y,radius;
			std::string id;
			id.assign(pAttrib->Value());
			pAttrib=pAttrib->Next();
			pAttrib->QueryDoubleValue(&x);
			pAttrib=pAttrib->Next();
			pAttrib->QueryDoubleValue(&y);
			pAttrib=pAttrib->Next();
			pAttrib->QueryDoubleValue(&radius);
			goal.set(x,y);
			goals.push_back(goal);
		} 
	}

	for (TiXmlNode* pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling()) {
		readGoals( pChild );
	}

}



}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "wsbs_planner");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	wsbs::Planner node(n,pn);
	return 0;
}
