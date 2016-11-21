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
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include <upo_msgs/PersonPoseArrayUPO.h>

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

	void readGoals(TiXmlNode *pParent);
	std::vector<utils::Vector2d> goals;
	
	ros::ServiceClient controller_start;
	ros::ServiceClient controller_stop;
	ros::ServiceClient controller_mode;
	
	unsigned targetId;
	bool running, firstOdom, firstPeople, target_hidden;
	model::Simulator *simulator;
	tf::TransformListener tf_listener;
	double cell_size;
	
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
	std::string goals_file, odom_id, people_id;
	double freq, discount, timeout,threshold,exploration_constant,tracking_range,goal_radius,running_time;
	
	pn.param<std::string>("goals_file",goals_file,"");
	pn.param<double>("freq",freq,0.5);
	pn.param<double>("discount",discount,0.8);
	pn.param<double>("cell_size",cell_size,0.5);

	pn.param<std::string>("odom_id",odom_id,"/odom");
	pn.param<std::string>("people_id",people_id,"/people/navigation");
	pn.param<double>("timeout",timeout,1.5);
	pn.param<double>("threshold",threshold,0.001);
	pn.param<double>("exploration_constant",exploration_constant,100);
	pn.param<double>("tracking_range",tracking_range,5);
	pn.param<double>("goal_radius",goal_radius,0.25);
	pn.param<double>("running_time",running_time,2.0);
	

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

	ros::ServiceServer start_srv = n.advertiseService("/wsbs/start", &Planner::start,this);
	ros::ServiceServer stop_srv  = n.advertiseService("/wsbs/stop", &Planner::stop,this);
	
	controller_start = n.serviceClient<teresa_wsbs::start>("/wsbs/controller/start");	
	controller_stop = n.serviceClient<teresa_wsbs::stop>("/wsbs/controller/stop");	
	controller_mode = n.serviceClient<teresa_wsbs::select_mode>("/wsbs/select_mode");

	ros::Subscriber controller_status_sub = n.subscribe<std_msgs::UInt8>("/wsbs/status", 1, &Planner::statusReceived,this);

	ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>(odom_id, 1, &Planner::odomReceived,this);
	ros::Subscriber people_sub = n.subscribe<upo_msgs::PersonPoseArrayUPO>(people_id, 1, &Planner::peopleReceived,this);

	ros::Rate r(freq);
	model::PathProvider pathProvider;

	simulator = new model::Simulator(discount,cell_size,goals,pathProvider,tracking_range,goal_radius,running_time);
	pomcp::PomcpPlanner<model::State,model::Observation,model::Action> planner(*simulator,timeout,threshold,exploration_constant);
	model::Observation obs;
	unsigned action;
	teresa_wsbs::select_mode mode_srv;
	while(n.ok()) {
		if (running && firstOdom && firstPeople) {
			action = planner.getAction();
			mode_srv.request.controller_mode = action;
			controller_mode.call(mode_srv);
			ros::spinOnce();
			getObservation(obs);
			planner.moveTo(action,obs);
		}

		r.sleep();	
		ros::spinOnce();	
	}

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
