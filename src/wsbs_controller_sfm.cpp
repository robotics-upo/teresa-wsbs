/***********************************************************************/
/**                                                                    */
/** wsbs_controller.cpp                                                */
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
#include <teresa_wsbs/start.h>
#include <teresa_wsbs/stop.h>
#include <teresa_wsbs/select_mode.h>
#include <teresa_wsbs/common.hpp>
#include <upo_msgs/PersonPoseArrayUPO.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt8.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <lightsfm/sfm.hpp>
#include <lightsfm/cmd_vel.hpp>

namespace wsbs
{

class Timeout
{
public:
	Timeout() :id(""), time(ros::Time::now()), timeout(0) {}
	void setId(const std::string& id) {Timeout::id = id;}
	void setTimeout(double timeout) {Timeout::timeout = timeout;}
	void setTime(const ros::Time& time) {Timeout::time = time;}
	double getTimeout() const {return timeout;}
	bool check(const ros::Time& current, bool isError = true) const
	{
		if ((current - time).toSec() >= timeout) {
			if (isError) {
				ROS_ERROR("%s timeout", id.c_str());
			}
			return true;
		}
		return false;
	}
private:
	std::string id;
	ros::Time time;
	double timeout;
};


class Controller
{
public:
	
	// Controller state
	enum State {
		WAITING_FOR_START    = 0, 
		WAITING_FOR_ODOM     = 1, 
		WAITING_FOR_LASER    = 2, 
		WAITING_FOR_XTION    = 3,
		WAITING_FOR_PEOPLE   = 4,
		RUNNING              = 5, 
		TARGET_LOST          = 6,
		FINISHED             = 7,
		ABORTED		     = 8
	};

	// Service error codes
	enum ErrorCode {
		SUCCESS                     = 0,
		STATE_IS_WAITING_FOR_START  = 1, 
		STATE_IS_WAITING_FOR_ODOM   = 2, 
		STATE_IS_WAITING_FOR_LASER  = 3, 
		STATE_IS_WAITING_FOR_XTION  = 4,
		STATE_IS_WAITING_FOR_PEOPLE = 5, 
		STATE_IS_RUNNING            = 6, 
		STATE_IS_TARGET_LOST        = 7,
		STATE_IS_FINISHED           = 8
	};




	Controller(ros::NodeHandle& n, ros::NodeHandle& pn);
	~Controller() {}

private:
	void odomReceived(const nav_msgs::Odometry::ConstPtr& odom);
	void laserReceived(const sensor_msgs::LaserScan::ConstPtr& scan);
	void xtionReceived(const sensor_msgs::LaserScan::ConstPtr& scan);
	void peopleReceived(const upo_msgs::PersonPoseArrayUPO::ConstPtr& people);
	bool start(teresa_wsbs::start::Request &req, teresa_wsbs::start::Response &res);
	bool stop(teresa_wsbs::stop::Request &req, teresa_wsbs::stop::Response &res);
	bool selectMode(teresa_wsbs::select_mode::Request &req, teresa_wsbs::select_mode::Response &res);
	static const char *getStateId(const State& state);
	void setState(const State& state);
	void checkTimeouts(const ros::Time& current_time);
	static std_msgs::ColorRGBA getColor(double r, double g, double b, double a);
	void publishGoal();
	void publishStatus();
	void publishForceMarker(unsigned index, const std_msgs::ColorRGBA& color, 
					const utils::Vector2d& force, visualization_msgs::MarkerArray& markers);

	void publishForces(double dt);

	State state;
	ControllerMode controller_mode;
	int targetId;
	utils::Vector2d targetPos;
	utils::Vector2d targetVel;
	utils::Vector2d targetGoal;

	std::string cmd_vel_id;
	std::string laser_id;
	std::string xtion_id;
	std::string odom_id; 
	std::string people_id; 

	Timeout odom_timeout;
	Timeout laser_timeout;
	Timeout xtion_timeout;
	Timeout people_timeout;
	Timeout finish_timeout;
	Timeout target_lost_timeout;
	Timeout goal_timeout;
	
	double obstacle_distance_threshold;
	double person_radius;	
	double target_radius;
	double target_velocity;
	double people_velocity;
	double naive_goal_time;
	double goal_radius;
	ros::Publisher status_pub;
	ros::Publisher robot_markers_pub;

	std::vector<sfm::Agent> agents; // agents[0] is the robot
	sfm::Goal robot_local_goal;
	ros::Publisher goal_pub;
	double robot_velocity;

};


Controller::Controller(ros::NodeHandle& n, ros::NodeHandle& pn)
: state(WAITING_FOR_START),
  controller_mode(HEURISTIC),
  targetId(-1)
{
	bool heuristic_controller;

	double odom_timeout_threshold;
	double laser_timeout_threshold;
	double xtion_timeout_threshold;
	double people_timeout_threshold;
	double finish_timeout_threshold;
	double target_lost_timeout_threshold;
	
	double freq;
	double lookahead;
	double robot_max_lin_acc, robot_max_ang_acc;
	double beta_y, beta_d, beta_v;
	std::string path_file;
	
	
	std::string start_service_name, stop_service_name;

	agents.resize(1);

	pn.param<std::string>("odom_id",odom_id,"/odom");
	pn.param<std::string>("laser_id",laser_id,"/scan360");
	pn.param<std::string>("xtion_id",xtion_id,"/depthcamscan_node/scanXtion");
	pn.param<std::string>("people_id",people_id,"/people/navigation");
	pn.param<std::string>("cmd_vel_id",cmd_vel_id,"/cmd_vel");
	pn.param<double>("robot_radius",agents[0].radius,0.35);
	pn.param<double>("person_radius",person_radius,0.35);
	pn.param<double>("target_radius",target_radius,0.35);
	pn.param<double>("target_velocity",target_velocity,0.6);
	pn.param<double>("people_velocity",people_velocity,0.6);
	pn.param<double>("robot_velocity",robot_velocity,0.6);
	pn.param<double>("robot_max_lin_acc",robot_max_lin_acc,1.0);
	pn.param<double>("robot_max_ang_acc",robot_max_ang_acc,1.0);
	pn.param<double>("beta_v",beta_v,0.4);
	pn.param<double>("beta_y",beta_y,0.3);
	pn.param<double>("beta_d",beta_d,0.3);
	agents[0].desiredVelocity = robot_velocity;
	pn.param<double>("odom_timeout",odom_timeout_threshold,0.5);
	pn.param<double>("laser_timeout",laser_timeout_threshold,0.5);
	pn.param<double>("xtion_timeout",xtion_timeout_threshold,0.5);
	pn.param<double>("people_timeout",people_timeout_threshold,2.0);
	pn.param<double>("finish_timeout",finish_timeout_threshold,10);
	pn.param<double>("target_lost_timeout",target_lost_timeout_threshold,200);
	pn.param<bool>("heuristic_controller",heuristic_controller, true);
	pn.param<double>("obstacle_distance_threshold",obstacle_distance_threshold,2.0);
	pn.param<double>("freq",freq,15);
	pn.param<double>("lookahead",lookahead,2.0);
	pn.param<double>("naive_goal_time",naive_goal_time,2.0);
	pn.param<std::string>("path_file",path_file,"");
	pn.param<double>("goal_radius",goal_radius,0.25);	
	status_pub = pn.advertise<std_msgs::UInt8>("/wsbs/status", 1);
	robot_markers_pub = pn.advertise<visualization_msgs::MarkerArray>("/wsbs/markers/robot_forces", 1);
	if (heuristic_controller) {
		start_service_name = "/wsbs/start";
		stop_service_name = "/wsbs/stop";
	} else {
		start_service_name = "/wsbs/controller/start";
		stop_service_name = "/wsbs/controller/stop";
	}
	ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>(cmd_vel_id, 1);
	ros::ServiceServer start_srv = n.advertiseService(start_service_name, &Controller::start,this);
	ros::ServiceServer stop_srv  = n.advertiseService(stop_service_name, &Controller::stop,this);
	ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>(odom_id, 1, &Controller::odomReceived,this);
	ros::Subscriber laser_sub = n.subscribe<sensor_msgs::LaserScan>(laser_id, 1, &Controller::laserReceived,this);
	goal_pub = pn.advertise<visualization_msgs::Marker>("/wsbs/markers/local_goal", 1);
	ros::Subscriber xtion_sub;
	if (!xtion_id.empty()) {
		xtion_sub = n.subscribe<sensor_msgs::LaserScan>(xtion_id, 1, &Controller::xtionReceived,this);
	}
	ros::Subscriber people_sub = n.subscribe<upo_msgs::PersonPoseArrayUPO>(people_id, 1, &Controller::peopleReceived,this);
	odom_timeout.setId(odom_id);
	odom_timeout.setTimeout(odom_timeout_threshold);
	odom_timeout.setTime(ros::Time::now());

	laser_timeout.setId(laser_id);
	laser_timeout.setTimeout(laser_timeout_threshold);
	laser_timeout.setTime(ros::Time::now());

	xtion_timeout.setId(xtion_id);
	xtion_timeout.setTimeout(xtion_timeout_threshold);
	xtion_timeout.setTime(ros::Time::now());

	people_timeout.setId(people_id);
	people_timeout.setTimeout(people_timeout_threshold);
	people_timeout.setTime(ros::Time::now());

	target_lost_timeout.setId("Target lost");
	target_lost_timeout.setTimeout(target_lost_timeout_threshold);
	target_lost_timeout.setTime(ros::Time::now());

	finish_timeout.setId("Finish");
	finish_timeout.setTimeout(finish_timeout_threshold);
	finish_timeout.setTime(ros::Time::now());

	AStarPathProvider pathProvider(path_file);
	GoalProvider goalProvider(0.5,100,lookahead,naive_goal_time,1.0,"odom",pathProvider);
	sfm::CmdVelProvider cmdVelProvider(obstacle_distance_threshold,robot_velocity ,robot_max_lin_acc,robot_max_ang_acc, beta_v,  beta_y,  beta_d);
	ros::ServiceServer select_mode_srv  = n.advertiseService("/wsbs/select_mode", &Controller::selectMode,this);	
	TF;
	ros::Rate r(freq);
	while(n.ok()) {
		checkTimeouts(ros::Time::now());
		if (state == RUNNING || state == TARGET_LOST) {
			goalProvider.update(agents[0].position,targetPos,targetVel,targetGoal);
			agents[0].goals.clear();
			robot_local_goal.center = goalProvider.getRobotLocalGoal(controller_mode);
			robot_local_goal.radius = goal_radius;
			agents[0].goals.push_back(robot_local_goal);
			sfm::SFM.computeForces(agents);
			cmdVelProvider.compute(agents[0],0.5);
			cmd_vel_pub.publish(cmdVelProvider.getCommand());
			publishGoal();
			publishForces(0.5);
			// TODO: Velocity Calc
			// TODO: Publish markers (robot forces, velocity arcs, local goal, target history) 
		}
		publishStatus();
		r.sleep();	
		ros::spinOnce();	
	}
}




void Controller::publishForceMarker(unsigned index, const std_msgs::ColorRGBA& color, 
					const utils::Vector2d& force, 
					visualization_msgs::MarkerArray& markers) 
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/odom";
	marker.header.stamp = ros::Time::now();
	marker.ns = "robot_forces";
	marker.id = index;
	marker.action = force.norm()>1e-4 && (state==RUNNING || state==TARGET_LOST)?0:2;
	marker.color = color;
	marker.scale.x = std::max(1e-4,force.norm());
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.pose.position.x = agents[0].position.getX();
	marker.pose.position.y = agents[0].position.getY();
	marker.pose.position.z = 0;
	marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,force.angle().toRadian());
	markers.markers.push_back(marker);
}




void Controller::publishForces(double dt)
{
	visualization_msgs::MarkerArray markers;
	publishForceMarker(0,getColor(0,0,1,1),agents[0].forces.obstacleForce,markers);
	publishForceMarker(1,getColor(0,1,1,1),agents[0].forces.socialForce,markers);	
	publishForceMarker(2,getColor(0,1,0,1),agents[0].forces.groupForce,markers);	
	publishForceMarker(3,getColor(1,0,0,1),agents[0].forces.desiredForce,markers);	
	//publishForceMarker(4,getColor(1,1,0,1),agents[0].forces.globalForce,markers);	
	utils::Vector2d velocityRef = agents[0].velocity + agents[0].forces.globalForce*dt;
	if (velocityRef.norm() > robot_velocity) {
		velocityRef.normalize();
		velocityRef *= robot_velocity;
	}
	publishForceMarker(4,getColor(1,1,0,1),velocityRef,markers);	
	robot_markers_pub.publish(markers);
}



void Controller::publishStatus()
{
	std_msgs::UInt8 status_msg;
	status_msg.data = (uint8_t)state;
	status_pub.publish(status_msg);

}

std_msgs::ColorRGBA Controller::getColor(double r, double g, double b, double a)
{
	std_msgs::ColorRGBA color;
	color.r = r;
	color.g = g;
	color.b = b;
	color.a = a;
	return color;
}


void Controller::publishGoal()
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/odom";
	marker.header.stamp = ros::Time::now();
	marker.ns = "goal";
	marker.type = 2;
	marker.id = 0;
	marker.action = (state==RUNNING || state==TARGET_LOST)?0:2;
	marker.color = getColor(1,0,0,1);
	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;
	marker.pose.position.x = robot_local_goal.center.getX();
	marker.pose.position.y = robot_local_goal.center.getY();
	marker.pose.position.z = 0;
	

	goal_pub.publish(marker);	


}



void Controller::checkTimeouts(const ros::Time& current_time)
{
	if (state == WAITING_FOR_START || state == FINISHED || state == ABORTED) {
		return;
	}

	if (state!= WAITING_FOR_ODOM && 
		odom_timeout.check(current_time)) {
		setState(WAITING_FOR_ODOM);
		return;
	}	
	
	if (state!= WAITING_FOR_ODOM && 
		state!= WAITING_FOR_LASER && 
		laser_timeout.check(current_time)) {
		setState(WAITING_FOR_LASER);
		return;
	}	
	
	if (!xtion_id.empty() && 
		state!= WAITING_FOR_ODOM && 
		state!= WAITING_FOR_LASER && 
		state!= WAITING_FOR_XTION && 
		xtion_timeout.check(current_time)) {
		setState(WAITING_FOR_XTION);
		return;
	}	

	if (state!= WAITING_FOR_ODOM && 
		state!= WAITING_FOR_LASER && 
		state!= WAITING_FOR_XTION && 
		state!= WAITING_FOR_PEOPLE && 
		people_timeout.check(current_time)) {
		setState(WAITING_FOR_PEOPLE);
		return;
	}	
}



bool Controller::start(teresa_wsbs::start::Request &req, teresa_wsbs::start::Response &res) 
{
	ROS_INFO("START received");
	if (state != WAITING_FOR_START && state != FINISHED && state != ABORTED) {
		ROS_ERROR("Error: State is %s",getStateId(state));
		res.error_code = state+1;
	} else {
		ROS_INFO("Target ID is %d",req.target_id);
		targetId = (int)req.target_id;
		setState(WAITING_FOR_ODOM);
		res.error_code = 0;
	}
	return true;
}


bool Controller::stop(teresa_wsbs::stop::Request &req, teresa_wsbs::stop::Response &res)
{
	ROS_INFO("STOP received");
	if (state == WAITING_FOR_START || state == FINISHED || state == ABORTED) {
		ROS_ERROR("Error: State is %s",getStateId(state));
		res.error_code = state+1;
	} else {
		setState(FINISHED);
		res.error_code = 0;
	}
	return true;
}

bool Controller::selectMode(teresa_wsbs::select_mode::Request &req, teresa_wsbs::select_mode::Response &res)
{
	ROS_INFO("SELECT_MODE received");
	if (state == WAITING_FOR_START || state == FINISHED || state == ABORTED) {
		ROS_ERROR("Error: State is %s",getStateId(state));
		res.error_code = state+1;
	} else {
		ROS_INFO("Controller mode is %d",req.controller_mode);
		controller_mode = (ControllerMode)req.controller_mode;
		ROS_INFO("Likely target ID is %d",req.target_id);
		targetId = req.target_id;
		ROS_INFO("Likely target position is (%f, %f)@odom",req.target_pos_x,req.target_pos_y);
		targetPos.set(req.target_pos_x,req.target_pos_y);
		ROS_INFO("Likely target yaw is (%f)@odom radians",req.target_yaw);
		ROS_INFO("Likely target velocity is %f m/s",req.target_vel);
		utils::Angle yaw = utils::Angle::fromRadian(req.target_yaw);
		targetVel.set(req.target_vel * yaw.cos(),req.target_vel * yaw.sin());
		ROS_INFO("Likely target goal is (%f, %f)@map", req.goal_x, req.goal_y);
		targetGoal.set(req.goal_x,req.goal_y);
		res.error_code = 0;
	}
	return true;
}


const char *Controller::getStateId(const State& state)
{
	static std::string ids[] = {"WAITING_FOR_START",
				    "WAITING_FOR_ODOM",
				    "WAITING_FOR_LASER",
				    "WAITING_FOR_XTION",
				    "WAITING_FOR_PEOPLE",
				    "RUNNING",
				    "TARGET_LOST",
				    "FINISHED",
				    "ABORTED"};

	return ids[state].c_str();
}


void Controller::setState(const State& state)
{
	if (state == Controller::state) {
		return;
	}
	ROS_INFO("State is %s",getStateId(state));
	Controller::state = state;

}

void Controller::odomReceived(const nav_msgs::Odometry::ConstPtr& odom)
{
	if (state == WAITING_FOR_START || state == FINISHED || state == ABORTED) {
		return;
	}
	if (odom->header.frame_id != "/odom" && odom->header.frame_id !="odom") {
		ROS_ERROR("Odometry frame is %s, it should be odom",odom->header.frame_id.c_str()); 
		return;
	}	
	agents[0].position.set(odom->pose.pose.position.x,odom->pose.pose.position.y); 
	agents[0].yaw = utils::Angle::fromRadian(tf::getYaw(odom->pose.pose.orientation));
	agents[0].linearVelocity = odom->twist.twist.linear.x;
	agents[0].angularVelocity = odom->twist.twist.angular.z;	
	agents[0].velocity.set(odom->twist.twist.linear.x * agents[0].yaw.cos(), odom->twist.twist.linear.x * agents[0].yaw.sin());
	odom_timeout.setTime(ros::Time::now());
	if (state == WAITING_FOR_ODOM) {
		setState(WAITING_FOR_LASER);
	}
}

void Controller::laserReceived(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	if (state == WAITING_FOR_START || state == FINISHED || state == ABORTED) {
		return;
	}
	if (scan->header.frame_id != "/base_link" && scan->header.frame_id !="base_link") {
		ROS_ERROR("Laser frame is %s, it should be base_link",scan->header.frame_id.c_str()); 
		return;
	}
	utils::Angle alpha = agents[0].yaw + utils::Angle::fromRadian(scan->angle_min);
	utils::Angle angle_inc = utils::Angle::fromRadian(scan->angle_increment);
	agents[0].obstacles1.clear();
	for (unsigned i=0; i<scan->ranges.size();i++) {
		if (!std::isnan(scan->ranges[i]) && scan->ranges[i]<obstacle_distance_threshold) {
			agents[0].obstacles1.emplace_back(scan->ranges[i]*alpha.cos(),scan->ranges[i]*alpha.sin());
		}
		alpha+=angle_inc;
	}	
	laser_timeout.setTime(ros::Time::now());
	if (state == WAITING_FOR_LASER) {
		setState(xtion_id.empty() ? WAITING_FOR_PEOPLE : WAITING_FOR_XTION);
	}
}


void Controller::xtionReceived(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	if (state == WAITING_FOR_START || state == FINISHED || state == ABORTED) {
		return;
	}
	if (scan->header.frame_id != "/base_link" && scan->header.frame_id !="base_link") {
		ROS_ERROR("Xtion frame is %s, it should be base_link",scan->header.frame_id.c_str()); 
		return;
	}
	utils::Angle alpha = agents[0].yaw + utils::Angle::fromRadian(scan->angle_min);
	utils::Angle angle_inc = utils::Angle::fromRadian(scan->angle_increment);
	agents[0].obstacles2.clear();
	for (unsigned i=0; i<scan->ranges.size();i++) {
		if (!std::isnan(scan->ranges[i]) && scan->ranges[i]<obstacle_distance_threshold) {
			agents[0].obstacles2.emplace_back(scan->ranges[i]*alpha.cos(),scan->ranges[i]*alpha.sin());
		}
		alpha+=angle_inc;
	}	
	xtion_timeout.setTime(ros::Time::now());
	if (state == WAITING_FOR_XTION) {
		setState(WAITING_FOR_PEOPLE);
	}
}

void Controller::peopleReceived(const upo_msgs::PersonPoseArrayUPO::ConstPtr& people)
{
	if (state == WAITING_FOR_START || state == FINISHED || state == ABORTED) {
		return;
	}
	if (people->header.frame_id != "/odom" && people->header.frame_id !="odom") {
		ROS_ERROR("People frame is %s, it should be odom",people->header.frame_id.c_str()); 
		return;
	}
	
	agents[0].groupId = -1;
	agents.resize(people->personPoses.size()+1);

	for (unsigned i=0; i< people->personPoses.size(); i++) {
		agents[i+1].position.set(people->personPoses[i].position.x,people->personPoses[i].position.y);
		agents[i+1].yaw = utils::Angle::fromRadian(tf::getYaw(people->personPoses[i].orientation));
		agents[i+1].velocity.set(people->personPoses[i].vel * agents[i+1].yaw.cos(), people->personPoses[i].vel * agents[i+1].yaw.sin());
		if (fabs(people->personPoses[i].vel) < 0.05) {
			agents[i+1].velocity.set(0,0);
		} 
		agents[i+1].goals.clear();
		sfm::Goal naiveGoal;
		naiveGoal.center =agents[i+1].position + naive_goal_time * agents[i+1].velocity;
		naiveGoal.radius = goal_radius;
		agents[i+1].goals.push_back(naiveGoal);
		if ((int)people->personPoses[i].id == targetId) {
			agents[i+1].radius = target_radius;
			agents[i+1].desiredVelocity = target_velocity;
			agents[i+1].groupId = 0;
			agents[0].groupId = 0;
			targetPos = agents[i+1].position;
			targetVel = agents[i+1].velocity;
		} else {
			agents[i+1].radius = person_radius;
			agents[i+1].desiredVelocity = people_velocity;
			agents[i+1].groupId = -1;
		}
	}
	people_timeout.setTime(ros::Time::now());
	if (state == RUNNING && agents[0].groupId == -1) {
		setState(TARGET_LOST);
	} else if (state == TARGET_LOST && agents[0].groupId != -1) {
		setState(RUNNING);
	} else	if (state == WAITING_FOR_PEOPLE) {
		setState(agents[0].groupId != -1 ? RUNNING : TARGET_LOST);
	}
}



}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "wsbs_controller");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	wsbs::Controller node(n,pn);
	return 0;
}
