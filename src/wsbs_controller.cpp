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
/** Noe Perez                                                          */
/** Fernando Caballero                                                 */
/** Jesus Capitan                                                      */
/** Luis Merino                                                        */
/**                                                                    */   
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/***********************************************************************/

#include <ros/ros.h>
#include <upo_msgs/PersonPoseArrayUPO.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt8.h>
#include <wsbs/start.h>
#include <wsbs/stop.h>
#include <wsbs/vector2d.hpp>
#include <wsbs/robot_forces.hpp>
#include <cstdint>
#include <cmath>
#include <tf/transform_listener.h>



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
		WAITING_FOR_START  = 0, 
		WAITING_FOR_ODOM   = 1, 
		WAITING_FOR_LASER  = 2, 
		WAITING_FOR_XTION  = 3,
		WAITING_FOR_PEOPLE = 4, 
		WAITING_FOR_GOALS  = 5, 
		RUNNING            = 6, 
		TARGET_LOST        = 7,
		FINISHED           = 8
	};

	// Service error codes
	enum ErrorCode {
		SUCCESS                     = 0,
		STATE_IS_WAITING_FOR_START  = 1, 
		STATE_IS_WAITING_FOR_ODOM   = 2, 
		STATE_IS_WAITING_FOR_LASER  = 3, 
		STATE_IS_WAITING_FOR_XTION  = 4,
		STATE_IS_WAITING_FOR_PEOPLE = 5, 
		STATE_IS_WAITING_FOR_GOALS  = 6, 
		STATE_IS_RUNNING            = 7, 
		STATE_IS_TARGET_LOST        = 8,
		STATE_IS_FINISHED           = 9
	};

	Controller(ros::NodeHandle& n, ros::NodeHandle& pn);
	~Controller() {}


private:



	bool start(wsbs::start::Request &req, wsbs::start::Response &res);
	bool stop(wsbs::stop::Request &req, wsbs::stop::Response &res);

	void peopleReceived(const upo_msgs::PersonPoseArrayUPO::ConstPtr& people);
	void odomReceived(const nav_msgs::Odometry::ConstPtr& odom);
	void laserReceived(const sensor_msgs::LaserScan::ConstPtr& laser);
	void xtionReceived(const sensor_msgs::LaserScan::ConstPtr& xtion);

	void checkTimeouts(const ros::Time& current_time);
	
	
	void setState(const State& state);
	static const char *getStateId(const State& state);

	std::string laser_id;
	std::string xtion_id;
	std::string odom_id; 
	std::string people_id; 
	std::string goals_id;

	Timeout odom_timeout;
	Timeout laser_timeout;
	Timeout xtion_timeout;
	Timeout people_timeout;
	Timeout goals_timeout;
	
	
	bool use_xtion;

	State state;

	RobotForces forces;

	
		

};

Controller::Controller(ros::NodeHandle& n, ros::NodeHandle& pn)
: state(WAITING_FOR_START),
  forces(pn)
{
	int target_goals_policy_int, target_lost_policy_int;

	std::string cmd_vel_id;

	double freq;
	double odom_timeout_threshold;
	double laser_timeout_threshold;
	double xtion_timeout_threshold;
	double people_timeout_threshold;
	double goals_timeout_threshold;
	
	

	pn.param<std::string>("laser_id",laser_id,"/scan360");
	pn.param<std::string>("xtion_id",xtion_id,"/depthcamscan_node/scanXtion");
	pn.param<std::string>("cmd_vel_id",cmd_vel_id,"/cmd_vel");
	pn.param<std::string>("odom_id",odom_id,"/odom");
	pn.param<std::string>("people_id",people_id,"/people/navigation");
	pn.param<std::string>("goals_id",goals_id,"/people/goals");
	
	   
	pn.param<double>("robot_max_lin_vel",forces.robot_max_lin_vel,0.3);          
	pn.param<double>("robot_max_ang_vel",forces.robot_max_ang_vel,0.75);   
	pn.param<double>("robot_lin_vel_inc",forces.robot_lin_vel_inc,0.15);        
	pn.param<double>("robot_ang_vel_inc",forces.robot_ang_vel_inc,0.15);     
	pn.param<double>("robot_max_lin_acc",forces.robot_max_lin_acc,1.0);
	pn.param<double>("robot_max_ang_acc",forces.robot_max_ang_acc,1.0);

	
	pn.param<double>("robot_radius",forces.robot_radius,0.35);
	pn.param<double>("collision_threshold",forces.collision_threshold,0.35);
	pn.param<double>("person_radius",forces.person_radius,0.35);
			
	pn.param<int>("target_goals_policy",target_goals_policy_int,NAIVE_GOALS);
	if (target_goals_policy_int<0 || target_goals_policy_int>2) {
		ROS_ERROR("Invalid target goals policy");
		ROS_BREAK();
	}
	forces.target_goals_policy = (TargetGoalsPolicy)target_goals_policy_int;
	pn.param<int>("target_lost_policy",target_lost_policy_int,WALK_TO_LAST_TARGET_POSITION);
	if (target_lost_policy_int<0 || target_lost_policy_int>2) {
		ROS_ERROR("Invalid target lost policy");
		ROS_BREAK();
	}
	forces.target_lost_policy = (TargetLostPolicy)target_lost_policy_int;

	pn.param<bool>("use_xtion",use_xtion,true);
	
	
	pn.param<double>("odom_timeout",odom_timeout_threshold,0.5);
	pn.param<double>("laser_timeout",laser_timeout_threshold,0.5);
	pn.param<double>("xtion_timeout",xtion_timeout_threshold,0.5);
	pn.param<double>("people_timeout",people_timeout_threshold,2.0);
	pn.param<double>("goals_timeout",goals_timeout_threshold,5.0);	
	pn.param<double>("finish_timeout",forces.finish_timeout,20);

	pn.param<double>("force_sigma_obstacle",forces.force_sigma_obstacle,0.1);
	pn.param<double>("force_factor_desired",forces.force_factor_desired,1.0);
	pn.param<double>("force_factor_obstacle",forces.force_factor_obstacle,20);
	pn.param<double>("force_factor_social",forces.force_factor_social,2.1);
	pn.param<double>("force_factor_group_gaze",forces.force_factor_group_gaze,3.0);
	pn.param<double>("force_factor_group_coherence",forces.force_factor_group_coherence,2.0);
	pn.param<double>("force_factor_group_repulsion",forces.force_factor_group_repulsion,1.0);
	pn.param<double>("relaxation_time",forces.relaxation_time,0.5);
	pn.param<double>("lambda",forces.lambda,2.0);
	pn.param<double>("gamma",forces.gamma,0.35);
	pn.param<double>("n_prime",forces.n_prime,3.0);
	pn.param<double>("n",forces.n,2.0);
	pn.param<double>("naive_goals_time",forces.naive_goals_time,5.0);

	pn.param<double>("freq",freq,15);

	ros::ServiceServer start_srv = n.advertiseService("/wsbs/start", &Controller::start,this);
	ros::ServiceServer stop_srv  = n.advertiseService("/wsbs/stop", &Controller::stop,this);	

	ros::Subscriber people_sub = n.subscribe<upo_msgs::PersonPoseArrayUPO>(people_id, 1, &Controller::peopleReceived,this);
	ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>(odom_id, 1, &Controller::odomReceived,this);
	ros::Subscriber laser_sub = n.subscribe<sensor_msgs::LaserScan>(laser_id, 1, &Controller::laserReceived,this);
	ros::Subscriber xtion_sub;
	if (use_xtion) {
		xtion_sub = n.subscribe<sensor_msgs::LaserScan>(xtion_id, 1, &Controller::xtionReceived,this);
	}

	ros::Publisher status_pub = pn.advertise<std_msgs::UInt8>("/wsbs/status", 1);
	forces.cmd_vel_pub = n.advertise<geometry_msgs::Twist>(cmd_vel_id, 1);
		
	ros::Rate r(freq);
	ros::Time current_time = ros::Time::now();

	odom_timeout.setId(odom_id);
	odom_timeout.setTimeout(odom_timeout_threshold);
	odom_timeout.setTime(current_time);

	laser_timeout.setId(laser_id);
	laser_timeout.setTimeout(laser_timeout_threshold);
	laser_timeout.setTime(current_time);

	xtion_timeout.setId(xtion_id);
	xtion_timeout.setTimeout(xtion_timeout_threshold);
	xtion_timeout.setTime(current_time);

	people_timeout.setId(people_id);
	people_timeout.setTimeout(people_timeout_threshold);
	people_timeout.setTime(current_time);

	goals_timeout.setId(goals_id);
	goals_timeout.setTimeout(goals_timeout_threshold);
	goals_timeout.setTime(current_time);

	
	double dt = 1/freq;
	std_msgs::UInt8 status_msg;
	while(n.ok()) {
		current_time = ros::Time::now();
		checkTimeouts(current_time);
		if (state == RUNNING || state == TARGET_LOST) {
			if (forces.generateCmdVel(dt)) {
				setState(FINISHED);
			
			}
			forces.publishMarkers();
		}
		status_msg.data = (uint8_t)state;
		status_pub.publish(status_msg);
		r.sleep();	
		ros::spinOnce();	
	}
}


bool Controller::start(wsbs::start::Request &req, wsbs::start::Response &res) 
{
	ROS_INFO("START received");
	if (state != WAITING_FOR_START && state != FINISHED) {
		ROS_ERROR("Error: State is %s",getStateId(state));
		res.error_code = state+1;
	} else {
		ROS_INFO("Target ID is %d",req.target_id);
		if (req.goal_radius<0) {
			ROS_INFO("The mission finishes when the target stops for %.02f seconds",forces.finish_timeout);
		} else {
			ROS_INFO("The mission finishes when the target and the robot are closer than %.02f meters from position (%.02f, %.02f)@Odom", req.goal_radius,req.goal_x,req.goal_y);
		}
		forces.target_id = req.target_id;
		forces.goalPosition.set(req.goal_x,req.goal_y);
		forces.goalR = req.goal_radius;
		setState(WAITING_FOR_ODOM);
		res.error_code = 0;
	}
	return true;
}


bool Controller::stop(wsbs::stop::Request &req, wsbs::stop::Response &res)
{
	ROS_INFO("STOP received");
	if (state == WAITING_FOR_START) {
		ROS_ERROR("Error: State is %s",getStateId(state));
		res.error_code = state+1;
	} else {
		setState(WAITING_FOR_START);
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
				    "WAITING_FOR_GOALS",
				    "RUNNING",
				    "TARGET_LOST",
				    "FINISHED"};

	return ids[state].c_str();
}


void Controller::setState(const State& state)
{
	if (state == Controller::state) {
		return;
	}
		
	if ( (Controller::state == RUNNING && state != TARGET_LOST) ||
		(Controller::state == TARGET_LOST && state != RUNNING)) {
		forces.stopRobot();
		forces.resetForces();
		forces.publishMarkers();
	}	
	ROS_INFO("State is %s",getStateId(state));
	Controller::state = state;

}



void Controller::checkTimeouts(const ros::Time& current_time)
{
	if (state == WAITING_FOR_START || state == FINISHED) {
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
	
	if (use_xtion && 
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

	if (forces.target_goals_policy == PEOPLE_GOALS && 
		state!= WAITING_FOR_ODOM && 
		state!= WAITING_FOR_LASER && 
		state!= WAITING_FOR_XTION && 
		state!= WAITING_FOR_PEOPLE && 
		state!= WAITING_FOR_GOALS && 
		goals_timeout.check(current_time)) {
		setState(WAITING_FOR_GOALS);
		return;
	}	

}

void Controller::odomReceived(const nav_msgs::Odometry::ConstPtr& odom)
{
	if (state == WAITING_FOR_START || state == FINISHED) {
		return;
	}
	if (forces.setRobot(odom)) {
		odom_timeout.setTime(ros::Time::now());
		if (state == WAITING_FOR_ODOM) {
			setState(WAITING_FOR_LASER);
		}
	} 
}




void Controller::laserReceived(const sensor_msgs::LaserScan::ConstPtr& laser)
{
	if (state == WAITING_FOR_START || state == FINISHED) {
		return;
	}
	if (forces.setLaser(laser)) {
		laser_timeout.setTime(ros::Time::now());
		if (state == WAITING_FOR_LASER) {
			setState(use_xtion ? WAITING_FOR_XTION : WAITING_FOR_PEOPLE);
		}
	}
}


void Controller::xtionReceived(const sensor_msgs::LaserScan::ConstPtr& xtion)
{
	if (state == WAITING_FOR_START || state == FINISHED) {
		return;
	}
	if (forces.setXtion(xtion)) {
		xtion_timeout.setTime(ros::Time::now());
		if (state == WAITING_FOR_XTION) {
			setState(WAITING_FOR_PEOPLE);
		}
	}
}

void Controller::peopleReceived(const upo_msgs::PersonPoseArrayUPO::ConstPtr& people)
{
	if (state == WAITING_FOR_START || state == FINISHED) {
		return;
	}
	if (forces.setPeople(people)) {
		people_timeout.setTime(ros::Time::now());
		if (state == RUNNING && !forces.isTarget()) {
			setState(TARGET_LOST);
		} else if (state == TARGET_LOST && forces.isTarget()) {
			setState(RUNNING);
		} else	if (state == WAITING_FOR_PEOPLE) {
			setState(forces.target_goals_policy == PEOPLE_GOALS ? WAITING_FOR_GOALS : (forces.isTarget() ? RUNNING : TARGET_LOST));
		}
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
