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
#include <teresa_wsbs/select_target_id.h>
#include <teresa_wsbs/forces.hpp>
#include <teresa_wsbs/cmd_vel.hpp>
#include <teresa_driver/Teresa_leds.h>
#include <upo_msgs/PersonPoseArrayUPO.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt8.h>


namespace wsbs
{





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

	bool start(teresa_wsbs::start::Request &req, teresa_wsbs::start::Response &res);
	bool stop(teresa_wsbs::stop::Request &req, teresa_wsbs::stop::Response &res);
	bool selectTargetId(teresa_wsbs::select_target_id::Request &req, teresa_wsbs::select_target_id::Response &res);
	bool selectMode(teresa_wsbs::select_mode::Request &req, teresa_wsbs::select_mode::Response &res);

	void odomReceived(const nav_msgs::Odometry::ConstPtr& odom);
	void laserReceived(const sensor_msgs::LaserScan::ConstPtr& laser);
	void xtionReceived(const sensor_msgs::LaserScan::ConstPtr& xtion);
	void peopleReceived(const upo_msgs::PersonPoseArrayUPO::ConstPtr& people);
	void publishForceMarker(unsigned index, const std_msgs::ColorRGBA& color, 
					const utils::Vector2d& force, visualization_msgs::MarkerArray& markers);

	void publishTrajectories();
	void stopRobot();
	void setState(const State& state);
	static const char *getStateId(const State& state);

	void setLeds();
	

	static std_msgs::ColorRGBA getColor(double r, double g, double b, double a);

	void publishForces();
	void publishPath();
	void publishGoal();
	void publishStatus();
	void checkEndingCondition(bool finishing);
	void checkTimeouts(const ros::Time& current_time);

	State state;

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
	
	ros::Publisher cmd_vel_pub;
	
	unsigned targetId;
	ControllerMode controller_mode;
	
	ros::Publisher markers_pub;
	ros::Publisher path_pub;
	ros::Publisher goal_pub;
	ros::Publisher trajectories_pub;
	ros::Publisher status_pub;

	ros::ServiceClient leds_client;

	geometry_msgs::Twist zeroTwist;

	bool is_finishing;

	bool use_leds;

	int number_of_leds;

	utils::Vector2d currentGoal;
};


Controller::Controller(ros::NodeHandle& n, ros::NodeHandle& pn)
:  state(WAITING_FOR_START),
   controller_mode(RIGHT),
   is_finishing(false)
{

	double odom_timeout_threshold;
	double laser_timeout_threshold;
	double xtion_timeout_threshold;
	double people_timeout_threshold;
	double finish_timeout_threshold;
	double target_lost_timeout_threshold;
	double goal_timeout_threshold;
	double freq;

	zeroTwist.linear.x = 0;
	zeroTwist.linear.y = 0;
	zeroTwist.linear.z = 0;
	zeroTwist.angular.x = 0;
	zeroTwist.angular.y = 0;
	zeroTwist.angular.z = 0;

	pn.param<std::string>("odom_id",odom_id,"/odom");
	pn.param<std::string>("laser_id",laser_id,"/scan360");
	pn.param<std::string>("xtion_id",xtion_id,"/depthcamscan_node/scanXtion");
	pn.param<std::string>("people_id",people_id,"/people/navigation");
	pn.param<std::string>("cmd_vel_id",cmd_vel_id,"/cmd_vel");
	
	pn.param<double>("robot_radius",FORCES.getParams().robotRadius,0.35);
	pn.param<double>("person_radius",FORCES.getParams().personRadius,0.35);

	pn.param<double>("odom_timeout",odom_timeout_threshold,0.5);
	pn.param<double>("laser_timeout",laser_timeout_threshold,0.5);
	pn.param<double>("xtion_timeout",xtion_timeout_threshold,0.5);
	pn.param<double>("people_timeout",people_timeout_threshold,2.0);
	pn.param<double>("finish_timeout",finish_timeout_threshold,20);
	pn.param<double>("target_lost_timeout",target_lost_timeout_threshold,20);
	pn.param<double>("goal_timeout_threshold",goal_timeout_threshold,40);
	pn.param<bool>("use_leds",use_leds,false);
	pn.param<int>("number_of_leds",number_of_leds,60);

	pn.param<double>("freq",freq,15);
	pn.param<bool>("heuristic_planner",FORCES.getParams().heuristicPlanner, true);

	ros::ServiceServer start_srv = n.advertiseService("/wsbs/start", &Controller::start,this);
	ros::ServiceServer stop_srv  = n.advertiseService("/wsbs/stop", &Controller::stop,this);
	ros::ServiceServer select_id_srv  = n.advertiseService("/wsbs/select_target_id", &Controller::selectTargetId,this);	
	ros::ServiceServer select_mode_srv  = n.advertiseService("/wsbs/select_mode", &Controller::selectMode,this);	

	ros::Subscriber people_sub = n.subscribe<upo_msgs::PersonPoseArrayUPO>(people_id, 1, &Controller::peopleReceived,this);
	ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>(odom_id, 1, &Controller::odomReceived,this);
	ros::Subscriber laser_sub = n.subscribe<sensor_msgs::LaserScan>(laser_id, 1, &Controller::laserReceived,this);
	ros::Subscriber xtion_sub;
	if (!xtion_id.empty()) {
		xtion_sub = n.subscribe<sensor_msgs::LaserScan>(xtion_id, 1, &Controller::xtionReceived,this);
	}
	if (use_leds) {
		leds_client =  n.serviceClient<teresa_driver::Teresa_leds>("teresa_leds");
	}

	status_pub = pn.advertise<std_msgs::UInt8>("/wsbs/status", 1);
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>(cmd_vel_id, 1);
	markers_pub = pn.advertise<visualization_msgs::MarkerArray>("/wsbs/markers/robot_forces", 1);
	path_pub = pn.advertise<visualization_msgs::Marker>("/wsbs/markers/target_path", 1);
	goal_pub = pn.advertise<visualization_msgs::Marker>("/wsbs/markers/local_goal", 1);
	trajectories_pub = pn.advertise<visualization_msgs::MarkerArray>("/wsbs/markers/trajectories", 1);

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

	goal_timeout.setId("Goal");
	goal_timeout.setTimeout(goal_timeout_threshold);
	goal_timeout.setTime(ros::Time::now());

	
	ros::Rate r(freq);
	bool finishing;
	while(n.ok()) {
		checkTimeouts(ros::Time::now());
		if (state == RUNNING || state == TARGET_LOST) {
			FORCES.compute(controller_mode);
			finishing = CMD_VEL.compute(FORCES.getParams().relaxationTime);
			cmd_vel_pub.publish(CMD_VEL.getCommand());
			checkEndingCondition(finishing);
			publishForces();
			publishPath();
			publishGoal();
			publishTrajectories();
			if (state == RUNNING) {
				setLeds();
			}
		}
		publishStatus();
		r.sleep();	
		ros::spinOnce();	
	}
}


void Controller::setLeds()
{
	if (!use_leds) {
		return;
	}

	
	teresa_driver::Teresa_leds leds;
	leds.request.rgb_values.resize(number_of_leds*3);
	if (state == RUNNING) {
		for (int i = 0; i< number_of_leds; i++) {
			leds.request.rgb_values[i*3] = 0;
			leds.request.rgb_values[i*3+1] = 0;
			leds.request.rgb_values[i*3+2] = 255;		
		}
		utils::Angle alpha = FORCES.getData().robot.position.angleTo(FORCES.getData().target.position);
		alpha -= FORCES.getData().robot.yaw;
		double alpha_value = alpha.toDegree(utils::Angle::PositiveOnlyRange);
		int index = (int)std::min(number_of_leds-1,(int)std::round( (alpha_value * (double)number_of_leds)/360.0));
		for (int i = index-2; i< index+3; i++) {
			int j = i;
			if (j <0) {
				j += number_of_leds;
			}
			j %= number_of_leds;
			leds.request.rgb_values[j*3] = 255;
			leds.request.rgb_values[j*3+1] = 255;
			leds.request.rgb_values[j*3+2] = 255;
		}		
		
	} else {
		int s = (int)state;
		for (int i = 0; i< number_of_leds; i++) {
			if (i%(s+1) == 0) {
				leds.request.rgb_values[i*3] = 0;
				leds.request.rgb_values[i*3+1] = 0;
				leds.request.rgb_values[i*3+2] = 255;		
			} else {
				leds.request.rgb_values[i*3] = 255;
				leds.request.rgb_values[i*3+1] = 255;
				leds.request.rgb_values[i*3+2] = 255;	
			}
		}
	}
	if (!leds_client.call(leds) || !leds.response.success) {
		ROS_ERROR("Error trying to set robot leds");
	}	
}


void Controller::checkEndingCondition(bool finishing)
{
	if (state == TARGET_LOST &&
		target_lost_timeout.check(ros::Time::now())) {
		setState(ABORTED);
		return;
	}

	
	if (!is_finishing && finishing) {
		finish_timeout.setTime(ros::Time::now());
	}
	
	is_finishing = finishing;

	if (state == RUNNING && is_finishing && finish_timeout.check(ros::Time::now(),false)) {
		is_finishing=false;
		setState(FINISHED);
		return;
	}
	
	if (state == RUNNING && (currentGoal - FORCES.getData().goal).norm()<0.01 &&
		goal_timeout.check(ros::Time::now())) {
			setState(ABORTED);
	} else {
		currentGoal = FORCES.getData().goal;
		goal_timeout.setTime(ros::Time::now());
	}
	
}

void Controller::publishStatus()
{
	std_msgs::UInt8 status_msg;
	status_msg.data = (uint8_t)state;
	status_pub.publish(status_msg);

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


void Controller::publishTrajectories()
{
	for (unsigned i=0; i< CMD_VEL.getMarkers().markers.size(); i++) {
		CMD_VEL.getMarkers().markers[i].action = (state==RUNNING || state==TARGET_LOST)?0:2;
	}
	trajectories_pub.publish(CMD_VEL.getMarkers());

}


void Controller::stopRobot()
{
	cmd_vel_pub.publish(zeroTwist);
}



bool Controller::start(teresa_wsbs::start::Request &req, teresa_wsbs::start::Response &res) 
{
	ROS_INFO("START received");
	if (state != WAITING_FOR_START && state != FINISHED && state != ABORTED) {
		ROS_ERROR("Error: State is %s",getStateId(state));
		res.error_code = state+1;
	} else {
		ROS_INFO("Target ID is %d",req.target_id);
		targetId = req.target_id;
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

bool Controller::selectTargetId(teresa_wsbs::select_target_id::Request &req, teresa_wsbs::select_target_id::Response &res)
{
	ROS_INFO("SELECT_TARGET_ID received");
	if (state == WAITING_FOR_START || state == FINISHED || state == ABORTED) {
		ROS_ERROR("Error: State is %s",getStateId(state));
		res.error_code = state+1;
	} else {
		ROS_INFO("Target ID is %d",req.target_id);
		targetId = req.target_id;
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
		res.error_code = 0;
	}
	return true;
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
		
	if ( (Controller::state == RUNNING && state != TARGET_LOST) ||
		(Controller::state == TARGET_LOST && state != RUNNING)) {
		stopRobot();
		FORCES.reset();
		publishForces();
		publishPath();
		publishGoal();
		publishTrajectories();
	}
	ROS_INFO("State is %s",getStateId(state));
	Controller::state = state;
	setLeds();	
	if (state == TARGET_LOST) {
		target_lost_timeout.setTime(ros::Time::now());
	}
	if (state == RUNNING) {
		goal_timeout.setTime(ros::Time::now());
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
	marker.pose.position.x = FORCES.getData().robot.position.getX();
	marker.pose.position.y = FORCES.getData().robot.position.getY();
	marker.pose.position.z = 0;
	marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,force.angle().toRadian());
	markers.markers.push_back(marker);
}




void Controller::publishForces()
{
	visualization_msgs::MarkerArray markers;
	publishForceMarker(0,getColor(0,0,1,1),FORCES.getData().obstacleForce,markers);
	publishForceMarker(1,getColor(0,1,1,1),FORCES.getData().socialForce,markers);	
	publishForceMarker(2,getColor(0,1,0,1),FORCES.getData().groupForce,markers);	
	publishForceMarker(3,getColor(1,0,0,1),FORCES.getData().desiredForce,markers);	
	//publishForceMarker(4,getColor(1,1,0,1),FORCES.getData().globalForce,markers);	
	publishForceMarker(4,getColor(1,1,0,1),FORCES.getData().velocityRef,markers);	
	markers_pub.publish(markers);
}


void Controller::publishPath()
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/odom";
	marker.header.stamp = ros::Time::now();
	marker.ns = "target_path";
	marker.type = 4;
	marker.id = 0;
	marker.action = FORCES.getData().targetHistory.size()>0  && (state==RUNNING || state==TARGET_LOST)?0:2;
	marker.color = getColor(0,1,0,1);
	marker.scale.x = 0.05;
	
	for (auto it = 	FORCES.getData().targetHistory.begin(); it != FORCES.getData().targetHistory.end(); ++it) {
		geometry_msgs::Point p;
		p.x = it->position.getX();
		p.y = it->position.getY();
		p.z = 0;
		marker.points.push_back(p);
	}
	path_pub.publish(marker);	


}


void Controller::publishGoal()
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/odom";
	marker.header.stamp = ros::Time::now();
	marker.ns = "goal";
	marker.type = 2;
	marker.id = 0;
	marker.action = FORCES.getData().validGoal && (state==RUNNING || state==TARGET_LOST)?0:2;
	marker.color = getColor(1,0,0,1);
	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;
	marker.pose.position.x = FORCES.getData().goal.getX();
	marker.pose.position.y = FORCES.getData().goal.getY();
	marker.pose.position.z = 0;
	

	goal_pub.publish(marker);	


}



void Controller::odomReceived(const nav_msgs::Odometry::ConstPtr& odom)
{
	if (state == WAITING_FOR_START || state == FINISHED || state == ABORTED) {
		return;
	}
	if (FORCES.setRobot(odom)) {
		odom_timeout.setTime(ros::Time::now());
		if (state == WAITING_FOR_ODOM) {
			setState(WAITING_FOR_LASER);
		}
	} 
}


void Controller::laserReceived(const sensor_msgs::LaserScan::ConstPtr& laser)
{
	if (state == WAITING_FOR_START || state == FINISHED || state == ABORTED) {
		return;
	}
	if (FORCES.setLaser(laser)) {
		laser_timeout.setTime(ros::Time::now());
		if (state == WAITING_FOR_LASER) {
			setState(xtion_id.empty() ? WAITING_FOR_PEOPLE : WAITING_FOR_XTION);
		}
	}
}

void Controller::xtionReceived(const sensor_msgs::LaserScan::ConstPtr& xtion)
{
	if (state == WAITING_FOR_START || state == FINISHED || state == ABORTED) {
		return;
	}
	if (FORCES.setXtion(xtion)) {
		xtion_timeout.setTime(ros::Time::now());
		if (state == WAITING_FOR_XTION) {
			setState(WAITING_FOR_PEOPLE);
		}
	}
}

void Controller::peopleReceived(const upo_msgs::PersonPoseArrayUPO::ConstPtr& people)
{
	if (state == WAITING_FOR_START || state == FINISHED || state == ABORTED) {
		return;
	}
	if (FORCES.setPeople(people, targetId)) {
		people_timeout.setTime(ros::Time::now());
		if (state == RUNNING && !FORCES.getData().targetFound) {
			setState(TARGET_LOST);
		} else if (state == TARGET_LOST && FORCES.getData().targetFound) {
			setState(RUNNING);
		} else	if (state == WAITING_FOR_PEOPLE) {
			setState(FORCES.getData().targetFound ? RUNNING : TARGET_LOST );
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
