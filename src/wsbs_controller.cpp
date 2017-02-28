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
#include <teresa_wsbs/Info.h>
#include <animated_marker_msgs/AnimatedMarker.h>
#include <animated_marker_msgs/AnimatedMarkerArray.h>	
#include <lightsfm/rosmap.hpp>

namespace wsbs
{

#define ABORT_CODE 100

const double PERSON_MESH_SCALE = (2.0 / 8.5 * 1.8)*0.9;

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
	void pointReceived(const geometry_msgs::PointStamped::ConstPtr& point); 
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
	
	void publishTarget();
	void publishScan();

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
	utils::Vector2d controller_mode_goal;
	
	ros::Publisher robot_markers_pub;
	ros::Publisher target_pub;
	ros::Publisher robot_pub;
	ros::Publisher path_pub;
	ros::Publisher goal_pub;
	ros::Publisher trajectories_pub;
	ros::Publisher status_pub;
	ros::Publisher forces_pub;
	ros::Publisher scan_pub;
	
	ros::ServiceClient leds_client;

	geometry_msgs::Twist zeroTwist;

	bool is_finishing;

	bool use_leds;

	int number_of_leds;

	utils::Vector2d currentGoal;
	utils::Vector2d targetPos;
	utils::Vector2d targetVel;
	utils::Vector2d targetMarkerPos;
	double targetMarkerVel;
	double targetMarkerYaw;
	bool use_estimated_target;
	bool clicked_goals;
	
	bool publish_target;
};


Controller::Controller(ros::NodeHandle& n, ros::NodeHandle& pn)
:  state(WAITING_FOR_START),
   controller_mode(HEURISTIC),
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
	std::string path_file;	
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
	pn.param<double>("people_timeout",people_timeout_threshold,1200.0);
	pn.param<double>("finish_timeout",finish_timeout_threshold,10);
	pn.param<double>("target_lost_timeout",target_lost_timeout_threshold,30);
	pn.param<double>("goal_timeout_threshold",goal_timeout_threshold,40);
	pn.param<bool>("use_leds",use_leds,false);
	pn.param<int>("number_of_leds",number_of_leds,60);
	pn.param<bool>("use_estimated_target",use_estimated_target,false);
	pn.param<bool>("publish_target",publish_target,true);
	pn.param<std::string>("path_file",path_file,"");
	AStarPathProvider pathProvider(path_file);
	
	pn.param<double>("freq",freq,15);
	pn.param<bool>("heuristic_controller",FORCES.getParams().heuristicPlanner, true);
	pn.param<bool>("clicked_goals",clicked_goals,false);

	if (clicked_goals) {
		controller_mode = SET_FINAL_GOAL;
	}
	ros::ServiceServer select_mode_srv;
	std::string start_service_name, stop_service_name;
	if (FORCES.getParams().heuristicPlanner || clicked_goals) {
		start_service_name = "/wsbs/start";
		stop_service_name = "/wsbs/stop";
	} else {
		start_service_name = "/wsbs/controller/start";
		stop_service_name = "/wsbs/controller/stop";
		select_mode_srv = n.advertiseService("/wsbs/select_mode", &Controller::selectMode,this);	
	}

	ros::ServiceServer start_srv = n.advertiseService(start_service_name, &Controller::start,this);
	ros::ServiceServer stop_srv  = n.advertiseService(stop_service_name, &Controller::stop,this);
	ros::ServiceServer select_id_srv  = n.advertiseService("/wsbs/select_target_id", &Controller::selectTargetId,this);	
	
	ros::Subscriber point_sub = n.subscribe<geometry_msgs::PointStamped>("/clicked_point",1,&Controller::pointReceived,this);
	ros::Subscriber people_sub = n.subscribe<upo_msgs::PersonPoseArrayUPO>(people_id, 1, &Controller::peopleReceived,this);
	
	ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>(odom_id, 1, &Controller::odomReceived,this);
	ros::Subscriber laser_sub = n.subscribe<sensor_msgs::LaserScan>(laser_id, 1, &Controller::laserReceived,this);
	scan_pub = pn.advertise<visualization_msgs::MarkerArray>("/wsbs/markers/scan",1);
	ros::Subscriber xtion_sub;
	if (!xtion_id.empty()) {
		xtion_sub = n.subscribe<sensor_msgs::LaserScan>(xtion_id, 1, &Controller::xtionReceived,this);
	}
	if (use_leds) {
		leds_client =  n.serviceClient<teresa_driver::Teresa_leds>("teresa_leds");
	}

	status_pub = pn.advertise<std_msgs::UInt8>("/wsbs/status", 1);
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>(cmd_vel_id, 1);
	robot_markers_pub = pn.advertise<visualization_msgs::MarkerArray>("/wsbs/markers/robot_forces", 1);
	target_pub = pn.advertise<animated_marker_msgs::AnimatedMarkerArray>("/wsbs/markers/target", 1);
	robot_pub = pn.advertise<animated_marker_msgs::AnimatedMarkerArray>("/wsbs/markers/robot", 1);
	forces_pub = pn.advertise<teresa_wsbs::Info>("/wsbs/controller/info", 1);
	path_pub = pn.advertise<visualization_msgs::Marker>("/wsbs/markers/target_path", 1);
	goal_pub = pn.advertise<visualization_msgs::Marker>("/wsbs/markers/local_goal", 1);
	trajectories_pub = pn.advertise<visualization_msgs::MarkerArray>("/wsbs/markers/trajectories", 1);
	
	sfm::MAP;

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
	upo_msgs::PersonPoseArrayUPO arrayAux;
	arrayAux.header.frame_id="/odom";
	upo_msgs::PersonPoseArrayUPO::ConstPtr ptr(&arrayAux);
	ros::Rate r(freq);
	//double dt = 1/freq;
	bool finishing;
	while(n.ok()) {
		checkTimeouts(ros::Time::now());
		if (state == RUNNING || state == TARGET_LOST) {
			if (people_timeout.getTimeElapsed()>=1.0) {
				peopleReceived(ptr);
			}
			FORCES.compute(controller_mode, controller_mode_goal,&pathProvider);
			finishing = CMD_VEL.compute(FORCES.getParams().relaxationTime);
			//finishing = CMD_VEL.compute(dt);
			cmd_vel_pub.publish(CMD_VEL.getCommand());
			checkEndingCondition(finishing);
			publishForces();
			publishPath();
			publishGoal();
			publishTrajectories();
			publishScan();
			publishTarget();			
			if (state == RUNNING) {
				setLeds();
			}
		}
		
		if(!r.sleep())
			ROS_WARN("WSBS controller desired rate not met");
	
		ros::spinOnce();
		publishStatus();	
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


void Controller::pointReceived(const geometry_msgs::PointStamped::ConstPtr& point)
{
	if (clicked_goals) {
		ROS_INFO("CLICKED GOAL received");
		controller_mode_goal.set(point->point.x, point->point.y);
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
	} else if (req.controller_mode == ABORT_CODE) {
		ROS_INFO("Abort code received");
		setState(ABORTED);
		res.error_code = 0;
	} else {
		ROS_INFO("Controller mode is %d",req.controller_mode);
		controller_mode = (ControllerMode)req.controller_mode;
		if (use_estimated_target) {
			ROS_INFO("Likely target ID is %d",req.target_id);
			if (req.target_id >= 0) {
				targetId = req.target_id;
			}
			ROS_INFO("Likely target position is (%f, %f)@odom",req.target_pos_x,req.target_pos_y);
			targetPos.set(req.target_pos_x,req.target_pos_y);
			ROS_INFO("Likely target yaw is (%f)@odom radians",req.target_yaw);
			ROS_INFO("Likely target velocity is %f m/s",req.target_vel);
			utils::Angle yaw = utils::Angle::fromRadian(req.target_yaw);
			targetVel.set(req.target_vel * yaw.cos(),req.target_vel * yaw.sin());
		}
		ROS_INFO("Likely target goal is (%f, %f)@map", req.goal_x, req.goal_y);
		controller_mode_goal.set(req.goal_x,req.goal_y);
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


void Controller::publishTarget()
{
	
	if (publish_target && FORCES.getData().targetFound && (state == RUNNING || state == TARGET_LOST)) {
		animated_marker_msgs::AnimatedMarkerArray marker_array;
		animated_marker_msgs::AnimatedMarker marker;
       		marker.mesh_use_embedded_materials = true;
		marker.lifetime = ros::Duration(1.0);	
		marker.header.frame_id = odom_id;
		marker.header.stamp = ros::Time::now();
		marker.action = FORCES.getData().targetFound?0:2;
		marker.id = 0;
		marker.type = animated_marker_msgs::AnimatedMarker::MESH_RESOURCE;
		marker.mesh_resource = "package://teresa_wsbs/images/animated_walking_man.mesh";
		marker.pose.position.x = targetMarkerPos.getX();
		marker.pose.position.y = targetMarkerPos.getY();
		marker.action = 0; 
		marker.scale.x = PERSON_MESH_SCALE;
		marker.scale.y = PERSON_MESH_SCALE;
		marker.scale.z = PERSON_MESH_SCALE;
		marker.color.a = 1.0;
		marker.color.r = 0.255;
		marker.color.g = 0.412;
		marker.color.b = 0.882;
		marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI*0.5, 0.0, targetMarkerYaw+M_PI*0.5);
  		marker.animation_speed = targetMarkerVel * 0.7;
		marker_array.markers.push_back(marker);	
		target_pub.publish(marker_array);
	}
	// Put the robot
	animated_marker_msgs::AnimatedMarkerArray marker_array1;
	animated_marker_msgs::AnimatedMarker marker1;
       	marker1.mesh_use_embedded_materials = true;
	marker1.lifetime = ros::Duration(1.0);	
	marker1.header.frame_id = odom_id;
	marker1.header.stamp = ros::Time::now();
	marker1.action = 0;
	marker1.id = 1;	
	marker1.type = animated_marker_msgs::AnimatedMarker::MESH_RESOURCE;
	marker1.mesh_resource = "package://teresa_wsbs/images/full_TERESA.dae";
	marker1.pose.position.x = FORCES.getData().robot.position.getX();
	marker1.pose.position.y = FORCES.getData().robot.position.getY();
	marker1.scale.x = PERSON_MESH_SCALE*0.25;
	marker1.scale.y = PERSON_MESH_SCALE*0.25;
	marker1.scale.z = PERSON_MESH_SCALE*0.25;
	marker1.color.a = 1.0;
	marker1.color.r = 1.0;//0.882;
	marker1.color.g = 1.0;//0.412;
	marker1.color.b = 1.0;//0.255;
	marker1.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
						0.0, 0.0, FORCES.getData().robot.yaw.toRadian()-M_PI*0.5);
  	marker1.animation_speed = FORCES.getData().robot.velocity.norm() * 0.7;
	

	
	marker_array1.markers.push_back(marker1);
	robot_pub.publish(marker_array1);
	
	
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
		publishTarget();
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
	marker.lifetime = ros::Duration(1.0);
	marker.scale.x = std::max(1e-4,force.norm());
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.pose.position.x = FORCES.getData().robot.position.getX();
	marker.pose.position.y = FORCES.getData().robot.position.getY();
	marker.pose.position.z = 0;
	marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,force.angle().toRadian());
	markers.markers.push_back(marker);
}


void Controller::publishScan()
{
	visualization_msgs::MarkerArray markers;
	for (unsigned i = 0; i< FORCES.getData().obstacles.size(); i++) {
		visualization_msgs::Marker marker;
		marker.header.frame_id = "base_link";
          	marker.header.stamp = ros::Time::now();
		marker.ns = "scan_markers";
		marker.type = visualization_msgs::Marker::CUBE;
		marker.id = i;
		marker.action = visualization_msgs::Marker::ADD;
		marker.color.a = 1.0;			
		marker.color.r = 0.0;
            	marker.color.g = 1.0;
            	marker.color.b = 0.0;
		marker.lifetime = ros::Duration(0.1);
		marker.scale.x = 0.1;
            	marker.scale.y = 0.1;
            	marker.scale.z = 0.1;
		marker.pose.position.x = FORCES.getData().obstacles[i].center.getX();
		marker.pose.position.y = FORCES.getData().obstacles[i].center.getY();
		marker.pose.position.z = 0;
		markers.markers.push_back(marker);
	}	
	scan_pub.publish(markers);

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
	robot_markers_pub.publish(markers);


	teresa_wsbs::Info forces;
	forces.header.frame_id = "/odom";
	forces.header.stamp = ros::Time::now();

	forces.status = (uint8_t)state;
	forces.mode = (uint8_t)controller_mode;
	forces.target_detected = FORCES.getData().targetFound;

	forces.target_pose.x = FORCES.getData().target.position.getX();
	forces.target_pose.y = FORCES.getData().target.position.getY();
	forces.target_pose.theta = FORCES.getData().target.yaw.toRadian();

	forces.robot_pose.x = FORCES.getData().robot.position.getX();
	forces.robot_pose.y = FORCES.getData().robot.position.getY();
	forces.robot_pose.theta = FORCES.getData().robot.yaw.toRadian();

	forces.target_lin_vel = FORCES.getData().target.linearVelocity;
	
	forces.robot_lin_vel = FORCES.getData().robot.linearVelocity;
	forces.robot_ang_vel = FORCES.getData().robot.angularVelocity;


	forces.robot_local_goal.x = FORCES.getData().goal.getX();
	forces.robot_local_goal.y = FORCES.getData().goal.getY();
	forces.robot_antimove =  FORCES.getData().validGoal;

	utils::Vector2d vis, att, rep, group;

	FORCES.computeTargetGroupForces(vis, att, rep);

	group = vis + att + rep;

	forces.target_group_force.x =  group.getX();
	forces.target_group_force.y =  group.getY();	
	forces.target_group_vis_force.x = vis.getX();
	forces.target_group_vis_force.y = vis.getY();
	forces.target_group_att_force.x = att.getX();
	forces.target_group_att_force.y = att.getY();
	forces.target_group_rep_force.x = rep.getX();
	forces.target_group_rep_force.y = rep.getY();


	forces.robot_global_force.x = FORCES.getData().globalForce.getX();
	forces.robot_global_force.y = FORCES.getData().globalForce.getY();
	forces.robot_desired_force.x = FORCES.getData().desiredForce.getX();
	forces.robot_desired_force.y = FORCES.getData().desiredForce.getY();
	forces.robot_obstacle_force.x = FORCES.getData().obstacleForce.getX();
	forces.robot_obstacle_force.y = FORCES.getData().obstacleForce.getY();
	forces.robot_social_force.x = FORCES.getData().socialForce.getX();
	forces.robot_social_force.y = FORCES.getData().socialForce.getY();
	forces.robot_group_force.x =  FORCES.getData().groupForce.getX();
	forces.robot_group_force.y =  FORCES.getData().groupForce.getY();	
	forces.robot_group_vis_force.x = FORCES.getData().groupGazeForce.getX();
	forces.robot_group_vis_force.y = FORCES.getData().groupGazeForce.getY();
	forces.robot_group_att_force.x = FORCES.getData().groupCoherenceForce.getX();
	forces.robot_group_att_force.y = FORCES.getData().groupCoherenceForce.getY();
	forces.robot_group_rep_force.x = FORCES.getData().groupRepulsionForce.getX();
	forces.robot_group_rep_force.y = FORCES.getData().groupRepulsionForce.getY();
	forces.robot_vref.x = FORCES.getData().velocityRef.getX();
	forces.robot_vref.y = FORCES.getData().velocityRef.getY();

	forces_pub.publish(forces);
}


void Controller::publishPath()
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/odom";
	marker.header.stamp = ros::Time::now();
	marker.ns = "target_path";
	marker.type = 4;
	marker.id = 0;
	marker.lifetime = ros::Duration(1.0);
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
	marker.lifetime = ros::Duration(1.0);
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
	if (FORCES.setPeople(people, targetId, false, targetPos, targetVel)) {
		targetMarkerPos = FORCES.getData().target.position; 
		targetMarkerVel = FORCES.getData().target.velocity.norm();
		targetMarkerYaw = FORCES.getData().target.yaw.toRadian();
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
