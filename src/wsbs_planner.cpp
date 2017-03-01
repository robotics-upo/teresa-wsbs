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
#include <teresa_wsbs/model.hpp>
#include <lightpomcp/Pomcp.hpp>
#include <lightpomcp/Timer.hpp>
#include <lightsfm/cmd_vel.hpp>
#include <teresa_wsbs/start.h>
#include <teresa_wsbs/stop.h>
#include <teresa_wsbs/select_mode.h>
#include <nav_msgs/Odometry.h>
#include <upo_msgs/PersonPoseArrayUPO.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/UInt8.h>
#include <animated_marker_msgs/AnimatedMarker.h>
#include <animated_marker_msgs/AnimatedMarkerArray.h>	

namespace wsbs
{





#define ABORT_CODE 100

const double PERSON_MESH_SCALE = (2.0 / 8.5 * 1.8)*0.9;

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
	void publishPrediction();	
	void publishPrediction(animated_marker_msgs::AnimatedMarkerArray& marker_array, unsigned& counter,
				const utils::Vector2d& goal, double probability, double dt);
	bool stop();
	void publishTree();
	void publishTree(const pomcp::Node<model::State,model::Observation, pomcp::VectorBelief<wsbs::model::State>> *root,
				animated_marker_msgs::AnimatedMarkerArray& marker_array, double alpha, int& counter);
	bool start(teresa_wsbs::start::Request &req, teresa_wsbs::start::Response &res);
	bool stop(teresa_wsbs::stop::Request &req, teresa_wsbs::stop::Response &res);
	void statusReceived(const std_msgs::UInt8::ConstPtr& status);
	void odomReceived(const nav_msgs::Odometry::ConstPtr& odom);
	void peopleReceived(const upo_msgs::PersonPoseArrayUPO::ConstPtr& people);
	void otherPeopleReceived(const upo_msgs::PersonPoseArrayUPO::ConstPtr& people);
	static double getPDF(double x, double y, const utils::Vector2d& m,  double sd0, double sd1, double cov);
	double getTargetLikelihood(double x, double y);
	utils::Vector2d publishGoals(const pomcp::VectorBelief<model::State>& belief);
	model::Observation& getObservation(model::Observation& observation);
	ros::ServiceClient controller_start;
	ros::ServiceClient controller_stop;
	ros::ServiceClient controller_mode;
	ros::Publisher goal_markers_pub;
	ros::Publisher belief_markers_pub;
	ros::Publisher target_pose_pub;
	model::Simulator *simulator;
	GoalProvider *goalProvider_ptr;
	AStarPathProvider *aStar_ptr;
	bool running;
	bool initiated;
	bool target_hidden;
	unsigned targetId;
	pomcp::PomcpPlanner<model::State,model::Observation,ControllerMode> *planner_ptr;
	double robot_cell_size,target_cell_size;
	ros::Publisher predicted_target_pub;
	ros::Publisher predicted_trajectory_pub;
	double likelihood_threshold;
	std::map<utils::Vector2d, double> goals;
	double target_likelihood_threshold;
	void addOtherAgent(double x, double y);
	std::vector<sfm::Agent> otherAgents;
	bool reset;
	double target_lost_timeout;
	utils::Timer target_timer;
};

inline
Planner::Planner(ros::NodeHandle& n, ros::NodeHandle& pn)
: simulator(NULL),
  goalProvider_ptr(NULL),
  running(false),
  initiated(false),
  target_hidden(true),
  targetId(0),
  planner_ptr(NULL),
  reset(true)
{
	double freq, discount;
	std::string path_file;
	double lookahead,naive_goal_time,timeout,threshold,exploration_constant;
	double tracking_range,running_time;
	std::string odom_id, people_id, other_people_id;
	int numParticlesInitialBelief;
	
	int planner_mode;

	pn.param<std::string>("odom_id",odom_id,"/odom");
	pn.param<std::string>("people_id",people_id,"/people/navigation");
	pn.param<std::string>("other_people_id",other_people_id,"/people/navigation");
	pn.param<double>("freq",freq,1.0); //1.0 0.5 2.0
	pn.param<double>("discount",discount,0.9); // 0.75
	pn.param<double>("robot_cell_size",robot_cell_size,1.0);
	pn.param<double>("target_cell_size",target_cell_size,1.0);
	pn.param<std::string>("path_file",path_file,"");
	pn.param<double>("lookahead",lookahead,2.0);
	pn.param<double>("naive_goal_time",naive_goal_time,2.0);
	pn.param<double>("timeout",timeout,1.0); // 1.0 
	pn.param<double>("threshold",threshold,0.6); // 0.01
	pn.param<double>("exploration_constant",exploration_constant,1);
	pn.param<double>("tracking_range",tracking_range,8);
	pn.param<double>("running_time",running_time,1.0); // 1.0 2.0
	pn.param<double>("likelihood_threshold",likelihood_threshold,0.00001  ); //
	pn.param<double>("target_likelihood_threshold",target_likelihood_threshold,0.5);
	pn.param<int>("num_particles",numParticlesInitialBelief,100);
	pn.param<double>("target_lost_timeout",target_lost_timeout,2.0); // The target should be lost this time before doing data association
	pn.param<int>("planner_mode",planner_mode,0); 
	ModelType modelType = (ModelType)planner_mode;

	AStarPathProvider pathProvider(path_file);
	aStar_ptr = &pathProvider;
	
	GoalProvider goalProvider(0.5,100,lookahead,naive_goal_time,1.0,"map",pathProvider,false);
	goalProvider_ptr = &goalProvider;
	ros::ServiceServer start_srv = n.advertiseService("/wsbs/start", &Planner::start,this);
	ros::ServiceServer stop_srv  = n.advertiseService("/wsbs/stop", &Planner::stop,this);
	
	controller_start = n.serviceClient<teresa_wsbs::start>("/wsbs/controller/start");	
	controller_stop = n.serviceClient<teresa_wsbs::stop>("/wsbs/controller/stop");	
	controller_mode = n.serviceClient<teresa_wsbs::select_mode>("/wsbs/select_mode");

	ros::Subscriber controller_status_sub = n.subscribe<std_msgs::UInt8>("/wsbs/status", 1, &Planner::statusReceived,this);

	ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>(odom_id, 1, &Planner::odomReceived,this);
	ros::Subscriber people_sub = n.subscribe<upo_msgs::PersonPoseArrayUPO>(people_id, 1, &Planner::peopleReceived,this);
	//ros::Subscriber other_people_sub = n.subscribe<upo_msgs::PersonPoseArrayUPO>(other_people_id, 1, &Planner::otherPeopleReceived,this);
	goal_markers_pub = pn.advertise<visualization_msgs::MarkerArray>("/wsbs/markers/goals", 1);
	belief_markers_pub = pn.advertise<visualization_msgs::MarkerArray>("/wsbs/markers/belief", 1);
	target_pose_pub = pn.advertise<upo_msgs::PersonPoseUPO>("/wsbs/planner/target",1);	
	//predicted_target_pub = pn.advertise<animated_marker_msgs::AnimatedMarkerArray>("/wsbs/markers/predicted_target", 1);
	predicted_trajectory_pub = pn.advertise<animated_marker_msgs::AnimatedMarkerArray>("/wsbs/markers/predicted_trajectory", 1);
	simulator = new model::Simulator(goalProvider,discount,robot_cell_size,target_cell_size,tracking_range,running_time,modelType);
	pomcp::PomcpPlanner<model::State,model::Observation,ControllerMode> planner(*simulator,timeout*2.0/3.0,timeout*1.0/3.0,threshold,exploration_constant,numParticlesInitialBelief);
	planner_ptr = &planner;
	ros::Rate r(freq);
	TF;
	sfm::MAP;
	unsigned DEFAULT_ACTION;
	if (planner_mode == 0 || planner_mode==2) {
		DEFAULT_ACTION = HEURISTIC;
	} else if (planner_mode == 1) {
		DEFAULT_ACTION = SET_FINAL_GOAL;
	} else {
		DEFAULT_ACTION=RIGHT;
	}
	unsigned action = DEFAULT_ACTION;
	utils::Vector2d likelyGoal;
	teresa_wsbs::select_mode mode_srv;
	unsigned size,depth;
	model::Observation obs;
	while(n.ok()) {
		if (running && initiated) {

			ros::spinOnce();
			
			getObservation(obs);
			std::cout<<obs<<std::endl;

			unsigned action0 = action;
			
			//Move in the planner to the next belief, given a and o. The first time it will generate an initial belief
			reset = planner.moveTo(action0,obs);
			std::cout<<"RESET: "<<reset<<std::endl;

			//Get best action for the belief from the planner. The first time we get the default one
			action = reset ? DEFAULT_ACTION : planner.getAction(false);

			//If there planner returns no action, we perform the default one
			if(action == simulator->getNumActions())
				action = DEFAULT_ACTION;

			//Publish visualization of the current belief
			likelyGoal = publishGoals(planner.getCurrentBelief());

			//Publish visualization of predicted trajectories
			publishPrediction();

			utils::Vector2d target_pos,target_vel;
			if (planner.getCurrentBelief().size()>0) {
				for (auto it = planner.getCurrentBelief().getParticles().begin(); 
					it != planner.getCurrentBelief().getParticles().end(); ++it) {
					target_pos += (it->target_pos);
					target_vel += (it->target_vel);
				}
				target_pos /= planner.getCurrentBelief().size();
				target_vel /= planner.getCurrentBelief().size();
			} else {
				target_pos = simulator->target_pos;
				target_vel = simulator->target_vel;
			}

			
			//publishTree(); 

			//Send command to the controller
			mode_srv.request.controller_mode = action;
			mode_srv.request.target_id = target_hidden ? -1 : targetId;
			double x = likelyGoal.getX();
			double y = likelyGoal.getY();
			mode_srv.request.goal_x = x;
			mode_srv.request.goal_y = y;
			x = target_pos.getX();
			y = target_pos.getY();
			TF.transformPoint(x,y,"map","odom");
			mode_srv.request.target_pos_x = x;
			mode_srv.request.target_pos_y = y;
			mode_srv.request.target_yaw =  target_vel.angle().toRadian();
			//TODO target_yaw should be in ODOM frame, not world frame
			mode_srv.request.target_vel = target_vel.norm();
			controller_mode.call(mode_srv);	
			
			//Plan for the next action
			planner.search();

			planner.computeInfo(size,depth);
			std::cout<<"DEPTH: "<<depth<<" SIZE: "<<size<<std::endl;
			
			r.sleep();							
							
								
		} else {

			ros::spinOnce();
			r.sleep();	
			
			
		}
		
			
	}

}

void Planner::publishPrediction()
{
	animated_marker_msgs::AnimatedMarkerArray marker_array;	
	unsigned counter=0;

	
	
	for (auto it = goals.begin(); it!= goals.end(); ++it) {
		publishPrediction(marker_array, counter, it->first, it->second, 0.5);
	}
	predicted_trajectory_pub.publish(marker_array);

}

void Planner::publishPrediction(animated_marker_msgs::AnimatedMarkerArray& marker_array, unsigned& counter,
				const utils::Vector2d& goal, double probability, double dt)
{
	std::vector<sfm::Agent> agents;
	agents.resize(2 + otherAgents.size());
	agents[0].position = simulator->robot_pos;
	agents[0].velocity = simulator->robot_vel;
	agents[0].desiredVelocity = 0.6;
	agents[0].yaw = simulator->robot_vel.angle();	
	agents[0].groupId = 0;

	agents[1].position = simulator->target_pos;
	agents[1].velocity = simulator->target_vel;
	agents[1].yaw = simulator->target_vel.angle();
	agents[1].desiredVelocity = 0.6;
 	agents[1].groupId = 0;
	sfm::Map *map = &sfm::MAP;
	unsigned publish=0;

	for(unsigned int i=0; i< otherAgents.size();i++)
	{
		agents[2+i] = otherAgents[i];
	}

	
		for (unsigned iteration=0; iteration<200 && (agents[1].position - goal).norm() > 0.5; iteration++) {
		sfm::Goal robotLocalGoal;
		robotLocalGoal.radius = 0.5;
		aStar_ptr->getNextPoint(agents[0].position,goal,robotLocalGoal.center);
		agents[0].goals.clear();		
		agents[0].goals.push_back(robotLocalGoal);

		sfm::Goal targetLocalGoal;
		targetLocalGoal.radius = 0.5;
		aStar_ptr->getNextPoint(agents[1].position,goal,targetLocalGoal.center);
		agents[1].goals.clear();		
		agents[1].goals.push_back(targetLocalGoal);

		sfm::SFM.computeForces(agents,map);
		sfm::SFM.updatePosition(agents,dt);
		
		animated_marker_msgs::AnimatedMarker marker;
       		marker.mesh_use_embedded_materials = true;
		marker.lifetime = ros::Duration(1.0);	
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time::now();
		marker.action = 0;
		
		marker.type = animated_marker_msgs::AnimatedMarker::MESH_RESOURCE;
		marker.mesh_resource = "package://teresa_wsbs/images/animated_walking_man.mesh";
		marker.pose.position.x = agents[1].position.getX();
		marker.pose.position.y = agents[1].position.getY();
		marker.action = 0; 
		marker.scale.x = PERSON_MESH_SCALE;
		marker.scale.y = PERSON_MESH_SCALE;
		marker.scale.z = PERSON_MESH_SCALE;
		marker.color.a = probability*0.5;
		marker.color.r = 0.255;
		marker.color.g = 0.412;
		marker.color.b = 0.882;
	
		
		marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
						M_PI*0.5, 0.0, agents[1].velocity.angle().toRadian()+M_PI*0.5);
  		marker.animation_speed = agents[1].velocity.norm() * 0.7;

		/*
		animated_marker_msgs::AnimatedMarker marker1;
       		marker1.mesh_use_embedded_materials = true;
		marker1.lifetime = ros::Duration(1.0);	
		marker1.header.frame_id = "map";
		marker1.header.stamp = ros::Time::now();
		marker1.action = 0;
		
		marker1.type = animated_marker_msgs::AnimatedMarker::MESH_RESOURCE;
		marker1.mesh_resource = "package://teresa_wsbs/images/full_TERESA.dae";
		marker1.pose.position.x = agents[0].position.getX();
		marker1.pose.position.y = agents[0].position.getY();
		marker1.action = 0; 
		marker1.scale.x = PERSON_MESH_SCALE*0.25;
		marker1.scale.y = PERSON_MESH_SCALE*0.25;
		marker1.scale.z = PERSON_MESH_SCALE*0.25;
		marker1.color.a = probability*0.5;
		marker1.color.r = 0.882;
		marker1.color.g = 0.412;
		marker1.color.b = 0.255;
	
	
		marker1.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
						0.0, 0.0, agents[0].velocity.angle().toRadian()+M_PI*0.5);
  		marker1.animation_speed = agents[0].velocity.norm() * 0.7;
		*/
		publish++;		
		if (publish==3 || (agents[1].position - goal).norm() <= 0.5) {
			marker.id = counter++;
			//marker1.id = counter++;
			marker_array.markers.push_back(marker);
			//marker_array.markers.push_back(marker1);
			publish=0;
		}
		
		
	}

}




inline
Planner::~Planner()
{
	delete simulator;

}


void Planner::publishTree()
{
	const pomcp::Node<model::State,model::Observation, pomcp::VectorBelief<wsbs::model::State>> *root = planner_ptr->getRoot();
	animated_marker_msgs::AnimatedMarkerArray marker_array;	
	int counter = 0;
	publishTree(root,marker_array,1.0,counter);	
	predicted_target_pub.publish(marker_array);
}

void Planner::publishTree(const pomcp::Node<model::State,model::Observation, pomcp::VectorBelief<wsbs::model::State>> *root, 
				animated_marker_msgs::AnimatedMarkerArray& marker_array, double alpha, int& counter)
{
	if (root == NULL || root->belief.size()<20) {
		return;
	}
	utils::Vector2d target_pos,target_vel;
	utils::Vector2d robot_pos,robot_vel;
	for (auto it = root->belief.getParticles().begin(); it != root->belief.getParticles().end(); ++it) {
		target_pos += (it->target_pos);
		target_vel += (it->target_vel);
		robot_pos += (it->robot_pos);
		robot_vel += (it->robot_vel);
	}
	target_pos /= root->belief.size();
	target_vel /= root->belief.size();
	robot_pos /= root->belief.size();
	robot_vel /= root->belief.size();
	animated_marker_msgs::AnimatedMarker marker;
       	marker.mesh_use_embedded_materials = true;
	marker.lifetime = ros::Duration(1.0);	
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();
	marker.action = 0;
	marker.id = counter++;
	marker.type = animated_marker_msgs::AnimatedMarker::MESH_RESOURCE;
	marker.mesh_resource = "package://teresa_wsbs/images/animated_walking_man.mesh";
	marker.pose.position.x = target_pos.getX();
	marker.pose.position.y = target_pos.getY();
	marker.action = 0; 
	marker.scale.x = PERSON_MESH_SCALE;
	marker.scale.y = PERSON_MESH_SCALE;
	marker.scale.z = PERSON_MESH_SCALE;
	marker.color.a = alpha;
	marker.color.r =0.882;
	marker.color.g = 0.412;
	marker.color.b =0.255;
	marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI*0.5, 0.0, target_vel.angle().toRadian()+M_PI*0.5);
  	marker.animation_speed = target_vel.norm() * 0.7;
	marker_array.markers.push_back(marker);
	for (auto it = root->childs.begin(); it!= root->childs.end(); ++it) {
		publishTree(it->second,marker_array,alpha*0.5,counter);
	}
	
	

}


model::Observation& Planner::getObservation(model::Observation& observation)
{
	observation.robot_pos_grid_x = (int)std::round(simulator->robot_pos.getX()/robot_cell_size);
	observation.robot_pos_grid_y = (int)std::round(simulator->robot_pos.getY()/robot_cell_size);
	observation.target_pos_grid_x = (int)std::round(simulator->target_pos.getX()/target_cell_size);
	observation.target_pos_grid_y = (int)std::round(simulator->target_pos.getY()/target_cell_size);
	observation.target_hidden = target_hidden;
	return observation;
}




utils::Vector2d Planner::publishGoals(const pomcp::VectorBelief<model::State>& belief)
{
	utils::Vector2d likelyGoal;
	utils::Vector2d targetPos;
	utils::Vector2d targetVel;	
	int index=0;
	
	goals.clear();
	visualization_msgs::MarkerArray belief_markers; 
	for (auto it = belief.getParticles().cbegin(); it!= belief.getParticles().cend(); ++it) {
		if (goals.count(it->goal)==0) {
			goals[it->goal]  = 1.0/(double)(belief.size());
		} else {
			goals[it->goal] += 1.0/(double)(belief.size());
		}
		targetPos += it->target_pos;
		targetVel += it->target_vel;
		visualization_msgs::Marker marker;
		marker.header.frame_id = "map";
          	marker.header.stamp = ros::Time::now();
		marker.ns = "belief_markers";
		marker.type = visualization_msgs::Marker::ARROW;
		marker.id = index++;
		marker.action = visualization_msgs::Marker::ADD;
		marker.color.a = 1.0;			
		marker.color.r = 1.0;
            	marker.color.g = 0.0;
            	marker.color.b = 0.0;
		marker.lifetime = ros::Duration(0.6);
		marker.scale.x = it->target_vel.norm();
            	marker.scale.y = 0.05;
            	marker.scale.z = 0.05;
		marker.pose.position.x = it->target_pos.getX();
		marker.pose.position.y = it->target_pos.getY();
		marker.pose.position.z = 0;
		marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, it->target_vel.angle().toRadian());	
		belief_markers.markers.push_back(marker);

		visualization_msgs::Marker marker1;
		marker1.header.frame_id = "map";
          	marker1.header.stamp = ros::Time::now();
		marker1.ns = "belief_markers";
		marker1.type = visualization_msgs::Marker::ARROW;
		marker1.id = index++;
		marker1.action = visualization_msgs::Marker::ADD;
		marker1.color.a = 1.0;			
		marker1.color.r = 0.0;
            	marker1.color.g = 1.0;
            	marker1.color.b = 0.0;
		marker1.lifetime = ros::Duration(0.6);
		marker1.scale.x = it->robot_vel.norm();
            	marker1.scale.y = 0.05;
            	marker1.scale.z = 0.05;
		marker1.pose.position.x = it->robot_pos.getX();
		marker1.pose.position.y = it->robot_pos.getY();
		marker1.pose.position.z = 0;
		marker1.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, it->robot_vel.angle().toRadian());	
		belief_markers.markers.push_back(marker1);

			
	}
	belief_markers_pub.publish(belief_markers);	
	if (goals.empty()) {
		for (unsigned i = 0 ; i< goalProvider_ptr->getGoals().size(); i++) {
			goals[goalProvider_ptr->getGoals()[i]] = 1.0 / (double) goalProvider_ptr->getGoals().size(); 
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
	index=0;
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
		running=true;
		initiated=false;
	} 
	return success;
}

bool Planner::stop()
{
	teresa_wsbs::stop srv;
	bool success= controller_stop.call(srv);
	if (planner_ptr!=NULL) {
		planner_ptr->reset();
		reset=true;
	}
	running=false;
	initiated=false;
	return success;
}


bool Planner::stop(teresa_wsbs::stop::Request &req, teresa_wsbs::stop::Response &res)
{
	teresa_wsbs::stop srv;
	srv.request = req;
	bool success= controller_stop.call(srv);
	res = srv.response;
	if (planner_ptr!=NULL) {
		planner_ptr->reset();
		reset=true;
	}
	return success;
}



void Planner::odomReceived(const nav_msgs::Odometry::ConstPtr& odom)
{
	double x,y;
	x = odom->pose.pose.position.x;
	y = odom->pose.pose.position.y;
	double yaw =tf::getYaw(odom->pose.pose.orientation);	
	if (TF.transformPose(x, y, yaw, odom->header.frame_id, "map")) {
		simulator->robot_pos.set(x,y);
		simulator->robot_vel.set(odom->twist.twist.linear.x * std::cos(yaw), odom->twist.twist.linear.x * std::sin(yaw));
	}
}



double Planner::getPDF(double x, double y, const utils::Vector2d& m,  double sd0, double sd1, double cov)
{
	
	double rho = cov/(sd0*sd1);
	double aux = 1 - rho*rho;
	
	double z = ((x-m[0])*(x-m[0]))/(sd0*sd0);
	z -= (2*rho*(x-m[0])*(y-m[1]))/(sd0*sd1);
	z += ((y-m[1])*(y-m[1]))/(sd1*sd1);

	double c = 1 / (6.283185307 * sd0 * sd1 * sqrt(aux));
	double e = -z / (2*aux);
	double p = c*exp(e);
	
	return p;
}


double Planner::getTargetLikelihood(double x, double y) 
{
	if (planner_ptr==NULL) {
		return 0;
	}
	std::cout<<"Belief size: "<<planner_ptr->getCurrentBelief().getParticles().size()<<std::endl;
	double sum=0;
	std::unordered_map<utils::Vector2d,unsigned> positions;
	for (auto it = planner_ptr->getCurrentBelief().getParticles().begin(); it != planner_ptr->getCurrentBelief().getParticles().end(); ++it) {
		positions[it->target_pos]++; // TODO: no es necesario
	}
	
	for (auto it = positions.begin(); it!= positions.end(); ++it) {
		double p = getPDF(x,y,it->first,0.2,0.2/*0.25,0.25*/,0);
		double w = (double)it->second/(double)planner_ptr->getCurrentBelief().getParticles().size();
		sum+=p*w;
	}
	return sum;
}

void Planner::peopleReceived(const upo_msgs::PersonPoseArrayUPO::ConstPtr& people)
{
	if (!running) {
		return;
	}
	target_hidden=true;

	if (!initiated || planner_ptr->getCurrentBelief().getParticles().size()==0) {
		for (unsigned i=0; i< people->personPoses.size(); i++) {
			if (people->personPoses[i].id == targetId) {
				
				double x = people->personPoses[i].position.x;
				double y = people->personPoses[i].position.y;
				double yaw = tf::getYaw(people->personPoses[i].orientation);
				if (TF.transformPose(x, y, yaw, people->personPoses[i].header.frame_id, "map")) {
					simulator->target_pos.set(x,y);
					simulator->target_vel.set(people->personPoses[i].vel * std::cos(yaw), 
								people->personPoses[i].vel * std::sin(yaw));
					target_hidden = false;
					initiated=true;
					break;
				}
			}
		}
	} else {
		double max=0;
		unsigned target_id;
		double target_x, target_id_x=0;
		double target_y, target_id_y=0;
		double target_yaw, target_id_yaw=0;
		double target_vel, target_id_vel=0;
		bool target_id_found=false;
		double target_likelihood=0;
		for (unsigned i=0; i< people->personPoses.size(); i++) {
			double x = people->personPoses[i].position.x;
			double y = people->personPoses[i].position.y;
			double yaw = tf::getYaw(people->personPoses[i].orientation);
			if (TF.transformPose(x, y, yaw, people->personPoses[i].header.frame_id, "map")) {
				double p = getTargetLikelihood(x,y);
				std::cout<<"LIKELIHOOD "<<people->personPoses[i].id<<": "<<p<<std::endl;
				if (people->personPoses[i].id == targetId) {
					target_id_x=x;
					target_id_y=y;
					target_id_yaw=yaw;
					target_id_vel= people->personPoses[i].vel;
					target_id_found=true;
					target_likelihood=p;
				}
				if (p>max) {
					target_id = people->personPoses[i].id;
					target_x = x;
					target_y = y;
					target_yaw = yaw;
					target_vel = people->personPoses[i].vel;
					max = p;			
				}
			}
		}
		std::cout<<target_id<<" MAX LIKELIHOOD: "<<max<<std::endl;
		if (target_id_found) {		
			std::cout<<"TARGET LIKELIHOOD: "<<target_likelihood<<std::endl;
			target_timer.reset();
		}
		if (max>likelihood_threshold) {
			if (target_id_found) {
				if (fabs(target_likelihood-max)>target_likelihood_threshold) {
					targetId = target_id;
					simulator->target_pos.set(target_x,target_y);
					simulator->target_vel.set(target_vel * std::cos(target_yaw), target_vel * std::sin(target_yaw));
					target_hidden = false;
				} else {
					simulator->target_pos.set(target_id_x,target_id_y);
					simulator->target_vel.set(target_id_vel * std::cos(target_id_yaw), 
									target_id_vel * std::sin(target_id_yaw));
					target_hidden = false;
				}				
			} else if (target_timer.elapsed()>2.0){
				targetId = target_id;
				simulator->target_pos.set(target_x,target_y);
				simulator->target_vel.set(target_vel * std::cos(target_yaw), target_vel * std::sin(target_yaw));
				target_hidden = false;
			}
		} else if (target_id_found) {
			simulator->target_pos.set(target_id_x,target_id_y);
			simulator->target_vel.set(target_id_vel * std::cos(target_id_yaw), target_id_vel * std::sin(target_id_yaw));
			target_hidden = false;
		}	
	}



}

void Planner::otherPeopleReceived(const upo_msgs::PersonPoseArrayUPO::ConstPtr& people)
{
	if (!running) {
		return;
	}

	simulator->otherAgents.clear();
	this->otherAgents.clear();

	for(int i=0; i< (int)people->personPoses.size() && initiated; i++)
	{

		if(people->personPoses[i].id != targetId)
		{
			double x = people->personPoses[i].position.x;
			double y = people->personPoses[i].position.y;
			double yaw = tf::getYaw(people->personPoses[i].orientation);
			if (TF.transformPose(x, y, yaw, people->personPoses[i].header.frame_id, "map")) {
				simulator->addOtherAgent(x,y);
				addOtherAgent(x,y);
			}
		}

	}


}

void Planner::addOtherAgent(double x, double y)
{
	sfm::Agent a;

	a.position = utils::Vector2d(x,y);
	a.velocity = utils::Vector2d(0.0,0.0);
	a.yaw.setRadian(0.0);
	a.desiredVelocity = 0.0;
	a.groupId = -1;

	otherAgents.push_back(a);

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
