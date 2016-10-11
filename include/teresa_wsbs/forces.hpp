/***********************************************************************/
/**                                                                    */
/** forces.hpp                                                         */
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


#ifndef _FORCES_HPP_
#define _FORCES_HPP_

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <upo_msgs/PersonPoseArrayUPO.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <std_msgs/ColorRGBA.h>
#include <tf/transform_listener.h>
#include <vector>
#include <list>
#include <limits>
#include <algorithm>
#include "vector2d.hpp"

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


// Controller Mode
enum ControllerMode {
	LEFT		= 0,
	RIGHT		= 1,
	BEHIND		= 2,
	FOLLOW_PATH	= 3,
	WAIT		= 4
};


struct Obstacle
{
	Obstacle(const utils::Vector2d& center, const utils::Vector2d& left, const utils::Vector2d& right)
		: center(center), left(left), right(right) {}

	utils::Vector2d center;
	utils::Vector2d left;
	utils::Vector2d right;
};


struct Agent
{
	Agent() : linearVelocity(0), angularVelocity(0) {}
	Agent(double linearVelocity, double angularVelocity) :
		linearVelocity(linearVelocity), angularVelocity(angularVelocity) {}
	
	utils::Vector2d position;
	utils::Vector2d velocity;	
	utils::Angle yaw;
	double linearVelocity;
	double angularVelocity;
	void move(double dt);
	

};




class Forces
{
public:

	struct Data
	{
		utils::Vector2d desiredForce;
		utils::Vector2d obstacleForce;
		utils::Vector2d socialForce;
		utils::Vector2d groupGazeForce;
		utils::Vector2d groupCoherenceForce;
		utils::Vector2d groupRepulsionForce;
		utils::Vector2d groupForce;
		utils::Vector2d globalForce;
		utils::Vector2d velocityRef;
	
		std::vector<Obstacle> obstacles;	
		
		utils::Vector2d goal;

		utils::Vector2d leftGoal;
		utils::Vector2d rightGoal;
		utils::Vector2d behindGoal;
		utils::Vector2d followGoal;

		utils::Vector2d desiredDirection;	

		Agent robot;
		Agent target;
		std::list<Agent> targetHistory;
				
		bool targetFound;
		bool pathFound;
		bool validGoal;
		
	};

	struct Parameters
	{
		Parameters()
		: forceFactorDesired(1.0),
		  forceFactorObstacle(10),
		  forceSigmaObstacle(0.2),
		  forceFactorSocial(2.1),  
		  forceFactorGroupGaze(3.0),
		  forceFactorGroupCoherence(2.0),
		  forceFactorGroupRepulsion(1.0),
		  robotRadius(0.4),
		  personRadius(0.4),
		  robotMaxLinearVelocity(0.4),
		  robotMaxAngularVelocity(0.8),
		  robotMaxLinearAcceleration(1.0), 
		  robotMaxAngularAcceleration(1.0), 
		  lambda(2.0),
		  gamma(0.35),
		  n(2.0),
		  nPrime(3.0),
		  relaxationTime(0.5),
		  heuristicPlanner(true),
		  naiveGoalTime(1.0),
		  goalRadius(0.25),
		  obstacleDistanceThreshold(2.0),
		  personVelocityZeroThreshold(0.05),
		  targetLookahead(2.0),
		  beta_v(0.4),
		  beta_d(0.3),
		  beta_y(0.3)
		{}
		double forceFactorDesired;
		double forceFactorObstacle;
		double forceSigmaObstacle;
		double forceFactorSocial;
		double forceFactorGroupGaze;
		double forceFactorGroupCoherence;
		double forceFactorGroupRepulsion;
		double robotRadius;
		double personRadius;
		double robotMaxLinearVelocity;
		double robotMaxAngularVelocity;
		double robotMaxLinearAcceleration;
		double robotMaxAngularAcceleration;
		double lambda;
		double gamma;
		double n;
		double nPrime;
		double relaxationTime;
		bool heuristicPlanner;
		double naiveGoalTime;
		double goalRadius;
		double obstacleDistanceThreshold;
		double personVelocityZeroThreshold;
		double targetLookahead;
		double beta_v;
		double beta_d;
		double beta_y;
	};


	Forces(Forces const&) = delete;
        void operator=(Forces const&) = delete;
	~Forces() {}
	static Forces& getInstance()
   	{
      		static Forces singleton;
      		return singleton;
	}
	#define FORCES Forces::getInstance()
	const Data& getData() const {return data;}
	Parameters& getParams() {return params;}
	const Parameters& getParams() const {return params;}
	void reset();
	void compute(ControllerMode controller_mode);
	bool setRobot(const nav_msgs::Odometry::ConstPtr& odom); 
	bool setLaser(const sensor_msgs::LaserScan::ConstPtr& laser);
	bool setXtion(const sensor_msgs::LaserScan::ConstPtr& xtion);
	bool setPeople(const upo_msgs::PersonPoseArrayUPO::ConstPtr& people, unsigned targetId);
	bool checkCollision(double linearVelocity, double angularVelocity, double& distance, double& time) const;
	

private:

	static bool compareNorm(const Obstacle& i, const Obstacle& j) {return i.center.squaredNorm() < j.center.squaredNorm();}
	
	
	#define PW(x) ((x)*(x))
	Forces();
	void computeObstacleForceSum(const sensor_msgs::LaserScan::ConstPtr& scan, utils::Vector2d& forceSum, unsigned& points, std::vector<Obstacle>& obstacles) const;
	utils::Vector2d computeSocialForce(const utils::Vector2d& position, const utils::Vector2d& velocity) const;
	void computeDesiredForce();
	void selectGoal(ControllerMode controller_mode);
	void computeGroupForce();
	

	Data data;
	Parameters params;
	utils::Vector2d laserForceSum;
	utils::Vector2d xtionForceSum;
	unsigned laserPoints;
	unsigned xtionPoints;
	std::vector<Obstacle> laserObstacles;
	std::vector<Obstacle> xtionObstacles;
	
	
};

inline
Forces::Forces()
{
	reset();
}


inline
void Forces::reset()
{
	data.desiredForce.set(0,0);
	data.obstacleForce.set(0,0);
	data.socialForce.set(0,0);
	data.groupForce.set(0,0);
	data.globalForce.set(0,0);
	data.velocityRef.set(0,0);
	data.goal.set(0,0);
	data.desiredDirection.set(0,0);
	data.targetFound = false;
	data.targetHistory.clear();
	data.pathFound = false;
	data.validGoal = false;
	laserForceSum.set(0,0);
	xtionForceSum.set(0,0);
	laserPoints = 0;
	xtionPoints = 0;
	laserObstacles.clear();
	xtionObstacles.clear();
	
}


inline
bool Forces::checkCollision(double linearVelocity, double angularVelocity, double& distance, double& time) const
{
	// No linear velocity, no collision
	if (fabs(linearVelocity) < 1e-7) {
		time = 999999;
		distance = 999999;
		return false;
	}
	
	// No angular velocity, special case
	if (fabs(angularVelocity) < 1e-7) {
		for (unsigned i = 0; i< data.obstacles.size(); i++) {
			if ((linearVelocity>0 && data.obstacles[i].center.getX()<0) || (linearVelocity<0 && data.obstacles[i].center.getX()>0)) {
				continue;
			}
			if (fabs(data.obstacles[i].center.getY())<= params.robotRadius) {
				distance = fabs(data.obstacles[i].center.getX());
				double offset = std::sqrt(PW(params.robotRadius) - PW(data.obstacles[i].center.getY()));
				distance -= offset;
				if (distance <0 ) {
					distance = 0;
				}
				time = distance / fabs(linearVelocity);
				
				return fabs(linearVelocity) > std::sqrt(2*distance * params.robotMaxLinearAcceleration);

			}		
		}
		
		time = 999999;
		distance = 999999;
		return false;
	} 

	// Angular and linear velocity, general case
	utils::Vector2d icc(0,linearVelocity/angularVelocity);
	for (unsigned i = 0; i< data.obstacles.size(); i++) {
		utils::Vector2d u = (data.obstacles[i].center - icc).leftNormalVector();
		if (linearVelocity >= 0 && angularVelocity < 0) {
			u.set(-u.getX(),u.getY());
		} else if (linearVelocity <0 && angularVelocity >= 0) {
			u.set(-u.getX(),-u.getY());
		} else if (linearVelocity<0 && angularVelocity<0) {
			u.set(u.getX(),-u.getY());
		}
		utils::Angle alpha = u.angle();
		double t = alpha.toRadian(utils::Angle::PositiveOnlyRange)/fabs(angularVelocity);
		Agent dummy(linearVelocity, angularVelocity);
		dummy.move(t);
		if ((dummy.position - data.obstacles[i].center).squaredNorm() <= PW(params.robotRadius)) {
			while ((dummy.position - data.obstacles[i].center).squaredNorm() <= PW(params.robotRadius)) {
				t-=0.01;
				dummy = Agent(linearVelocity, angularVelocity);
				dummy.move(t);
			}
			t+=0.005; 
			time = t;
			if (time <0 ) {
				time = 0;
			}
			distance = fabs(linearVelocity)*time;
			return fabs(linearVelocity) > std::sqrt(2*distance * params.robotMaxLinearAcceleration) || 
					fabs(angularVelocity) > std::sqrt(2*distance * params.robotMaxAngularAcceleration);
		}
	}

	time = 999999;
	distance = 999999;
	return false;


}




inline
void Agent::move(double dt)
{
	double imd = linearVelocity * dt;
	utils::Vector2d inc(imd * std::cos(yaw.toRadian() + angularVelocity*dt*0.5), imd * std::sin(yaw.toRadian() + angularVelocity*dt*0.5));
	yaw += utils::Angle::fromRadian(angularVelocity * dt);	
	position += inc;
	velocity.set(linearVelocity * yaw.cos(), linearVelocity * yaw.sin());
}

inline
void Forces::compute(ControllerMode controller_mode)
{
	if (laserPoints==0 && xtionPoints==0) {
		data.obstacleForce.set(0,0);
		data.obstacles.clear();
	} else {
		data.obstacleForce = (laserForceSum + xtionForceSum)/(double)(laserPoints + xtionPoints); 
		data.obstacles.swap(laserObstacles);
		data.obstacles.insert(data.obstacles.end(), xtionObstacles.begin(), xtionObstacles.end());
		std::sort(data.obstacles.begin(),data.obstacles.end(),compareNorm);
	}
	selectGoal(controller_mode);
	computeDesiredForce();
	computeGroupForce();
	data.globalForce = data.desiredForce + data.obstacleForce + data.socialForce + data.groupForce;
	data.velocityRef = data.robot.velocity + params.relaxationTime * data.globalForce;
	
}


inline
bool Forces::setRobot(const nav_msgs::Odometry::ConstPtr& odom)
{
	if (odom->header.frame_id != "/odom" && odom->header.frame_id !="odom") {
		ROS_ERROR("Odometry frame is %s, it should be odom",odom->header.frame_id.c_str()); 
		return false;
	}
	data.robot.position.set(odom->pose.pose.position.x,odom->pose.pose.position.y); 
	data.robot.yaw = utils::Angle::fromRadian(tf::getYaw(odom->pose.pose.orientation));	
	data.robot.velocity.set(odom->twist.twist.linear.x * data.robot.yaw.cos(), odom->twist.twist.linear.x * data.robot.yaw.sin());
	data.robot.linearVelocity = odom->twist.twist.linear.x;
	data.robot.angularVelocity = odom->twist.twist.angular.z;	
	return true;
}

inline
bool Forces::setLaser(const sensor_msgs::LaserScan::ConstPtr& laser)
{
	if (laser->header.frame_id != "/base_link" && laser->header.frame_id !="base_link") {
		ROS_ERROR("Laser frame is %s, it should be base_link",laser->header.frame_id.c_str()); 
		return false;
	}
	computeObstacleForceSum(laser,laserForceSum,laserPoints,laserObstacles);
	return true;
}

inline
bool Forces::setXtion(const sensor_msgs::LaserScan::ConstPtr& xtion)
{
	if (xtion->header.frame_id != "/base_link" && xtion->header.frame_id !="base_link") {
		ROS_ERROR("Xtion frame is %s, it should be base_link",xtion->header.frame_id.c_str()); 
		return false;
	}
	computeObstacleForceSum(xtion,xtionForceSum,xtionPoints,xtionObstacles);
	return true;
}


inline
utils::Vector2d Forces::computeSocialForce(const utils::Vector2d& position, const utils::Vector2d& velocity) const
{
	utils::Vector2d diff = position - data.robot.position;
	utils::Vector2d diffDirection = diff.normalized();
	utils::Vector2d velDiff = data.robot.velocity - velocity;
	utils::Vector2d interactionVector = params.lambda * velDiff + diffDirection;
	double interactionLength = interactionVector.norm();
	utils::Vector2d interactionDirection = interactionVector/interactionLength;
	utils::Angle theta = interactionDirection.angleTo(diffDirection);
	double B = params.gamma * interactionLength;
	double thetaRad = theta.toRadian();	
	double forceVelocityAmount = -std::exp(-diff.norm()/B - PW(params.nPrime*B*thetaRad) );
	double forceAngleAmount = -theta.sign() * std::exp(-diff.norm() / B - PW(params.n * B * thetaRad));
	utils::Vector2d forceVelocity = forceVelocityAmount * interactionDirection;
	utils::Vector2d forceAngle = forceAngleAmount * interactionDirection.leftNormalVector();
	return forceVelocity + forceAngle;
}




inline
void Forces::computeDesiredForce()
{
	if (data.validGoal) {
		double vel = params.robotMaxLinearVelocity;
		if (data.targetFound) {
			if ((data.robot.position - data.target.position).norm() <= FORCES.getParams().targetLookahead) {
				if (data.target.velocity.norm()>params.personVelocityZeroThreshold) {
					if ((data.goal - data.robot.position).norm()>params.goalRadius) {
						vel = std::min(params.robotMaxLinearVelocity,data.target.velocity.norm()*1.5);
					} else {
						vel = data.target.velocity.norm();
					}
					utils::Vector2d diff = data.goal - data.robot.position;
					data.desiredDirection = diff.normalized();
					data.desiredForce = params.forceFactorDesired * (data.desiredDirection * vel  - data.robot.velocity)/params.relaxationTime;
				} else if ((data.goal - data.robot.position).norm()>params.goalRadius) {
					utils::Vector2d diff = data.goal - data.robot.position;
					data.desiredDirection = diff.normalized();
					data.desiredForce = params.forceFactorDesired * (data.desiredDirection * vel  - data.robot.velocity)/params.relaxationTime;
				} else {
					data.desiredForce = -data.robot.velocity / params.relaxationTime;
					data.desiredDirection.set(0,0);
				}
			} else if ((data.goal - data.robot.position).norm()>params.goalRadius) {
				utils::Vector2d diff = data.goal - data.robot.position;
				data.desiredDirection = diff.normalized();
				data.desiredForce = params.forceFactorDesired * (data.desiredDirection * vel  - data.robot.velocity)/params.relaxationTime;
			} else {
				data.desiredForce = -data.robot.velocity / params.relaxationTime;
				data.desiredDirection.set(0,0);
			}
		} else if ((data.goal - data.robot.position).norm()>params.goalRadius) {
			utils::Vector2d diff = data.goal - data.robot.position;
			data.desiredDirection = diff.normalized();
			data.desiredForce = params.forceFactorDesired * (data.desiredDirection * vel  - data.robot.velocity)/params.relaxationTime;
		} else {
			data.desiredForce = -data.robot.velocity / params.relaxationTime;
			data.desiredDirection.set(0,0);
		}
	} else {
		data.desiredForce = -data.robot.velocity / params.relaxationTime;
		data.desiredDirection.set(0,0);
	}



}

inline
void Forces::computeGroupForce()
{
	data.groupGazeForce.set(0,0);
	data.groupCoherenceForce.set(0,0);
	data.groupRepulsionForce.set(0,0);

	if (!data.targetFound || !data.validGoal || (data.robot.position - data.target.position).norm() > FORCES.getParams().targetLookahead) {
		data.groupForce.set(0,0);
		return;
	}
	

	// Gaze force
	if (data.desiredDirection.norm() > 1e-7) {
		utils::Vector2d relativeCom = data.target.position - data.robot.position;
		utils::Angle visionAngle = utils::Angle::fromDegree(90);
		double elementProduct = data.desiredDirection.dot(relativeCom);
		utils::Angle comAngle = utils::Angle::fromRadian(std::acos(elementProduct / (data.desiredDirection.norm() * relativeCom.norm())));
		if (comAngle > visionAngle) {
			#ifdef _PAPER_VERSION_
			utils::Angle necessaryRotation = comAngle - visionAngle;
			data.groupGazeForce = -necessaryRotation.toRadian() * data.desiredDirection;		
			#else
			double desiredDirectionSquared = data.desiredDirection.squaredNorm();
			double desiredDirectionDistance = elementProduct / desiredDirectionSquared;
			data.groupGazeForce = desiredDirectionDistance * data.desiredDirection;
			#endif
			data.groupGazeForce *= params.forceFactorGroupGaze;
		}
	}
	// Coherence force
	utils::Vector2d com = (data.target.position + data.robot.position)*0.5;
	utils::Vector2d relativeCom = com - data.robot.position;
	double cohDistance = relativeCom.norm();
	#ifdef _PAPER_VERSION_
	if (cohDistance >= 0.5) {
		data.groupCoherenceForce = relativeCom.normalized();
		data.groupCoherenceForce *=  params.forceFactorGroupCoherence;
	} 
	#else
	data.groupCoherenceForce = relativeCom;
	double softenedFactor = params.forceFactorGroupCoherence * (std::tanh(cohDistance - 0.5) + 1) / 2;
	data.groupCoherenceForce *= softenedFactor;
	#endif

	// Repulsion Force
	utils::Vector2d diff = data.robot.position - data.target.position;
	double repDistance = diff.norm();
	if (repDistance < params.robotRadius + params.personRadius) {
		data.groupRepulsionForce += diff;
	}
	data.groupRepulsionForce *= params.forceFactorGroupRepulsion;

	data.groupForce =  data.groupGazeForce +  data.groupCoherenceForce + data.groupRepulsionForce;


	
}

inline
void Forces::selectGoal(ControllerMode controller_mode)
{
	

	// By default, if the goal is not valid, it will perform an antimove
	data.validGoal = false;
	if (params.heuristicPlanner) {
		if (data.targetFound && 
			(data.robot.position - data.target.position).norm() <= FORCES.getParams().targetLookahead) {
			if ((data.rightGoal - data.robot.position).squaredNorm() <
				 (data.leftGoal - data.robot.position).squaredNorm()) {
				data.goal = data.rightGoal;
				data.validGoal = true;
			} else {
				data.goal = data.leftGoal;
				data.validGoal = true;
			}

		} else if (data.pathFound) {
			data.goal = data.followGoal;
			data.validGoal = true;
		}

	} else {
		if (data.targetFound && controller_mode != FOLLOW_PATH) {
			if (controller_mode == LEFT) {
				data.goal = data.leftGoal;
				data.validGoal = true;
			} else if (controller_mode == RIGHT) {
				data.goal = data.rightGoal;
				data.validGoal = true;
			} else if (controller_mode == BEHIND) {
				data.goal = data.behindGoal;
				data.validGoal = true;
			} // If controller mode is WAIT, then validGoal is false

		} else if (data.pathFound && controller_mode == FOLLOW_PATH) {
			data.goal = data.followGoal;
			data.validGoal = true;
		}

	}
	

}


inline
bool Forces::setPeople(const upo_msgs::PersonPoseArrayUPO::ConstPtr& people, unsigned targetId)
{
	if (people->header.frame_id != "/odom" && people->header.frame_id !="odom") {
		ROS_ERROR("People frame is %s, it should be odom",people->header.frame_id.c_str()); 
		return false;
	}
	data.socialForce.set(0,0);
	data.targetFound = false;
	data.pathFound = false;
	
	for (unsigned i=0; i< people->personPoses.size(); i++) {
		utils::Vector2d position(people->personPoses[i].position.x,people->personPoses[i].position.y);
		utils::Angle yaw = utils::Angle::fromRadian(tf::getYaw(people->personPoses[i].orientation));
		utils::Vector2d velocity(people->personPoses[i].vel * yaw.cos(), people->personPoses[i].vel * yaw.sin());
		if (fabs(people->personPoses[i].vel) < params.personVelocityZeroThreshold) {
			velocity.set(0,0);
		} 	
		data.socialForce += params.forceFactorSocial*computeSocialForce(position,velocity);
		if (people->personPoses[i].id == targetId) {
			data.target.position = position;
			data.target.velocity = velocity;
			data.target.yaw = yaw;
			data.target.linearVelocity = people->personPoses[i].vel;
			data.target.angularVelocity = 0;
			data.targetFound = true;
			if (fabs(people->personPoses[i].vel) >= params.personVelocityZeroThreshold) {
				data.leftGoal = 
					data.target.position + params.naiveGoalTime * data.target.velocity +  
						velocity.normalized().leftNormalVector();
				data.rightGoal =
					  data.target.position + params.naiveGoalTime * data.target.velocity +
						  velocity.normalized().rightNormalVector();
				data.behindGoal =
					  data.target.position + params.naiveGoalTime * data.target.velocity - velocity.normalized();
			}


			if (data.targetHistory.empty() || (position - data.targetHistory.front().position).norm() > 0.5) {
				data.targetHistory.push_front(data.target);
				if (data.targetHistory.size()>100) {
					auto it = data.targetHistory.rbegin();
					++it;
					if ((data.robot.position - it->position).norm() <= FORCES.getParams().targetLookahead) {
						data.targetHistory.pop_back();
					} else {
						data.targetHistory.pop_front();
					}
				}
			}
		}
	}
	for (auto it = data.targetHistory.begin(); it!= data.targetHistory.end(); ++it) {
		if ((data.robot.position - it->position).norm() <= FORCES.getParams().targetLookahead) {
			data.pathFound = true;
			data.followGoal = it->position;
			data.targetHistory.erase(++it,data.targetHistory.end());
			break;	
		}
	}
	
		
 
	return true;
}


inline
void Forces::computeObstacleForceSum(const sensor_msgs::LaserScan::ConstPtr& scan, utils::Vector2d& forceSum, unsigned& points, std::vector<Obstacle>& obstacles) const
{
	forceSum.set(0,0);
	obstacles.clear();
	points=0;		
	utils::Angle alpha = data.robot.yaw + utils::Angle::fromRadian(scan->angle_min);
	utils::Angle beta = utils::Angle::fromRadian(scan->angle_min);
	utils::Angle angle_inc = utils::Angle::fromRadian(scan->angle_increment);
	utils::Angle angle_inc_mid = utils::Angle::fromRadian(0.5*scan->angle_increment);
	double c = angle_inc_mid.cos();	
	for (unsigned i=0; i<scan->ranges.size();i++) {
		if (!std::isnan(scan->ranges[i])) {
			utils::Vector2d diff(-scan->ranges[i]*alpha.cos(),-scan->ranges[i]*alpha.sin());
			forceSum += params.forceFactorObstacle * std::exp(-(scan->ranges[i] - params.robotRadius)/params.forceSigmaObstacle) * diff.normalized();
			points++;
			if (scan->ranges[i]<params.obstacleDistanceThreshold) {
				utils::Vector2d center(scan->ranges[i]*beta.cos(),scan->ranges[i]*beta.sin());
				double d = scan->ranges[i] / c;
				utils::Angle gamma = beta + angle_inc_mid;
				utils::Vector2d left(d*gamma.cos(),d*gamma.sin());
				gamma = beta - angle_inc_mid;
				utils::Vector2d right(d*gamma.cos(),d*gamma.sin());
				obstacles.emplace_back(center,left,right);
			}
		}
		alpha+=angle_inc;
		beta+=angle_inc;
	}
}



}

#endif
