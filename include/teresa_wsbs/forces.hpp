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
#include <teresa_wsbs/common.hpp>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_listener.h>
#include <vector>
#include <list>
#include <limits>
#include <algorithm>
#include <lightsfm/vector2d.hpp>
#include <teresa_wsbs/model.hpp>
#include <lightsfm/rosmap.hpp>

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
	const ros::Time& getTime() const {return time;}
	double getTimeElapsed() const {return (ros::Time::now() - time).toSec();}
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
		: forceFactorDesired(3.0), //2.0 
		  forceFactorObstacle(10),  
		  forceSigmaObstacle(0.2),
		  forceFactorSocial(2.1),  
		  forceFactorGroupGaze(3.0),
		  forceFactorGroupCoherence(2.0),
		  forceFactorGroupRepulsion(1.0),
		  robotRadius(0.375), //0.35 Radius of the robot plus 2.5 cms.
		  personRadius(0.35),
		  robotMaxLinearVelocity(0.6), //0.5 0.6 0.5 0.3
		  robotMaxAngularVelocity(0.8), // 0.8 1.5
		  robotMaxLinearAcceleration(1.0), // 0.5 
		  robotMaxAngularAcceleration(2.0), // 1.0 
		  lambda(2.0),
		  gamma(0.35),
		  n(2.0),
		  nPrime(3.0),
		  relaxationTime(0.5), // 0.5
		  heuristicPlanner(true),
		  naiveGoalTime(1.0), // 1.0
		  goalRadius(1.0), //0.25
		  obstacleDistanceThreshold(2.0),
		  personVelocityZeroThreshold(0.05),
		  targetLookahead(2.0), //2.0
		  beta_v(0.4),
		  beta_d(0.3),
		  beta_y(0.3),
		  target_vel_sma_period(15),
                  k_distance_obstacle(-2.0),
		  k_distance_robot(1.0)

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
		unsigned target_vel_sma_period;
		double k_distance_obstacle;
		double k_distance_robot;
	};


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
	void compute(ControllerMode controller_mode, const utils::Vector2d& controller_mode_goal, PathProvider* pathProvider);
	bool setRobot(const nav_msgs::Odometry::ConstPtr& odom); 
	bool setLaser(const sensor_msgs::LaserScan::ConstPtr& laser);
	bool setXtion(const sensor_msgs::LaserScan::ConstPtr& xtion);
	bool setPeople(const upo_msgs::PersonPoseArrayUPO::ConstPtr& people, unsigned targetId, bool targetReceived, const utils::Vector2d& targetPos, const utils::Vector2d& targetVel);
	bool checkCollision(double linearVelocity, double angularVelocity, double& distance, double& time) const;
	void computeTargetGroupForces(utils::Vector2d& vis, utils::Vector2d& att, utils::Vector2d& rep) const;
private:
	bool distanceToObstacle(const utils::Vector2d& point, double& distance, const std::string& frame);
	static bool compareNorm(const Obstacle& i, const Obstacle& j) {return i.center.squaredNorm() < j.center.squaredNorm();}
	bool isTrajectoryObstructed(const utils::Vector2d& start, const utils::Vector2d& goal);
	
	#define PW(x) ((x)*(x))
	Forces();
	void computeObstacleForceSum(const sensor_msgs::LaserScan::ConstPtr& scan, utils::Vector2d& forceSum, unsigned& points, std::vector<Obstacle>& obstacles) const;
	utils::Vector2d computeSocialForce(const utils::Vector2d& position, const utils::Vector2d& velocity) const;
	void computeDesiredForce();
	void selectGoal(ControllerMode controller_mode, const utils::Vector2d& controller_mode_goal, PathProvider* pathProvider);
	void selectGoal1(ControllerMode controller_mode, const utils::Vector2d& controller_mode_goal);
	void selectHeuristicGoal();
	void computeGroupForce();
	bool transformPoint(double& x, double& y, const std::string& sourceFrameId, const std::string& targetFrameId) const;

	Data data;
	Parameters params;
	utils::Vector2d laserForceSum;
	utils::Vector2d xtionForceSum;
	unsigned laserPoints;
	unsigned xtionPoints;
	std::vector<Obstacle> laserObstacles;
	std::vector<Obstacle> xtionObstacles;
	std::list<utils::Vector2d> targetVelocities;
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

		time = 999999;
		distance = 999999;
		bool coll=false;
		for (unsigned i = 0; i< data.obstacles.size(); i++) {
			if ((linearVelocity>0 && data.obstacles[i].center.getX()<0) || (linearVelocity<0 && data.obstacles[i].center.getX()>0)) {
				continue;
			}
			if (fabs(data.obstacles[i].center.getY())<= params.robotRadius) {
				double aux_distance = fabs(data.obstacles[i].center.getX());
				double offset = std::sqrt(PW(params.robotRadius) - PW(data.obstacles[i].center.getY()));
				aux_distance -= offset;
				if (aux_distance <0 ) {
					aux_distance = 0;
				}
				double aux_time = aux_distance / fabs(linearVelocity);

				coll = coll || ((aux_time >= 0.0) && fabs(linearVelocity) > std::sqrt(2*aux_distance * params.robotMaxLinearAcceleration));

				if((aux_time >= 0.0) && (aux_time < time))
				{
					time=aux_time;
					distance=fabs(aux_distance);	
				}
				
				

			}		
		}
		
		
		return coll;
	}


	// Angular and linear velocity, general case
	time = 999999;
	distance = 999999;

	//double ang=999999;
	bool coll=false;

	//Alternative version:
	utils::Vector2d icc(0,linearVelocity/angularVelocity); //Instantaneous Centre of Rotation. In the local frame (base_link), it is located in the Y axis
	for (unsigned i = 0; i< data.obstacles.size(); i++) {

		//Obstacle with respect to the instantaneous rotation center
		utils::Vector2d u = (data.obstacles[i].center - icc); 

		//If the obstacle is outside the annular disc traversed by the robot (radii a and b, centered at icc), no collision
		double a = icc.norm() - params.robotRadius;
		double b = icc.norm() + params.robotRadius;
		
		//icc within robot radius
		if (a < 0.0)
			a = 0.0;

		//No collission
		if(u.norm() < a || u.norm() > b)
			continue;

		//Angle, along the circular path, where the collision occurs
		double ctheta = (icc.squaredNorm()+u.squaredNorm()-PW(params.robotRadius))/(2.0*icc.norm()*u.norm());
		double theta = std::acos(ctheta);

		//Set the correct cuadrant according to the motion of the robot
		if ((linearVelocity>0 && angularVelocity > 0)) {
			u.set(u.getX(),-u.getY());
		}else if ((linearVelocity<0 && angularVelocity < 0))
		{
			u.set(-u.getX(),-u.getY());
		}else if ((linearVelocity<0 && angularVelocity > 0))
		{
			u.set(-u.getX(),u.getY());
		}

		//Angular distance until the collision (the current angle of the obstacle minus the angle where the collision occurs)
		//We convert it to have a positive value
		double angleObs = std::atan2(u[0],u[1]);
		if(angleObs <0.0)
			angleObs+=2.0*M_PI;

		double deltaTheta = angleObs - theta;


		double aux_distance = deltaTheta * u.norm(); //Distance until collision. Angle (in radians) times radius. It has sign
		double aux_time = aux_distance/fabs(linearVelocity); 
		

		//double aux_time = deltaTheta/fabs(angularVelocity);
		//double aux_distance = fabs(linearVelocity)*aux_time;
		//Check the condition on angular velocities. Can we avoid the obstacle by stopping the rotation?
		

		coll = coll || ((aux_time >= 0.0) && ((fabs(linearVelocity) > std::sqrt(2*fabs(aux_distance) * params.robotMaxLinearAcceleration)) || 
					(fabs(angularVelocity) > std::sqrt(2*fabs(deltaTheta) * params.robotMaxAngularAcceleration))));

		//coll = coll || ((aux_time >= 0.0) && (fabs(linearVelocity) > std::sqrt(2*fabs(aux_distance) * params.robotMaxLinearAcceleration)));	
		
		
		if((aux_time >= 0.0) && (aux_time < time))
		{
			time=aux_time;
			distance=fabs(aux_distance);
			//ang=deltaTheta;		
		}	

		

	}

	return coll;


}

//Old version
/*
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
*/



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
void Forces::compute(ControllerMode controller_mode, const utils::Vector2d& controller_mode_goal, PathProvider *pathProvider)
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
	selectGoal(controller_mode, controller_mode_goal,pathProvider);
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
void Forces::computeTargetGroupForces(utils::Vector2d& vis, utils::Vector2d& att, utils::Vector2d& rep) const
{
	vis.set(0,0);
	att.set(0,0);
	rep.set(0,0);

	if (!data.targetFound) {
		return;
	}

	// Gaze force
	if (data.target.velocity.norm() > 0.01) {
		utils::Vector2d desiredDirection = data.target.velocity.normalized();
		utils::Vector2d relativeCom =  data.robot.position - data.target.position;
		utils::Angle visionAngle = utils::Angle::fromDegree(90);
		double elementProduct = desiredDirection.dot(relativeCom);
		utils::Angle comAngle = utils::Angle::fromRadian(std::acos(elementProduct / (desiredDirection.norm() * relativeCom.norm())));
		if (comAngle > visionAngle) {
			double desiredDirectionSquared = desiredDirection.squaredNorm();
			double desiredDirectionDistance = elementProduct / desiredDirectionSquared;
			vis = desiredDirectionDistance * desiredDirection;
			vis *= params.forceFactorGroupGaze;
		}
	}
	// Coherence force
	utils::Vector2d com = (data.target.position + data.robot.position)*0.5;
	utils::Vector2d relativeCom = com - data.target.position;
	double cohDistance = relativeCom.norm();
	att = relativeCom;
	double softenedFactor = params.forceFactorGroupCoherence * (std::tanh(cohDistance - 0.5) + 1) / 2;
	att *= softenedFactor;
	
	// Repulsion Force
	utils::Vector2d diff = data.target.position - data.robot.position;
	double repDistance = diff.norm();
	if (repDistance < params.robotRadius + params.personRadius) {
		rep += diff;
	}
	rep *= params.forceFactorGroupRepulsion;
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


bool Forces::transformPoint(double& x, double& y, const std::string& sourceFrameId, const std::string& targetFrameId) const
{

	static tf::TransformListener tf_listener(ros::Duration(10));
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


inline
bool Forces::distanceToObstacle(const utils::Vector2d& point, double& distance, const std::string& frame)
{
	double x = point.getX();
	double y = point.getY();
	if (!transformPoint(x,y,frame,"map")) {
		return false;
	}
	utils::Vector2d aux(x,y);
	distance = sfm::MAP.getNearestObstacle(aux).distance;
	return true;
}

inline
bool Forces::isTrajectoryObstructed(const utils::Vector2d& start, const utils::Vector2d& goal)
{
	// start and goal in map frame
	utils::Vector2d u = goal - start;
	double distance = u.norm();
	u = (u/distance) * 0.05;
	utils::Vector2d w;
	while (w.norm() < distance) {
		if (sfm::MAP.isObstacle(start+w)) {
			return true;
		}
		w+=u;
	}
	return false;
}

inline
void Forces::selectGoal(ControllerMode controller_mode, const utils::Vector2d& controller_mode_goal, PathProvider *pathProvider)
{
	data.validGoal = false;
	if (params.heuristicPlanner || controller_mode == HEURISTIC) {
		selectHeuristicGoal();
	}else if (controller_mode == SET_FINAL_GOAL) {
		double x = controller_mode_goal.getX();
		double y = controller_mode_goal.getY();		
		if (transformPoint(x,y, "map", "odom")) {
			data.goal.set(x,y);			
			data.validGoal = true;
		}
	} else if (controller_mode == SET_GOAL && pathProvider!=NULL) {
		double x = data.robot.position.getX();
		double y = data.robot.position.getY();
		if (transformPoint(x,y, "odom", "map")) {
			utils::Vector2d a,b;
			a.set(x,y);
			utils::Vector2d localGoal;	
			pathProvider->getNextPoint(a,controller_mode_goal,b);
			x = b.getX();
			y = b.getY();
			if (transformPoint(x,y, "map", "odom")) {
				data.goal.set(x,y);			
				data.validGoal = true;
			}
		}
	} else if (data.targetFound && 
			(data.robot.position - data.target.position).norm() <= FORCES.getParams().targetLookahead) {
		if (controller_mode == LEFT) {
			data.goal = data.leftGoal;
			data.validGoal = true;
		} else if (controller_mode == RIGHT) {
			data.goal = data.rightGoal;
			data.validGoal = true;
		} else if (controller_mode == BEHIND) {
			data.goal = data.behindGoal;
			data.validGoal = true;
		} else if (controller_mode == FOLLOW_PATH && data.pathFound) {
			data.goal = data.followGoal;
			data.validGoal = true;
		}
	} else if (controller_mode!=WAIT && data.pathFound) {
		data.goal = data.followGoal;
		data.validGoal = true;
	}
}



inline
void Forces::selectHeuristicGoal()
{
	data.validGoal = false;
	if (data.targetFound && 
			(data.robot.position - data.target.position).norm() <= FORCES.getParams().targetLookahead) {

		double robot_x = data.robot.position.getX();
		double robot_y = data.robot.position.getY();
		double right_x = data.rightGoal.getX();
		double right_y = data.rightGoal.getY();
		double left_x = data.leftGoal.getX();
		double left_y = data.leftGoal.getY();	
		double behind_x = data.behindGoal.getX();
		double behind_y = data.behindGoal.getY();

		if (!transformPoint(robot_x,robot_y,"odom","map") ||
			!transformPoint(right_x,right_y,"odom","map") ||	
			!transformPoint(left_x,left_y,"odom","map") ||
			!transformPoint(behind_x,behind_y,"odom","map")) {
			return;
		}
		utils::Vector2d start(robot_x,robot_y);
		utils::Vector2d goal(left_x,left_y);
		bool left_obstructed = isTrajectoryObstructed(start,goal);
		double left_value = params.k_distance_obstacle * sfm::MAP.getNearestObstacle(goal).distance + 
					params.k_distance_robot * (data.leftGoal - data.robot.position).norm();
		goal.set(right_x,right_y);
		bool right_obstructed = isTrajectoryObstructed(start,goal);
		double right_value = params.k_distance_obstacle * sfm::MAP.getNearestObstacle(goal).distance + 
					params.k_distance_robot * (data.rightGoal - data.robot.position).norm();
		goal.set(behind_x,behind_y);
		bool behind_obstructed = isTrajectoryObstructed(start,goal);
		
		if (left_obstructed && right_obstructed && behind_obstructed && data.pathFound) {
			data.goal = data.followGoal;
			data.validGoal = true;
			return;
		}	
		if (left_obstructed && right_obstructed && !behind_obstructed) {
			data.goal = data.behindGoal;
			data.validGoal = true;
			return;
		}
		if (left_obstructed || right_value < left_value) {
			data.goal = data.rightGoal;
			data.validGoal = true;
			return;
		}
		if (right_obstructed || left_value < right_value) {
			data.goal = data.leftGoal;
			data.validGoal = true;
			return;
		}
	} else if (data.pathFound) {
		data.goal = data.followGoal;
		data.validGoal = true;
	}

} 


inline
void Forces::selectGoal1(ControllerMode controller_mode, const utils::Vector2d& controller_mode_goal)
{
	//static  model::AStarPathProvider aStar;

	// By default, if the goal is not valid, it will perform an antimove
	data.validGoal = false;
	if (params.heuristicPlanner || controller_mode == HEURISTIC) {
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
		
		if (controller_mode == SET_GOAL) {
			/*double x = data.robot.position.getX();
			double y = data.robot.position.getY();
			transformPoint(x,y, "odom", "map") ;
			utils::Vector2d a,b;
			a.set(x,y);
			utils::Vector2d localGoal;	
			aStar.getNextPoint(a,controller_mode_goal,b);
			x = b.getX();
			y = b.getY();
			transformPoint(x,y, "map", "odom") ;
			data.goal.set(x,y);			
			data.validGoal = true;*/
		} else if (data.targetFound && controller_mode != FOLLOW_PATH) {
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
bool Forces::setPeople(const upo_msgs::PersonPoseArrayUPO::ConstPtr& people, unsigned targetId, bool targetReceived, const utils::Vector2d& targetPos,const utils::Vector2d& targetVel)
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
		if (people->personPoses[i].id != targetId || !targetReceived) {
			data.socialForce += params.forceFactorSocial*computeSocialForce(position,velocity);
		}	
		if (people->personPoses[i].id == targetId && !targetReceived) {
			data.target.position = position;
			data.target.velocity = velocity;
			data.target.yaw = yaw;
			data.target.linearVelocity = people->personPoses[i].vel;
			data.target.angularVelocity = 0;
			data.targetFound = true;
			if (fabs(people->personPoses[i].vel) >= params.personVelocityZeroThreshold) {
				targetVelocities.push_back(velocity);
				if (targetVelocities.size()>params.target_vel_sma_period) {
					targetVelocities.pop_front();
				}
				utils::Vector2d targetVelocity = velocity;
				if (params.target_vel_sma_period>1 && targetVelocities.size() == params.target_vel_sma_period) {
					targetVelocity.set(0,0);
					for (auto it = targetVelocities.begin(); it != targetVelocities.end(); ++it) {
						targetVelocity += *it;
					}
					targetVelocity /= (double) params.target_vel_sma_period;
				}
				data.leftGoal = 
					data.target.position + params.naiveGoalTime * targetVelocity +  
						targetVelocity.normalized().leftNormalVector();
				data.rightGoal =
					  data.target.position + params.naiveGoalTime * targetVelocity +
						  targetVelocity.normalized().rightNormalVector();
				data.behindGoal =
					  data.target.position + params.naiveGoalTime * targetVelocity - targetVelocity.normalized();
			}


			if (data.targetHistory.empty() || (position - data.targetHistory.front().position).norm() > 0.25) {
				data.targetHistory.push_front(data.target);
				if (data.targetHistory.size()>20) {
					data.targetHistory.pop_back();
					//auto it = data.targetHistory.rbegin();
					//++it;
					//if ((data.robot.position - it->position).norm() <= FORCES.getParams().targetLookahead) {
					//	data.targetHistory.pop_back();
					//} else {
					//	data.targetHistory.pop_front();
					//}
				}
			}
		}
	}
	if (targetReceived) {
		utils::Vector2d position = targetPos;
		utils::Angle yaw = targetVel.angle();
		utils::Vector2d velocity = targetVel;
		
		if (targetVel.norm() < params.personVelocityZeroThreshold) {
			velocity.set(0,0);
		} 
		data.socialForce += params.forceFactorSocial*computeSocialForce(position,velocity);
		data.target.position = position;
		data.target.velocity = velocity;
		data.target.yaw = yaw;
		data.target.linearVelocity = velocity.norm();
		data.target.angularVelocity = 0;
		data.targetFound = true;
		if (fabs(data.target.linearVelocity) >= params.personVelocityZeroThreshold) {
			targetVelocities.push_back(velocity);
			if (targetVelocities.size()>params.target_vel_sma_period) {
				targetVelocities.pop_front();
			}
			utils::Vector2d targetVelocity = velocity;
			if (params.target_vel_sma_period>1 && targetVelocities.size() == params.target_vel_sma_period) {
				targetVelocity.set(0,0);
				for (auto it = targetVelocities.begin(); it != targetVelocities.end(); ++it) {
					targetVelocity += *it;
				}
				targetVelocity /= (double) params.target_vel_sma_period;
			}
			data.leftGoal = 
				data.target.position + params.naiveGoalTime * targetVelocity +  
					targetVelocity.normalized().leftNormalVector();
			data.rightGoal =
				  data.target.position + params.naiveGoalTime * targetVelocity +
					  targetVelocity.normalized().rightNormalVector();
			data.behindGoal =
				  data.target.position + params.naiveGoalTime * targetVelocity - targetVelocity.normalized();
		}


		if (data.targetHistory.empty() || (position - data.targetHistory.front().position).norm() > 0.2) {
			data.targetHistory.push_front(data.target);
			if (data.targetHistory.size()>20) {
				data.targetHistory.pop_back();
				//auto it = data.targetHistory.rbegin();
				//++it;
				//if ((data.robot.position - it->position).norm() <= FORCES.getParams().targetLookahead) {
				//	data.targetHistory.pop_back();
				//} else {
				//	data.targetHistory.pop_front();
				//}
			}
		}
		
	}	
	double x = data.robot.position.getX();
	double y = data.robot.position.getY();
	if (transformPoint(x,y,"odom","map")) {
		utils::Vector2d start(x,y);
		bool first=true;
		for (auto it = data.targetHistory.begin(); it!= data.targetHistory.end(); ++it) {
			if ((data.robot.position - it->position).norm() <= FORCES.getParams().targetLookahead) {
				double x = it->position.getX();
				double y = it->position.getY();
				if (transformPoint(x,y,"odom","map")) {
					utils::Vector2d goal(x,y);
					if (!isTrajectoryObstructed(start,goal)) {
						if (!first) {
							data.pathFound = true;
							data.followGoal = it->position;
						}
						break;
					}
				}	
			}
			first=false;
		}
	}

	//If there is not a point within the lookahead, look for the closest point
	
	if(!data.pathFound)
	{
		double min_dist=99999.9;
		for (auto it = data.targetHistory.begin(); it!= data.targetHistory.end(); ++it) {
			if ((data.robot.position - it->position).norm() <= min_dist) {
				min_dist = (data.robot.position - it->position).norm();			
				data.pathFound = true;
				data.followGoal = it->position;
			}
		}
	}
	
	if (!data.targetFound) {
		targetVelocities.clear();
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
