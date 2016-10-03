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


#ifndef _ROBOT_FORCES_HPP_
#define _ROBOT_FORCES_HPP_

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <upo_msgs/PersonPoseArrayUPO.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_listener.h>
#include "vector2d.hpp"

namespace wsbs
{


// How to get the current goal of the target
enum TargetGoalsPolicy {
	NAIVE_GOALS    = 0,
	PEOPLE_GOALS   = 1
};

// What to do when the target is lost
enum TargetLostPolicy {
	STOP_ROBOT                   = 0,
	WALK_TO_LAST_TARGET_POSITION = 1,
	WALK_TO_LAST_TARGET_GOAL     = 2
};


class RobotForces
{
public:
	RobotForces(ros::NodeHandle& pn);
	~RobotForces() {}

	unsigned target_id;
	utils::Vector2d goalPosition;
	double goalR;
	TargetGoalsPolicy target_goals_policy;
	TargetLostPolicy target_lost_policy;
	double naive_goals_time;
	double finish_timeout;
	
	double robot_radius;
	double person_radius;
	double collision_threshold;
	double relaxation_time;
	double force_factor_desired;	
	double force_factor_obstacle;
	double force_sigma_obstacle;
	
	double force_factor_social;
	double force_factor_group_gaze;
	double force_factor_group_coherence;
	double force_factor_group_repulsion;
	double lambda;
	double gamma;
	double n_prime;
	double n;

	double robot_max_lin_vel;
	double robot_max_ang_vel;
	
	double robot_lin_vel_inc;
	double robot_ang_vel_inc;

	double robot_max_lin_acc;
	double robot_max_ang_acc;

	

	ros::Publisher cmd_vel_pub;
	
	bool setRobot(const nav_msgs::Odometry::ConstPtr& odom);
	bool setLaser(const sensor_msgs::LaserScan::ConstPtr& laser);
	bool setXtion(const sensor_msgs::LaserScan::ConstPtr& xtion);
	bool setPeople(const upo_msgs::PersonPoseArrayUPO::ConstPtr& people);

	void publishMarkers();

	bool isCollision() {return laser_collision || xtion_collision;}
	bool isTarget() const {return is_target;}

	utils::Vector2d getObstacleForce() const;
	const utils::Vector2d& getSocialForce() const {return socialForce;}
	utils::Vector2d getGlobalForce() const {return desiredForce + getObstacleForce() + socialForce + groupForce;}

	void stopRobot() {cmd_vel_pub.publish(zeroTwist);}

	bool generateCmdVel(double dt);	

	void resetForces()
	{
		desiredForce = utils::Vector2d::Zero();
		desiredDirection = utils::Vector2d::Zero();
		laserForceSum = utils::Vector2d::Zero();
		xtionForceSum = utils::Vector2d::Zero();
		socialForce = utils::Vector2d::Zero();
		groupForce = utils::Vector2d::Zero();
	}

private:

	

	#define PW(x) ((x)*(x))

	utils::Vector2d computeSocialForce(const utils::Vector2d& position, const utils::Vector2d& velocity) const;
	void computeGroupForce(const utils::Vector2d& position);
	bool isCollision(const sensor_msgs::LaserScan::ConstPtr& scan);
	
	static std_msgs::ColorRGBA getColor(double r, double g, double b, double a);

	void computeObstacleForceSum(const sensor_msgs::LaserScan::ConstPtr& scan, utils::Vector2d& force, unsigned& points, std::vector<utils::Vector2d>& obstacles, double& min) const;
	void publishForceMarker(unsigned index, const std_msgs::ColorRGBA& color, 
					const utils::Vector2d& force, visualization_msgs::MarkerArray& markers);
	
	void computeDesiredForce(bool antimove=false);

	bool evaluateTwist(double linVel, double angVel, double dt, const utils::Vector2d& positionRef, const utils::Angle& yawRef,double& value);

	void generateVelocities();

	void computeNextPose(double linVel, double angVel, double dt, utils::Vector2d& position, utils::Angle& yaw) const;

	utils::Vector2d desiredForce;
	utils::Vector2d desiredDirection;	
	utils::Vector2d laserForceSum;
	utils::Vector2d xtionForceSum;
	utils::Vector2d socialForce;
	utils::Vector2d groupForce;
	
	unsigned laserPoints;
	unsigned xtionPoints;
	ros::Publisher markers_pub;

	utils::Vector2d robotPosition;
	utils::Vector2d robotVelocity;
	utils::Angle robotYaw;

	geometry_msgs::Twist zeroTwist;
	geometry_msgs::Twist rotateLeftTwist;
	geometry_msgs::Twist rotateRightTwist;	

	utils::Vector2d targetPosition;
	utils::Vector2d targetGoal;

	bool laser_collision;
	bool xtion_collision;
	bool is_target;
	bool first_time;
	double robot_lin_vel;
	double robot_ang_vel;	
	double target_lin_vel;
	std::vector<geometry_msgs::Twist> velocities;

	std::vector<utils::Vector2d> laserObstacles;
	std::vector<utils::Vector2d> xtionObstacles;

	double minXtionDistance;
	double minLaserDistance;
	
	ros::Time timer;
	
};

inline
RobotForces::RobotForces(ros::NodeHandle& pn)
:target_id(9999999),
 goalR(0),
 target_goals_policy(NAIVE_GOALS),
 target_lost_policy(WALK_TO_LAST_TARGET_POSITION),
 naive_goals_time(5.0),
 finish_timeout(20.0),
 robot_radius(0.35),
 person_radius(0.35),
 collision_threshold(0.35),
 relaxation_time(0.5),
 force_factor_desired(1.0),
 force_factor_obstacle(10),
 force_sigma_obstacle(0.2), 
 force_factor_social(2.1),
 force_factor_group_gaze(3.0),
 force_factor_group_coherence(2.0),
 force_factor_group_repulsion(1.0),
 lambda(2.0),
 gamma(0.35),
 n_prime(3.0),
 n(2.0),
 robot_max_lin_vel(0.3),
 robot_max_ang_vel(M_PI*0.125),
 robot_lin_vel_inc(0.05),
 robot_ang_vel_inc(M_PI/240),
 robot_max_lin_acc(1.0),
 robot_max_ang_acc(1.0),
 laserPoints(0),
 xtionPoints(0),
 laser_collision(false),
 xtion_collision(false),
 is_target(false),
 first_time(true),
 robot_lin_vel(0),
 robot_ang_vel(0),
 minXtionDistance(999999),
 minLaserDistance(999999)
{
	markers_pub = pn.advertise<visualization_msgs::MarkerArray>("/wsbs/markers/robot_forces", 1);
	zeroTwist.linear.x = 0;
	zeroTwist.linear.y = 0;
	zeroTwist.linear.z = 0;
	zeroTwist.angular.x = 0;
	zeroTwist.angular.y = 0;
	zeroTwist.angular.z = 0;

	rotateLeftTwist = zeroTwist;
	rotateLeftTwist.angular.z = 0.3;

	rotateRightTwist = zeroTwist;
	rotateRightTwist.angular.z = -0.3;	

	timer = ros::Time::now();
	
}


inline
void RobotForces::generateVelocities()
{
	double ang_vels[9] = {-0.8, -0.6, -0.4, -0.2, 0, 0.2, 0.4, 0.6, 0.8};
	double lin_vels[5] = {-0.3, -0.15, 0, 0.15, 0.3};
	for (unsigned i = 0; i< 9; i++) {
		for (unsigned j = 0; j<5; j++) {
			geometry_msgs::Twist twist;
			twist.linear.x = lin_vels[j];
			twist.linear.y = 0;
			twist.linear.z = 0;
			twist.angular.x = 0;
			twist.angular.y = 0;
			twist.angular.z = ang_vels[i];
			velocities.push_back(twist);
		}
	}

	/*
	for (double lin_vel = 0; lin_vel<=robot_max_lin_vel; lin_vel+=robot_lin_vel_inc) {
		for (double ang_vel=0; ang_vel<=robot_max_ang_vel;ang_vel+=robot_ang_vel_inc) {
			geometry_msgs::Twist twist;
			twist.linear.x = lin_vel;
			twist.linear.y = 0;
			twist.linear.z = 0;
			twist.angular.x = 0;
			twist.angular.y = 0;
			twist.angular.z = ang_vel;
			velocities.push_back(twist);
			if (ang_vel>0) {
				twist.angular.z = -ang_vel;
				velocities.push_back(twist);
			}
		}
		if (lin_vel>0) {
			for (double ang_vel=0; ang_vel<=robot_max_ang_vel;ang_vel+=robot_ang_vel_inc) {
				geometry_msgs::Twist twist;
				twist.linear.x = -lin_vel;
				twist.linear.y = 0;
				twist.linear.z = 0;
				twist.angular.x = 0;
				twist.angular.y = 0;
				twist.angular.z = ang_vel;
				velocities.push_back(twist);
				if (ang_vel>0) {
					twist.angular.z = -ang_vel;
					velocities.push_back(twist);
				}
			}
		}
	}
	*/
}

void RobotForces::computeNextPose(double linVel, double angVel, double dt, utils::Vector2d& position, utils::Angle& yaw) const
{
	yaw = robotYaw + utils::Angle::fromRadian(angVel * dt);
	double imd = linVel*dt;
	utils::Vector2d inc(imd * std::cos(yaw.toRadian() + angVel*dt/2),imd * std::sin(yaw.toRadian() + angVel*dt/2));
	position = robotPosition + inc;
}


bool RobotForces::evaluateTwist(double linVel, double angVel, double dt, 
			const utils::Vector2d& positionRef, const utils::Angle& yawRef, double& value)
{
	utils::Vector2d position;
	utils::Angle yaw;
	if (fabs(linVel)>0) {	
		
		for (unsigned j = 10; j<= 20; j++) {
			double t = ((double)j)*0.1;
			computeNextPose(linVel,angVel,t,position,yaw);	
			position -= robotPosition;	
			for (unsigned i=0;i<laserObstacles.size();i++) {
				if ((laserObstacles[i] - position).squaredNorm() < PW(robot_radius)) {
					return true;
				}
			}
			for (unsigned i=0;i<xtionObstacles.size();i++) {
				if ((xtionObstacles[i] - position).squaredNorm() < PW(robot_radius)) {
					return true;
				}
			}
		}
	}
	
	computeNextPose(linVel,angVel,dt,position,yaw);	

	double a = position.getX() - positionRef.getX();
	double b = position.getY() - positionRef.getY();
	utils::Angle angle = yaw - yawRef;
	double c = angle.toRadian();
	value = (a*a) + (b*b) + (c*c);
	return false;
}


inline
bool RobotForces::generateCmdVel(double dt)
{
	if (is_target && target_lin_vel<0.05 && (robotPosition - targetPosition).norm()<1.0) {
		utils::Vector2d u(robotYaw.cos(),robotYaw.sin());
		utils::Vector2d w = targetPosition-robotPosition;
		double theta = u.angleTo(w).toDegree();
		if (theta>10) {
			cmd_vel_pub.publish(rotateLeftTwist);
		} else if (theta <-10) {
			cmd_vel_pub.publish(rotateRightTwist);
		} else {
			stopRobot();
			if (goalR<0 && (ros::Time::now() - timer).toSec()>finish_timeout) {
				return true;
			} else if (goalR>0 && (robotPosition - goalPosition).norm() < goalR ){
				return true;
			}
		}
		return false;
	}
	timer = ros::Time::now();

	if (velocities.empty()) {
		generateVelocities();
	}


	utils::Vector2d velocityRef = robotVelocity + getGlobalForce() * dt;
	if (velocityRef.norm() > robot_max_lin_vel) {
		velocityRef.normalize();
		velocityRef *= robot_max_lin_vel;
	}

	utils::Vector2d positionRef = robotPosition + velocityRef * dt;
	utils::Angle yawRef = velocityRef.angle();

	geometry_msgs::Twist bestTwist = zeroTwist;
	double minValue = 9999999999999;
	for (unsigned i=0;i<velocities.size();i++) {
		double linVel = velocities[i].linear.x;
		double angVel = velocities[i].angular.z;
		
		if (fabs(robot_lin_vel) < 0.01 && fabs(robot_ang_vel) < 0.01 && (fabs(linVel)<0.15 || fabs(angVel)<0.6)) {
			continue;
		}
		
		//if (isCollision() && fabs(linVel)>0 && fabs(angVel) > 0) {
		//	continue;
		//}
		double linAcc = (linVel - robot_lin_vel)/dt;
		double angAcc = (angVel - robot_ang_vel)/dt;
		if (fabs(linAcc) > robot_max_lin_acc) {
			if (linAcc<0) {
				linVel = robot_lin_vel - robot_max_lin_acc*dt;	
			} else {
				linVel = robot_lin_vel + robot_max_lin_acc*dt;
			}
		}
		if (fabs(angAcc) > robot_max_ang_acc) {
			if (angAcc<0) {
				angVel = robot_ang_vel - robot_max_ang_acc*dt;	
			} else {
				angVel = robot_ang_vel + robot_max_ang_acc*dt;
			}
		}
		if (linVel > robot_max_lin_vel) {
			linVel = robot_max_lin_vel;
		} else if (linVel < -robot_max_lin_vel) {
			linVel = -robot_max_lin_vel;
		}

		if (angVel > robot_max_ang_vel) {
			angVel = robot_max_ang_vel;
		} else if (angVel < -robot_max_ang_vel) {
			angVel = -robot_max_ang_vel;
		}
		double value=0;
		bool collision = evaluateTwist(linVel,angVel,dt,positionRef,yawRef,value);
		if (!collision && value < minValue) {
			minValue = value;
			bestTwist.linear.x = linVel;
			bestTwist.angular.z = angVel;
		}
	}	
	cmd_vel_pub.publish(bestTwist);
	return false;


}


inline
bool RobotForces::setRobot(const nav_msgs::Odometry::ConstPtr& odom)
{
	
	if (odom->header.frame_id != "/odom" && odom->header.frame_id !="odom") {
		ROS_ERROR("Odometry frame is %s, it should be odom",odom->header.frame_id.c_str()); 
		return false;
	}
	robotPosition.set(odom->pose.pose.position.x,odom->pose.pose.position.y); 
	robotYaw = utils::Angle::fromRadian(tf::getYaw(odom->pose.pose.orientation));	
	robotVelocity.set(odom->twist.twist.linear.x * robotYaw.cos(), odom->twist.twist.linear.x * robotYaw.sin());
	robot_lin_vel = odom->twist.twist.linear.x;
	robot_ang_vel = odom->twist.twist.angular.z;	
	return true;
}


inline 
bool RobotForces::setLaser(const sensor_msgs::LaserScan::ConstPtr& laser)
{
	if (laser->header.frame_id != "/base_link" && laser->header.frame_id !="base_link") {
		ROS_ERROR("Laser frame is %s, it should be base_link",laser->header.frame_id.c_str()); 
		return false;
	}
	bool collision = isCollision(laser);
	if (!isCollision() && collision) {
		
		//stopRobot();	
	}
	laser_collision = collision;
	computeObstacleForceSum(laser,laserForceSum,laserPoints,laserObstacles,minLaserDistance);
	
	return true;
}


inline
bool RobotForces::setXtion(const sensor_msgs::LaserScan::ConstPtr& xtion)
{
	if (xtion->header.frame_id != "/base_link" && xtion->header.frame_id !="base_link") {
		ROS_ERROR("Xtion frame is %s, it should be base_link",xtion->header.frame_id.c_str()); 
		return false;
	}
	bool collision = isCollision(xtion);
	if (!isCollision() && collision) {
		//stopRobot();	
	}
	xtion_collision = collision;
	computeObstacleForceSum(xtion,xtionForceSum,xtionPoints,xtionObstacles,minXtionDistance);
	return true;
}



inline
utils::Vector2d RobotForces::computeSocialForce(const utils::Vector2d& position, const utils::Vector2d& velocity) const
{
	utils::Vector2d diff = position - robotPosition;
	utils::Vector2d diffDirection = diff.normalized();
	utils::Vector2d velDiff = robotVelocity - velocity;
	utils::Vector2d interactionVector = lambda * velDiff + diffDirection;
	double interactionLength = interactionVector.norm();
	utils::Vector2d interactionDirection = interactionVector/interactionLength;
	utils::Angle theta = interactionDirection.angleTo(diffDirection);
	double B = gamma * interactionLength;
	double thetaRad = theta.toRadian();	
	double forceVelocityAmount = -std::exp(-diff.norm()/B - PW(n_prime*B*thetaRad) );
	double forceAngleAmount = -theta.sign() * std::exp(-diff.norm() / B - PW(n * B * thetaRad));
	utils::Vector2d forceVelocity = forceVelocityAmount * interactionDirection;
	utils::Vector2d forceAngle = forceAngleAmount * interactionDirection.leftNormalVector();
	return forceVelocity + forceAngle;
}




inline
void RobotForces::computeGroupForce(const utils::Vector2d& position)
{
	utils::Vector2d gazeForce;
	utils::Vector2d coherenceForce;
	utils::Vector2d repulsionForce;

	// Gaze force
	utils::Vector2d relativeCom = position - robotPosition;
	utils::Angle visionAngle = utils::Angle::fromDegree(90);
	double elementProduct = desiredDirection.dot(relativeCom);
	utils::Angle comAngle = utils::Angle::fromRadian(std::acos(elementProduct / (desiredDirection.norm() * relativeCom.norm())));
	if (comAngle > visionAngle) {
		#ifdef _PAPER_VERSION_
		utils::Angle necessaryRotation = comAngle - visionAngle;
		gazeForce = -necessaryRotation.toRadian() * desiredDirection;		
		#else
		double desiredDirectionSquared = desiredDirection.squaredNorm();
		double desiredDirectionDistance = elementProduct / desiredDirectionSquared;
		gazeForce = desiredDirectionDistance * desiredDirection;
		#endif
		gazeForce *= force_factor_group_gaze;
	}

	// Coherence force
	utils::Vector2d com = (position + robotPosition)*0.5;
	relativeCom = com - robotPosition;
	double cohDistance = relativeCom.norm();
	#ifdef _PAPER_VERSION_
	if (cohDistance >= 0.5) {
		coherenceForce = relativeCom.normalized();
		coherenceForce *= force_factor_group_coherence;
	} 
	#else
	coherenceForce = relativeCom;
	double softenedFactor = force_factor_group_coherence * (std::tanh(cohDistance - 0.5) + 1) / 2;
	coherenceForce *= softenedFactor;
	#endif

	// Repulsion Force
	utils::Vector2d diff = robotPosition - position;
	double repDistance = diff.norm();
	if (repDistance < robot_radius + person_radius) {
		repulsionForce += diff;
	}
	repulsionForce *= force_factor_group_repulsion;

	groupForce =  gazeForce +  coherenceForce + repulsionForce;
	
}


inline
void RobotForces::computeDesiredForce(bool antimove)
{
	
	if (antimove) {
		desiredForce = -robotVelocity / relaxation_time;
		desiredDirection = utils::Vector2d::Zero();
		return;
	}
	utils::Vector2d diff = targetGoal - robotPosition;
	desiredDirection = diff.normalized();	
	desiredForce = force_factor_desired * (desiredDirection * robot_max_lin_vel - robotVelocity)/relaxation_time;

}


inline
bool RobotForces::setPeople(const upo_msgs::PersonPoseArrayUPO::ConstPtr& people)
{
	if (people->header.frame_id != "/odom" && people->header.frame_id !="odom") {
		ROS_ERROR("People frame is %s, it should be odom",people->header.frame_id.c_str()); 
		return false;
	}
	desiredForce = utils::Vector2d::Zero();
	socialForce = utils::Vector2d::Zero();
	groupForce = utils::Vector2d::Zero();
	is_target = false;
	for (unsigned i=0; i< people->personPoses.size(); i++) {
		utils::Vector2d position(people->personPoses[i].position.x,people->personPoses[i].position.y);
		utils::Angle yaw = utils::Angle::fromRadian(tf::getYaw(people->personPoses[i].orientation));
		utils::Vector2d velocity(people->personPoses[i].vel * yaw.cos(), people->personPoses[i].vel * yaw.sin());
		if (fabs(people->personPoses[i].vel) < 0.05) {
			velocity = utils::Vector2d::Zero();
		} else	if (people->personPoses[i].vel < 0) {
			yaw += utils::Angle::fromDegree(180);
			velocity.set(-people->personPoses[i].vel * yaw.cos(), -people->personPoses[i].vel * yaw.sin());
			
		}		
		socialForce += force_factor_social*computeSocialForce(position,velocity);
		if (people->personPoses[i].id == target_id) {
			first_time=false;
	 		is_target = true;
			target_lin_vel = people->personPoses[i].vel;
			targetPosition = position;
			if (target_goals_policy == NAIVE_GOALS) {
				targetGoal = position + naive_goals_time * velocity;
				computeDesiredForce();
			}
			computeGroupForce(position);				
		}
	}
	if (!is_target) {
		if (first_time || target_lost_policy == STOP_ROBOT) {
			computeDesiredForce(true);
		} else if (target_lost_policy == WALK_TO_LAST_TARGET_POSITION) {
			targetGoal = targetPosition;
			computeDesiredForce();
		} else {
			computeDesiredForce();
		}
	}
	return true;
}


inline
utils::Vector2d RobotForces::getObstacleForce() const
{
	if (laserPoints==0 && xtionPoints==0) {
		return utils::Vector2d::Zero();
	}
	return (laserForceSum + xtionForceSum)/(double)(laserPoints + xtionPoints); 
}



inline
void RobotForces::computeObstacleForceSum(const sensor_msgs::LaserScan::ConstPtr& scan, utils::Vector2d& force, unsigned& points, std::vector<utils::Vector2d>& obstacles, double& min) const
{
	force = utils::Vector2d::Zero();
	obstacles.clear();
	points=0;		
	utils::Angle alpha = robotYaw + utils::Angle::fromRadian(scan->angle_min);
	utils::Angle alpha_inc = utils::Angle::fromRadian(scan->angle_increment);	
	min = 9999999;
	for (unsigned i=0; i<scan->ranges.size();i++) {
		if (!std::isnan(scan->ranges[i])) {
			if (scan->ranges[i]<1.0) {
				obstacles.emplace_back(scan->ranges[i]*alpha.cos(),scan->ranges[i]*alpha.sin());
			}
			if (scan->ranges[i]<min) {
				min = scan->ranges[i];
			}
			utils::Vector2d diff(-scan->ranges[i]*alpha.cos(),-scan->ranges[i]*alpha.sin());
			force += force_factor_obstacle * std::exp(-(scan->ranges[i] - robot_radius)/force_sigma_obstacle) * diff.normalized();
			points++;
		}
		alpha+=alpha_inc;
	}
}

inline
std_msgs::ColorRGBA RobotForces::getColor(double r, double g, double b, double a)
{
	std_msgs::ColorRGBA color;
	color.r = r;
	color.g = g;
	color.b = b;
	color.a = a;
	return color;
}


inline
void RobotForces::publishForceMarker(unsigned index, const std_msgs::ColorRGBA& color, 
					const utils::Vector2d& force, 
					visualization_msgs::MarkerArray& markers) 
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/odom";
	marker.header.stamp = ros::Time::now();
	marker.ns = "robot_forces";
	marker.id = index;
	marker.action = force.norm()>0?0:2;
	marker.color = color;
	marker.scale.x = std::max(0.0001,force.norm());
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.pose.position.x = robotPosition.getX();
	marker.pose.position.y = robotPosition.getY();
	marker.pose.position.z = 0;
	marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,force.angle().toRadian());
	markers.markers.push_back(marker);
}


inline
void RobotForces::publishMarkers()
{
	
	visualization_msgs::MarkerArray markers;
	
	publishForceMarker(0,getColor(0,0,1,1),getObstacleForce(),markers);
	publishForceMarker(1,getColor(0,1,1,1),socialForce,markers);	
	publishForceMarker(2,getColor(0,1,0,1),groupForce,markers);	
	publishForceMarker(3,getColor(1,0,0,1),desiredForce,markers);	

	markers_pub.publish(markers);

}

inline
bool RobotForces::isCollision(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	for (unsigned i=0; i<scan->ranges.size(); i++) {
		if (!std::isnan(scan->ranges[i]) && scan->ranges[i] <= collision_threshold) {
			return true;
		}
	}
	return false;
}





}

#endif
