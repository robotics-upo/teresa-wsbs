#ifndef _CMD_VEL_HPP_
#define _CMD_VEL_HPP_


#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include "forces.hpp"

namespace wsbs
{

class CmdVel
{
public:
	CmdVel(CmdVel const&) = delete;
        void operator=(CmdVel const&) = delete;
	~CmdVel() {}
	static CmdVel& getInstance()
   	{
      		static CmdVel singleton;
      		return singleton;
	}
	#define CMD_VEL CmdVel::getInstance()
	bool compute(double dt);
	const geometry_msgs::Twist& getCommand() const {return command;}
	visualization_msgs::MarkerArray& getMarkers() {return markers;}

private:
	CmdVel();
	//static double evaluate(double linVel, double angVel, double dt, const utils::Vector2d& positionRef, const utils::Angle& yawRef);
	static double evaluate(double linVel, double angVel, double distance, double dt, const utils::Vector2d& velocityRef, const utils::Angle& yawRef);
	std::vector<geometry_msgs::Twist> velocities;
	
	geometry_msgs::Twist command;
	visualization_msgs::MarkerArray markers;

	visualization_msgs::MarkerArray full_markers;
};


inline
CmdVel::CmdVel()
{

	
	static const std::vector<double> ang_vels = 
		{-0.8, -0.75, -0.7, -0.65, -0.6, -0.55, -0.5, -0.45, -0.4, 
		-0.35, -0.3, -0.25, -0.2, -0.15, -0.1, -0.05, 0, 0.05, 0.1, 
		0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8};
	
	static const std::vector<double> lin_vels = 
		{/*-0.6, -0.55, -0.5, -0.45, -0.4, -0.35, -0.3, -0.25, -0.2, -0.15, -0.1, -0.05,*/
		 0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4 /*, 0.45, 0.5, 0.55, 0.6*/};



	
	full_markers.markers.resize(ang_vels.size() * lin_vels.size());
	markers.markers.resize(ang_vels.size() * lin_vels.size());
	unsigned counter = 0;
	for (unsigned i = 0; i< lin_vels.size(); i++) {
		for (unsigned j = 0; j<ang_vels.size(); j++) {
			geometry_msgs::Twist twist;
			twist.linear.x = lin_vels[i];
			twist.linear.y = 0;
			twist.linear.z = 0;
			twist.angular.x = 0;
			twist.angular.y = 0;
			twist.angular.z = ang_vels[j];
			velocities.push_back(twist);
			full_markers.markers[counter].points.resize(30);
			Agent dummy(lin_vels[i],ang_vels[j]);
			for (unsigned k = 0; k< 30; k++) {
				full_markers.markers[counter].points[k].x = dummy.position.getX();
				full_markers.markers[counter].points[k].y = dummy.position.getY();
				full_markers.markers[counter].points[k].z = 0;
				dummy.move(0.1);
			}
			markers.markers[counter].header.frame_id="base_link";
			markers.markers[counter].ns = "trajectories";
			markers.markers[counter].id = counter;
			markers.markers[counter].type = 4;
			markers.markers[counter].action = 0;
			markers.markers[counter].scale.x = 0.01;
			counter++;
		}
	}
	
}

	
inline
double CmdVel::evaluate(double linVel, double angVel, double distance, double dt, const utils::Vector2d& velocityRef, const utils::Angle& yawRef)
{
	if (distance > FORCES.getParams().obstacleDistanceThreshold) {
		distance =  FORCES.getParams().obstacleDistanceThreshold;
	}
	Agent dummy(FORCES.getData().robot);
	dummy.linearVelocity = linVel;
	dummy.angularVelocity = angVel;
	dummy.move(dt);
	double velRef = velocityRef.getX() * FORCES.getData().robot.yaw.cos() + velocityRef.getY() * FORCES.getData().robot.yaw.sin();	
	double a = fabs(linVel - velRef)/FORCES.getParams().robotMaxLinearVelocity;
	utils::Angle angle = dummy.yaw - yawRef;
	double b = fabs(angle.toRadian())/M_PI;
	double c = 1.0 - distance/ FORCES.getParams().obstacleDistanceThreshold;
	
	return FORCES.getParams().beta_v * a + FORCES.getParams().beta_y * b + FORCES.getParams().beta_d *c;
}


/*
inline
double CmdVel::evaluate(double linVel, double angVel, double dt, const utils::Vector2d& positionRef, const utils::Angle& yawRef)
{
	Agent dummy(FORCES.getData().robot);
	dummy.linearVelocity = linVel;
	dummy.angularVelocity = angVel;
	dummy.move(dt);
	double a = dummy.position.getX() - positionRef.getX();
	double b = dummy.position.getY() - positionRef.getY();
	utils::Angle angle = dummy.yaw - yawRef;
	double c = angle.toRadian();
	
	return (a*a) + (b*b) + (c*c);
}
*/

inline
bool CmdVel::compute(double dt)
{
	bool finishing;
	utils::Vector2d velocityRef = FORCES.getData().robot.velocity + FORCES.getData().globalForce * dt;
	
	utils::Vector2d positionRef;
	utils::Angle yawRef;
	//if (FORCES.getData().targetFound) {
	//	std::cout<< "DIS: "<<(FORCES.getData().robot.position - FORCES.getData().target.position).norm()<<std::endl;
	//	std::cout<< "Vref: "<<velocityRef.norm()<<std::endl;
	//}
	if ( FORCES.getData().targetFound &&
		(FORCES.getData().robot.position - FORCES.getData().target.position).norm() <= FORCES.getParams().targetLookahead &&
		velocityRef.norm()<0.1) {
		velocityRef.set(0,0);
		positionRef = FORCES.getData().robot.position;
		yawRef = (FORCES.getData().target.position - FORCES.getData().robot.position).angle();
		finishing = true;

	} else {
		if (velocityRef.norm() > FORCES.getParams().robotMaxLinearVelocity) {
			velocityRef.normalize();
			velocityRef *= FORCES.getParams().robotMaxLinearVelocity;
		}
		positionRef = FORCES.getData().robot.position + velocityRef * dt;
		yawRef = velocityRef.angle();
		finishing = false;
	}
	command.linear.x = 0;
	command.angular.z = 0;
	double min = 999999999;
	double robot_lin_vel = FORCES.getData().robot.linearVelocity;
	double robot_ang_vel = FORCES.getData().robot.angularVelocity;

	double distance=0;
	double time = 0;
	ros::Time current_time = ros::Time::now();
	for (unsigned i=0;i<velocities.size();i++) {
		double linVel = velocities[i].linear.x;
		double angVel = velocities[i].angular.z;
		double linAcc = (linVel - robot_lin_vel)/dt;
		double angAcc = (angVel - robot_ang_vel)/dt;
		markers.markers[i].header.stamp = current_time;
		if (fabs(linAcc) > FORCES.getParams().robotMaxLinearAcceleration ||
			fabs(angAcc) > FORCES.getParams().robotMaxAngularAcceleration) {
			FORCES.checkCollision(linVel, angVel,distance,time);
			unsigned size = std::min(30u,(unsigned)std::round(time * 10.0)); 
			markers.markers[i].points.assign(full_markers.markers[i].points.begin(),full_markers.markers[i].points.begin()+size);
			markers.markers[i].color.r = 0.5;
			markers.markers[i].color.g = 0.5;
			markers.markers[i].color.b = 0.5;
			markers.markers[i].color.a = 1.0;
			continue;
		}
		if (FORCES.checkCollision(linVel, angVel,distance,time)) {
			unsigned size = std::min(30u,(unsigned)std::round(time * 10.0));
			markers.markers[i].points.assign(full_markers.markers[i].points.begin(),full_markers.markers[i].points.begin()+size);
			markers.markers[i].color.r = 0.0;
			markers.markers[i].color.g = 1.0;
			markers.markers[i].color.b = 0.0;
			markers.markers[i].color.a = 1.0;
			continue;
		}		
		unsigned size = std::min(30u,(unsigned)std::round(time * 10.0)); 
		markers.markers[i].points.assign(full_markers.markers[i].points.begin(),full_markers.markers[i].points.begin()+size);
		markers.markers[i].color.r = 0.0;
		markers.markers[i].color.g = 1.0;
		markers.markers[i].color.b = 0.0;
		markers.markers[i].color.a = 1.0;
		//double value = evaluate(linVel, angVel, dt, positionRef, yawRef);
		double value = evaluate(linVel, angVel, distance, dt, velocityRef, yawRef);
		

		if (value < min) {
			min = value;
			command.linear.x = linVel;
			command.angular.z = angVel;
		}
	}
	
	return finishing;
	
	
}




}

#endif
