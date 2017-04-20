#ifndef _CMD_VEL_HPP_
#define _CMD_VEL_HPP_


#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
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
	
	/*static const std::vector<double> ang_vels = 
		{-1.0, -0.95, -0.9, -0.85, -0.8, -0.75, -0.7, -0.65, -0.6, -0.55, -0.5, -0.45, -0.4, 
		-0.35, -0.3, -0.25, -0.2, -0.15, -0.1, -0.05, 0, 0.05, 0.1, 
		0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1.0};*/


	/*static const std::vector<double> ang_vels = 
		{-1.5, -1.45, -1.4, -1.35, -1.3, -1.25, -1.2, -1.15, -1.1, -1.05, -1, -0.95, -0.9, -0.85, -0.8, -0.75, -0.7, -0.65, -0.6, -0.55, -0.5, -0.45, -0.4, 
		-0.35, -0.3, -0.25, -0.2, -0.15, -0.1, -0.05, 0, 0.05, 0.1, 
		0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1, 1.05, 1.1, 1.15, 1.2, 1.25, 1.3, 1.35, 1.4, 1.45, 1.5};*/

	static const std::vector<double> lin_vels = 
		{ 0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6 /*, 0.65, 0.7, 0.75, 0.8*/};



	
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
	static bool finishing=false;
	utils::Vector2d velocityRef = FORCES.getData().robot.velocity + FORCES.getData().globalForce * dt;
	utils::Vector2d positionRef;
	static utils::Angle yawRef;
	command.linear.x = 0;
	command.angular.z = 0;
	//if (FORCES.getData().targetFound) {
	//	std::cout<< "DIS: "<<(FORCES.getData().robot.position - FORCES.getData().target.position).norm()<<std::endl;
	//	std::cout<< "Vref: "<<velocityRef.norm()<<std::endl;
	//}
	//std::cout<<"TARGET FOUND: "<<FORCES.getData().targetFound<<std::endl;
	//std::cout<<"DISTANCE TO TARGET: "<<(FORCES.getData().robot.position - FORCES.getData().target.position).norm()<<std::endl;
	//std::cout<<"TARGET VEL: "<<FORCES.getData().target.velocity.norm()<<std::endl;
	//std::cout<<"Vref: "<<velocityRef.norm()<<std::endl;
	//std::cout<<"History size: "<<FORCES.getData().targetHistory.size()<<std::endl;
	if (!FORCES.getData().targetFound && !FORCES.getData().validGoal) {
		return false;
	} else if ( FORCES.getData().targetFound && FORCES.getData().target.velocity.norm()<0.2 &&
		(FORCES.getData().robot.position - FORCES.getData().target.position).norm() <= 1.0/* &&
		velocityRef.norm()<0.2*/) {
		//std::cout<<"FINISHING"<<std::endl;
		if (!finishing) {
			yawRef = (FORCES.getData().target.position - FORCES.getData().robot.position).angle();
		}
		utils::Angle diff = FORCES.getData().robot.yaw - yawRef;
		if (std::abs(diff.toDegree())<15) {
			return true;
		} else if (diff.sign()>0) {
			command.linear.x = 0;
			command.angular.z=-0.8;
			return true;
		} else {
			command.linear.x = 0;
			command.angular.z=0.8;
			return true;
		}
		
		/*		
		velocityRef.set(0,0);
		positionRef = FORCES.getData().robot.position;
		if (!finishing) {
			yawRef = (FORCES.getData().target.position - FORCES.getData().robot.position).angle();
		}
		finishing = true;
		*/
		
	} else {
		if (velocityRef.norm() > FORCES.getParams().robotMaxLinearVelocity) {
			velocityRef.normalize();
			velocityRef *= FORCES.getParams().robotMaxLinearVelocity;
		}
		positionRef = FORCES.getData().robot.position + velocityRef * dt;
		yawRef = velocityRef.angle();
		finishing = false;
	}
	
	double min = 999999999;
	double robot_lin_vel = FORCES.getData().robot.linearVelocity;
	double robot_ang_vel = FORCES.getData().robot.angularVelocity;

	int i_min=-1;

	double distance=0;
	double time = 0;
	ros::Time current_time = ros::Time::now();
	for (unsigned i=0;i<velocities.size();i++) {
		double linVel = velocities[i].linear.x;
		double angVel = velocities[i].angular.z;
		double linAcc = (linVel - robot_lin_vel)/dt;
		double angAcc = (angVel - robot_ang_vel)/dt;
		markers.markers[i].header.stamp = current_time;
		markers.markers[i].lifetime = ros::Duration(1.0);
		if (fabs(linAcc) > FORCES.getParams().robotMaxLinearAcceleration ||
			fabs(angAcc) > FORCES.getParams().robotMaxAngularAcceleration) {
			FORCES.checkCollision(linVel, angVel,distance,time);
			unsigned size = std::min(30u,(unsigned)std::round(time * 10.0)); 
			markers.markers[i].points.assign(full_markers.markers[i].points.begin(),full_markers.markers[i].points.begin()+size);
			markers.markers[i].color.r = 0.5;
			markers.markers[i].color.g = 0.5;
			markers.markers[i].color.b = 0.5;
			markers.markers[i].color.a = 1.0;
			markers.markers[i].scale.x = 0.01;
			//std::cout << "Limits" << std::endl;
			continue;
		}
		if (FORCES.checkCollision(linVel, angVel,distance,time)) {
			unsigned size = std::min(30u,(unsigned)std::round(time * 10.0));
			markers.markers[i].points.assign(full_markers.markers[i].points.begin(),full_markers.markers[i].points.begin()+size);
			markers.markers[i].color.r = 1.0;
			markers.markers[i].color.g = 0.0;
			markers.markers[i].color.b = 0.0;
			markers.markers[i].color.a = 1.0;
			markers.markers[i].scale.x = 0.01;
			//std::cout << "Collision" << std::endl;
			continue;
		}		
		unsigned size = std::min(30u,(unsigned)std::round(time * 10.0)); 
		markers.markers[i].points.assign(full_markers.markers[i].points.begin(),full_markers.markers[i].points.begin()+size);
		markers.markers[i].color.r = 0.0;
		markers.markers[i].color.g = 1.0;
		markers.markers[i].color.b = 0.0;
		markers.markers[i].color.a = 1.0;
		markers.markers[i].scale.x = 0.01;
		//double value = evaluate(linVel, angVel, dt, positionRef, yawRef);
		double value = evaluate(linVel, angVel, distance, dt, velocityRef, yawRef);
		
		markers.markers[i].color.g = value;

		if (value < min) {
			min = value;
			command.linear.x = linVel;
			command.angular.z = angVel;
			i_min = i;
		}
	}

	if(i_min>=0)
	{
		markers.markers[i_min].color.r = 0.0;
		markers.markers[i_min].color.g = 0.0;
		markers.markers[i_min].color.b = 1.0;
		markers.markers[i_min].color.a = 1.0;
		markers.markers[i_min].scale.x = 0.1;
	}

	
	return finishing;
	
	
}




}

#endif
