/***********************************************************************/
/**                                                                    */
/** wsbs_analysis.cpp                                                  */
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
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>


namespace wsbs
{

class Analysis
{
public:
	Analysis(ros::NodeHandle& n, ros::NodeHandle& pn);
	~Analysis() {}

private:
	void odomReceived(const nav_msgs::Odometry::ConstPtr& odom);
	void joyCmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel);
	void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel);

	double maxPositiveLinAcc;
	double maxNegativeLinAcc;
	
	double maxPositiveAngAcc;
	double maxNegativeAngAcc;

	double cmdMaxLinAcc;
	double cmdMaxAngAcc;

	nav_msgs::Odometry robot;
	geometry_msgs::Twist cmd_vel_joy;
	ros::Time cmd_vel_time;
	
	bool odom_first_time;
	bool cmd_vel_first_time;
	

};

Analysis::Analysis(ros::NodeHandle& n, ros::NodeHandle& pn)
: maxPositiveLinAcc(0),
  maxNegativeLinAcc(0),
  maxPositiveAngAcc(0),
  maxNegativeAngAcc(0),
  cmdMaxLinAcc(0),
  cmdMaxAngAcc(0), 
  odom_first_time(true),
  cmd_vel_first_time(true)
{
	std::string odom_id, cmd_vel_joy_id, cmd_vel_id;

	pn.param<std::string>("odom_id",odom_id,"/odom");
	pn.param<std::string>("cmd_vel_joy_id",cmd_vel_joy_id,"/cmd_vel_joy");
	pn.param<std::string>("cmd_vel_id",cmd_vel_id,"/cmd_vel");

	ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>(odom_id, 1, &Analysis::odomReceived,this);
	ros::Subscriber cmd_vel_joy_sub = n.subscribe<geometry_msgs::Twist>(cmd_vel_joy_id, 1, &Analysis::joyCmdVelReceived,this);
	ros::Subscriber cmd_vel_sub = n.subscribe<geometry_msgs::Twist>(cmd_vel_id, 1, &Analysis::cmdVelReceived,this);

	ros::spin();

}


void Analysis::odomReceived(const nav_msgs::Odometry::ConstPtr& odom)
{
	/*if (!odom_first_time) {
		double diffLinVel = odom->twist.twist.linear.x - robot.twist.twist.linear.x;
		double diffAngVel = odom->twist.twist.angular.z - robot.twist.twist.angular.z;
		double diffTime = (odom->header.stamp - robot.header.stamp).toSec();
		double linAcc = diffLinVel / diffTime;
		double angAcc = diffAngVel / diffTime;		
		if (linAcc < 0 && linAcc < maxNegativeLinAcc) {
			maxNegativeLinAcc = linAcc;
			ROS_INFO("Max negative linear acceleration: %f m/s^2",maxNegativeLinAcc);
		}
		if (linAcc > 0 && linAcc > maxPositiveLinAcc) {
			maxPositiveLinAcc = linAcc;
			ROS_INFO("Max positive linear acceleration: %f m/s^2",maxPositiveLinAcc);
		}
		if (angAcc < 0 && angAcc < maxNegativeAngAcc) {
			maxNegativeAngAcc = angAcc;
			ROS_INFO("Max negative angular acceleration: %f rad/s^2",maxNegativeAngAcc);
		}
		if (angAcc > 0 && angAcc > maxPositiveAngAcc) {
			maxPositiveAngAcc = angAcc;
			ROS_INFO("Max positive angular acceleration: %f rad/s^2",maxPositiveAngAcc);
		}
	}
	*/
	odom_first_time = false;
	robot = *odom;
}

void Analysis::joyCmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
	ros::Time now = ros::Time::now();
	if (!cmd_vel_first_time) {
		
		double diffLinVel = cmd_vel->linear.x - cmd_vel_joy.linear.x;
		double diffAngVel = cmd_vel->angular.z - cmd_vel_joy.angular.z;
		double diffTime = (now - cmd_vel_time).toSec();
		double linAcc = diffLinVel / diffTime;
		double angAcc = diffAngVel / diffTime;		
		if (fabs(linAcc) > cmdMaxLinAcc) {
			cmdMaxLinAcc = fabs(linAcc);
			ROS_INFO("CMD_VEL Max linear acceleration: %f m/s^2",cmdMaxLinAcc);
		}
		
		if (fabs(angAcc) > cmdMaxAngAcc) {
			cmdMaxAngAcc = fabs(angAcc);
			ROS_INFO("CMD_VEL Max angular acceleration: %f rad/s^2",cmdMaxAngAcc);
		}
		
	}
	cmd_vel_time = now;
	cmd_vel_first_time = false;
	cmd_vel_joy = *cmd_vel;

	//ROS_INFO("Cmd_vel: %f %f - %f %f",cmd_vel->linear.x, cmd_vel->angular.z, robot.twist.twist.linear.x, robot.twist.twist.angular.z);
	

}

void Analysis::cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{


}



}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "wsbs_analysis");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	wsbs::Analysis node(n,pn);
	return 0;
}
