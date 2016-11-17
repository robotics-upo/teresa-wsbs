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
#include <nav_msgs/GetPlan.h>
#include <tinyxml.h>
#include <lightsfm/sfm.hpp>
#include <vector>

namespace wsbs
{

class Planner
{
public:
	Planner(ros::NodeHandle& n, ros::NodeHandle& pn);
	~Planner() {}

private:
	bool makePlan(const utils::Vector2d& start, const utils::Vector2d& goal, double tolerance, std::vector<utils::Vector2d>& plan);
	void readGoals(TiXmlNode *pParent);
	std::vector<sfm::Goal> goals;
	ros::ServiceClient plan_client;

};


inline
Planner::Planner(ros::NodeHandle& n, ros::NodeHandle& pn)
{
	std::string goals_file;

	pn.param<std::string>("goals_file",goals_file,"");

	TiXmlDocument xml_doc(goals_file);
	if (xml_doc.LoadFile()) {
		readGoals(&xml_doc);
		if (goals.empty()) {
			ROS_FATAL("No goals");
			ROS_BREAK();
		}
	} else {
		ROS_FATAL("Cannot read goals");
		ROS_BREAK();
	}

	plan_client = n.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
	utils::Vector2d start;
	start.set(6,26);	
	std::vector<utils::Vector2d> plan;
	ros::Rate r(1.0);
	while (!makePlan(start,goals[0].center,1,plan)) {
		ROS_ERROR("Error getting plan");
		r.sleep();	

	}
	for (unsigned i=0;i<plan.size();i++) {
		std::cout<<plan[i]<<std::endl;
	}

}

inline
bool Planner::makePlan(const utils::Vector2d& start, const utils::Vector2d& goal, double tolerance, std::vector<utils::Vector2d>& plan)
{
	nav_msgs::GetPlan query;


	query.request.start.header.frame_id="map";
	query.request.start.pose.position.x = start.getX();
	query.request.start.pose.position.y = start.getY();
	query.request.start.pose.position.z = 0;
	
	query.request.goal.header.frame_id="map";
	query.request.goal.pose.position.x = goal.getX();
	query.request.goal.pose.position.y = goal.getY();
	query.request.goal.pose.position.z = 0;
	query.request.tolerance = tolerance;
	bool success=plan_client.call(query);
	if (success) {
		std::cout<<"SIZE: "<<query.response.plan.poses.size()<<std::endl;
		plan.clear();
		for (unsigned i = 0; i< query.response.plan.poses.size(); i++) {
			utils::Vector2d point;
			std::cout<<query.response.plan.poses[i].header.frame_id<<std::endl;
			point.set(query.response.plan.poses[i].pose.position.x,query.response.plan.poses[i].pose.position.y);
			plan.push_back(point);
		}
	}

	return success;
}


inline
void Planner::readGoals(TiXmlNode *pParent)
{
	if ( !pParent ) return;

	if ( pParent->Type() == TiXmlNode::TINYXML_ELEMENT) {
		TiXmlElement* pElement = pParent->ToElement();
		TiXmlAttribute* pAttrib=pElement->FirstAttribute();
		if (strcmp(pParent->Value(),"goal")==0) {
			sfm::Goal goal;
			double x,y;
			std::string id;
			id.assign(pAttrib->Value());
			pAttrib=pAttrib->Next();
			pAttrib->QueryDoubleValue(&x);
			pAttrib=pAttrib->Next();
			pAttrib->QueryDoubleValue(&y);
			pAttrib=pAttrib->Next();
			pAttrib->QueryDoubleValue(&goal.radius);
			goal.center.set(x,y);
			goals.push_back(goal);
		} 
	}

	for (TiXmlNode* pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling()) {
		readGoals( pChild );
	}

}



}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "wsbs_controller");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	wsbs::Planner node(n,pn);
	return 0;
}
