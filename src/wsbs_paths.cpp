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
#include <lightsfm/rosmap.hpp>
#include <vector>
#include <map>
#include <fstream>

namespace wsbs
{

struct PathIndex
{
	unsigned x; // Cell x
	unsigned y; // Cell y
	unsigned goal_index; 

	bool operator<(const PathIndex& other) const
	{
		return x < other.x ||
			(x==other.x && y< other.y) ||
			(x==other.x && y==other.y && goal_index<other.goal_index);
	}
};



class Paths
{
public:
	Paths(ros::NodeHandle& n, ros::NodeHandle& pn);
	~Paths() {}

private:
	bool makePlan(const utils::Vector2d& start, const utils::Vector2d& goal, double tolerance, std::vector<utils::Vector2d>& plan);
	void readGoals(TiXmlNode *pParent);
	std::vector<sfm::Goal> goals;
	ros::ServiceClient plan_client;

};


inline
Paths::Paths(ros::NodeHandle& n, ros::NodeHandle& pn)
{
	std::string goals_file,paths_file;
	double grid_size;

	pn.param<std::string>("goals_file",goals_file,"");
	pn.param<double>("grid_size",grid_size,0.5);
	pn.param<std::string>("paths_file",paths_file,"/home/ignacio/paths.txt");

	std::ofstream file;
	file.open(paths_file);

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
	sfm::RosMap::Pixel pixel;
	PathIndex index;
	index.x=0;
	index.y=0;
	index.goal_index=0;
	unsigned counter=0;
	std::map<PathIndex,std::vector<utils::Vector2d> > paths;
	unsigned step = (unsigned)std::round(grid_size/sfm::MAP.getInfo().resolution);
	
	for (unsigned i=0; i<sfm::MAP.getInfo().width; i+=step) {
		index.y=0;
		for (unsigned j=0; j< sfm::MAP.getInfo().height; j+=step) {
			pixel.x = (int)i;
			pixel.y = (int)j;
			sfm::MAP.pixelToMap(pixel,start);
			if (sfm::MAP.isObstacle(start) || sfm::MAP.getNearestObstacle(start).distance< grid_size) {
				continue;
			}
			for (unsigned k=0; k< goals.size(); k++) {
				index.goal_index=k;				
				makePlan(start,goals[k].center,grid_size,paths[index]);
				if (paths[index].size()>0) {				
					std::cout<<"--- "<<(++counter)<<" ---"<<std::endl;				
					std::cout<<"START: "<<start<<std::endl;
					std::cout<<"GOAL: "<<goals[k].center<<std::endl;
					std::cout<<"X: "<<index.x<<std::endl;
					std::cout<<"Y: "<<index.y<<std::endl;
					std::cout<<"K: "<<index.goal_index<<std::endl;
					std::cout<<"PATH SIZE: "<<paths[index].size()<<std::endl;
					double size = 0;
					for (unsigned i=1; i< paths[index].size(); i++) {
						size += (paths[index][i] - paths[index][i-1]).norm();
						if (size >= grid_size) {
							std::cout<<"Local goal: "<<paths[index][i]<<std::endl;
							
							file<<start.getX()<<" "<<start.getY()<<" "<<goals[k].center.getX()<<" "<<goals[k].center.getY()<<" "<<
								paths[index][i].getX()<<" "<<paths[index][i].getY()<<std::endl;
							break;
						}
					}
				}
			}
			index.y++;
		}
		index.x++;
	}
	file.close();	

}

inline
bool Paths::makePlan(const utils::Vector2d& start, const utils::Vector2d& goal, double tolerance, std::vector<utils::Vector2d>& plan)
{
	std::cout<<"New query: "<<start<<" -> "<<goal<<std::endl;
	nav_msgs::GetPlan query;

	query.request.start.header.stamp=ros::Time(0);
	query.request.start.header.frame_id="map";
	query.request.start.pose.position.x = start.getX();
	query.request.start.pose.position.y = start.getY();
	query.request.start.pose.position.z = 0;
	
	query.request.goal.header.stamp=ros::Time(0);	
	query.request.goal.header.frame_id="map";
	query.request.goal.pose.position.x = goal.getX();
	query.request.goal.pose.position.y = goal.getY();
	query.request.goal.pose.position.z = 0;
	query.request.tolerance = tolerance;
	bool success=plan_client.call(query);
	if (success) {
		plan.clear();
		for (unsigned i = 0; i< query.response.plan.poses.size(); i++) {
			utils::Vector2d point(query.response.plan.poses[i].pose.position.x,query.response.plan.poses[i].pose.position.y);
			plan.push_back(point);
		}
	}

	return success;
}


inline
void Paths::readGoals(TiXmlNode *pParent)
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
	ros::init(argc, argv, "wsbs_paths");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	wsbs::Paths node(n,pn);
	return 0;
}
