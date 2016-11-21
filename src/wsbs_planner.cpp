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
#include <teresa_wsbs/start.h>
#include <teresa_wsbs/stop.h>
#include <vector>

namespace wsbs
{

class Planner
{
public:
	Planner(ros::NodeHandle& n, ros::NodeHandle& pn);
	~Planner() {}

private:
	bool start(teresa_wsbs::start::Request &req, teresa_wsbs::start::Response &res);
	bool stop(teresa_wsbs::stop::Request &req, teresa_wsbs::stop::Response &res);

	void readGoals(TiXmlNode *pParent);
	std::vector<sfm::Goal> goals;
	
	static ros::ServiceClient controller_start, controller_stop;

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

	ros::ServiceServer start_srv = n.advertiseService("/wsbs/start", &Controller::start,this);
	ros::ServiceServer stop_srv  = n.advertiseService("/wsbs/stop", &Controller::stop,this);
	
	controller_start = n.serviceClient<teresa_driver::Teresa_leds>("teresa_leds");	
	controller_stop = n.serviceClient<teresa_driver::Teresa_leds>("teresa_leds");	

}


bool Planner::start(teresa_wsbs::start::Request &req, teresa_wsbs::start::Response &res) 
{
	


	return true;
}


bool Planner::stop(teresa_wsbs::stop::Request &req, teresa_wsbs::stop::Response &res)
{
	
	return true;
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
	ros::init(argc, argv, "wsbs_planner");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	wsbs::Planner node(n,pn);
	return 0;
}
