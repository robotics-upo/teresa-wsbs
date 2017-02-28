#ifndef _COMMON_HPP_
#define _COMMON_HPP_

#include <tinyxml.h>
#include <tf/transform_listener.h>
#include <lightsfm/vector2d.hpp>
#include <lightsfm/astar.hpp>
#include <lightsfm/sfm.hpp>

namespace wsbs
{

class TfListener
{
public:
	TfListener(TfListener const&) = delete;
	void operator=(TfListener const&) = delete;
	~TfListener() {}

	static TfListener& getInstance()
   	{
      		static TfListener singleton;
      		return singleton;
	}

	#define TF TfListener::getInstance()

	bool transformPose(double& x, double& y, double& theta, const std::string& sourceFrameId, const std::string& targetFrameId) const
	{
		tf::Stamped<tf::Pose> pose,tfPose;
		pose.setData(tf::Pose(tf::createQuaternionFromRPY(0,0,theta), tf::Vector3(x,y,0)));
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
		tf::Matrix3x3 m(tfPose.getRotation());
		double roll,pitch;
		m.getRPY(roll, pitch, theta);
		return true;
	}

	bool transformPoint(double& x, double& y, const std::string& sourceFrameId, const std::string& targetFrameId) const
	{
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
	bool transformPoint(utils::Vector2d& point, const std::string& sourceFrameId, const std::string& targetFrameId) const
	{
		double x = point.getX();
		double y = point.getY();
		if (transformPoint(x,y,sourceFrameId,targetFrameId)) {
			point.set(x,y);
			return true;
		}
		return false;
	}


private:
	TfListener()  {}
	tf::TransformListener tf_listener;
};



// Controller Mode
enum ControllerMode {
	HEURISTIC	= 0,
	LEFT		= 1,
	RIGHT		= 2,
	BEHIND		= 3,
	FOLLOW_PATH	= 4,
	WAIT		= 5,
	SET_GOAL	= 6,
	SET_FINAL_GOAL  = 7
};




class PathProvider
{
public:
	PathProvider() {}
	virtual ~PathProvider() {}
	const std::vector<utils::Vector2d>& getGoals() const {return goals;}
	virtual utils::Vector2d& getNextPoint(const utils::Vector2d& position, const utils::Vector2d& goal, utils::Vector2d& nextPoint)
	{
		nextPoint = goal;
		return nextPoint;
	}

protected:
	std::vector<utils::Vector2d> goals;	
};



class AStarPathProvider : public PathProvider
{
public:
	AStarPathProvider(const std::string& file)
	: aStar(file)
	{
		double x,y,r;
		for (unsigned i=0; i< aStar.getGoals().size();i++) {
			aStar.getPos(aStar.getGoals()[i], x, y, r);
			goals.emplace_back(x,y);
		}
	}
	virtual ~AStarPathProvider() {}
	virtual utils::Vector2d& getNextPoint(const utils::Vector2d& position, const utils::Vector2d& goal, utils::Vector2d& nextPoint)
	{
		std::string start_id,goal_id;
		aStar.getClosestNode(position.getX(),position.getY(),start_id);
		aStar.getClosestNode(goal.getX(),goal.getY(),goal_id);
		std::list<std::string> path;		
		aStar.getPath(start_id,goal_id,path);
		if (path.size()<2) {
			nextPoint = goal;
			return nextPoint;
		}
		double x,y,r;
		auto it = path.begin();
		++it;
		aStar.getPos(*it,x,y,r);
		nextPoint.set(x,y);
		return nextPoint;
	}
private:
	utils::AStar aStar;
};



class GoalProvider
{
public:
	GoalProvider(double threshold, unsigned size, double lookahead, double naiveGoalTime, double wsbsDistance, const std::string& frame, PathProvider& pathProvider, bool allowHistory=true);
	~GoalProvider() {}

	void update(const utils::Vector2d& robotPos, const utils::Vector2d& targetPos, const utils::Vector2d& targetVel, const utils::Vector2d& targetGlobalGoal);
	void update(const utils::Vector2d& robotPos, const utils::Vector2d& targetPos, const utils::Vector2d& targetVel);
	void update(const utils::Vector2d& robotPos);
	const utils::Vector2d& getRobotLocalGoal(const ControllerMode& mode) const;
	const std::vector<utils::Vector2d>& getGoals() const {return pathProvider.getGoals();}	
	PathProvider& getPathProvider() {return pathProvider;}
	
private:
	double threshold;
	unsigned size;
	double lookahead;
	double naiveGoalTime;
	double wsbsDistance;
	std::string frame;
	PathProvider& pathProvider;
	bool allowHistory;
	utils::Vector2d rightGoal;
	utils::Vector2d leftGoal;
	utils::Vector2d behindGoal;
	utils::Vector2d followGoal;
	utils::Vector2d waitGoal;
	utils::Vector2d heuristicGoal;
	utils::Vector2d targetNaiveGoal;
	utils::Vector2d robotPos;
	utils::Vector2d targetGlobalGoal;
	utils::Vector2d robotLocalGoal;
	std::list<utils::Vector2d> history;

};



GoalProvider::GoalProvider(double threshold, unsigned size, double lookahead, double naiveGoalTime, double wsbsDistance, const std::string& frame, PathProvider& pathProvider, bool allowHistory)
: threshold(threshold),
  size(size),
  lookahead(lookahead),
  naiveGoalTime(naiveGoalTime),
  wsbsDistance(wsbsDistance),
  frame(frame),
  pathProvider(pathProvider),
  allowHistory(allowHistory)
{
}

inline
void GoalProvider::update(const utils::Vector2d& robotPos)
{
	leftGoal = robotPos;
	rightGoal = robotPos;
	behindGoal = robotPos;
	waitGoal = robotPos;
	followGoal = robotPos;
	GoalProvider::robotPos = robotPos;
	
	if (allowHistory) {
		for (auto it = history.begin(); it!= history.end(); ++it) {
			if ((robotPos - *it).norm() <= lookahead) {
				followGoal = *it;
				history.erase(++it,history.end());
				break;	
			}
		}
		heuristicGoal = followGoal;
	} else {
		heuristicGoal = robotPos;
	}
}


inline
void GoalProvider::update(const utils::Vector2d& robotPos, const utils::Vector2d& targetPos, const utils::Vector2d& targetVel)
{
	if (targetVel.norm()>0.05) {
		targetNaiveGoal = targetPos + naiveGoalTime * targetVel;
		leftGoal =  targetNaiveGoal +  wsbsDistance*targetVel.normalized().leftNormalVector();
		rightGoal =  targetNaiveGoal +  wsbsDistance*targetVel.normalized().rightNormalVector();				
		behindGoal = targetNaiveGoal - wsbsDistance*targetVel.normalized();
		
	}
	waitGoal = robotPos;
	GoalProvider::robotPos = robotPos;
	
	
	if (allowHistory && (history.empty() || (targetPos - history.front()).norm() > threshold)) {
		history.push_front(targetPos);
		if (history.size()>size) {
			auto it = history.rbegin();
			++it;
			if ((robotPos - *it).norm() <= lookahead) {
				history.pop_back();
			} else {
				history.pop_front();
			}
		}
	}
	followGoal = waitGoal;
	for (auto it = history.begin(); it!= history.end(); ++it) {
		if ((robotPos - *it).norm() <= lookahead) {
			followGoal = *it;
			history.erase(++it,history.end());
			break;	
		}
	}
	if ((robotPos - targetPos).norm() <= lookahead) {
		if ((rightGoal - robotPos).squaredNorm() <
			 (leftGoal - robotPos).squaredNorm()) {
			heuristicGoal = rightGoal;
		} else {
			heuristicGoal = leftGoal;
		}
	} else {
		heuristicGoal = followGoal;
	}
}

void GoalProvider::update(const utils::Vector2d& robotPos, const utils::Vector2d& targetPos, const utils::Vector2d& targetVel, const utils::Vector2d& targetGlobalGoal)
{
	update(robotPos,targetPos,targetVel);
	GoalProvider::targetGlobalGoal = targetGlobalGoal;
	utils::Vector2d pos = robotPos;
	if (TF.transformPoint(pos,frame,"map")) {
		utils::Vector2d next;
		pathProvider.getNextPoint(pos,targetGlobalGoal,next);
		if (TF.transformPoint(next,"map",frame)) {
			robotLocalGoal = next;
		}
	}  
	
}

inline
const utils::Vector2d& GoalProvider::getRobotLocalGoal(const ControllerMode& mode) const
{
	switch(mode)
	{
		case HEURISTIC:
			return heuristicGoal;

		case LEFT:
			return leftGoal;

		case RIGHT:
			return rightGoal;

		case BEHIND:
			return behindGoal;

		case FOLLOW_PATH:
			return followGoal;

		case WAIT:
			return waitGoal;

		case SET_GOAL:
			return robotLocalGoal;

		case SET_FINAL_GOAL:
			return targetGlobalGoal;	
	}
	return waitGoal;
}


}

#endif
