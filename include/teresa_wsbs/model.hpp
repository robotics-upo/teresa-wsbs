#ifndef _MODEL_HPP_
#define _MODEL_HPP_

#include <tf/transform_listener.h>
#include <lightpomcp/MonteCarloSimulator.hpp>
#include <lightpomcp/Random.hpp>
#include <boost/functional/hash.hpp>
#include <lightsfm/sfm.hpp>
#include <lightsfm/map.hpp>
#include <lightsfm/rosmap.hpp>
#include <fstream>
#include <teresa_wsbs/astar.hpp>


namespace wsbs
{

namespace model
{


struct State
{
	utils::Vector2d robot_pos;
	utils::Vector2d robot_vel;
	utils::Vector2d target_pos;
	utils::Vector2d target_vel;
	utils::Vector2d goal;
	
	
	bool operator == (const State& other) const
	{
		return robot_pos == other.robot_pos && 
			robot_vel == other.robot_vel && 
			target_pos == other.target_pos && 
			target_vel == other.target_vel && 
			goal == other.goal;
	}
};

struct Observation
{
	int robot_pos_grid_x;
	int robot_pos_grid_y;
	int target_pos_grid_x;
	int target_pos_grid_y;
	bool target_hidden;

	bool operator == (const Observation& other) const
	{
		return robot_pos_grid_x == other.robot_pos_grid_x &&
			robot_pos_grid_y == other.robot_pos_grid_y &&
			target_pos_grid_x == other.target_pos_grid_x &&
			target_pos_grid_y == other.target_pos_grid_y &&
			target_hidden == other.target_hidden;
	}
	
};


}
}

namespace std
{


template<>
struct hash<wsbs::model::State>
{
	size_t operator()(const wsbs::model::State& state) const
	{
		using boost::hash_value;
      		using boost::hash_combine;
		std::size_t seed = 0;		
		hash_combine(seed,hash_value(state.robot_pos[0]));
		hash_combine(seed,hash_value(state.robot_pos[1]));
		hash_combine(seed,hash_value(state.robot_vel[0]));
		hash_combine(seed,hash_value(state.robot_vel[1]));
		hash_combine(seed,hash_value(state.target_pos[0]));
		hash_combine(seed,hash_value(state.target_pos[1]));
		hash_combine(seed,hash_value(state.target_vel[0]));
		hash_combine(seed,hash_value(state.target_vel[1]));
		hash_combine(seed,hash_value(state.goal[0]));
		hash_combine(seed,hash_value(state.goal[1]));
		return seed;
	}

};

template<>
struct hash<wsbs::model::Observation>
{
	size_t operator()(const wsbs::model::Observation& observation) const
	{
		using boost::hash_value;
      		using boost::hash_combine;
		std::size_t seed = 0;		
		hash_combine(seed,hash_value(observation.robot_pos_grid_x));
		hash_combine(seed,hash_value(observation.robot_pos_grid_y));
		hash_combine(seed,hash_value(observation.target_pos_grid_x));
		hash_combine(seed,hash_value(observation.target_pos_grid_y));
		hash_combine(seed,hash_value(observation.target_hidden));
		return seed;
	}

};

inline
ostream& operator<<(ostream& stream, const wsbs::model::Observation& obs)
{
	stream<<obs.target_pos_grid_x<<" "<<obs.target_pos_grid_y<<" - "<<obs.robot_pos_grid_x<<" "<<obs.robot_pos_grid_y;
	return stream;
}


}

namespace wsbs
{
namespace model
{

enum Action {
	LEFT		= 0,
	RIGHT		= 1,
	BEHIND		= 2,
	FOLLOW_PATH	= 3,
	WAIT		= 4,
	SET_GOAL	= 5
};

class PathProvider
{
public:
	PathProvider() {}
	virtual ~PathProvider() {}

	virtual utils::Vector2d& getNextPoint(const utils::Vector2d& position, const utils::Vector2d& goal, utils::Vector2d& nextPoint)
	{
		nextPoint = goal;
		return nextPoint;
	}	
};


class FilePathProvider : public PathProvider
{
public:
	FilePathProvider(const std::string& file, double gridSize)
	: gridSize(gridSize)
	{
		std::ifstream in(file);
		std::string line;
		double start_x,start_y,goal_x,goal_y,local_x,local_y;
		utils::Vector2d start,goal,local;
		if (in.is_open()) {
			while ( getline (in,line) ) {
      				sscanf(line.c_str(),"%lf %lf %lf %lf %lf %lf",&start_x,&start_y,&goal_x,&goal_y,&local_x,&local_y);
				start.set(start_x,start_y);
				goal.set(goal_x,goal_y);
				local.set(local_x,local_y);
				data[start][goal] = local;
			}
			in.close();
		} 
	}
	virtual ~FilePathProvider() {}

	virtual utils::Vector2d& getNextPoint(const utils::Vector2d& position, const utils::Vector2d& goal, utils::Vector2d& nextPoint)
	{
		auto it = data.lower_bound(position);
		if (it == data.end() || it->second.count(goal)==0) {
			nextPoint = goal;
		} else {
			nextPoint =  it->second.at(goal);
			if ((nextPoint - position).norm() > 2*gridSize) {
				nextPoint = goal;
			}
		}
		return nextPoint;
		

	}

private:
	double gridSize;
	std::map<utils::Vector2d, std::unordered_map<utils::Vector2d, utils::Vector2d> > data;
	

};


class AStarPathProvider : public PathProvider
{
public:
	AStarPathProvider()
	{
		/*
		aStar.addNode("A",7,32);
		aStar.addNode("B",27,7);
		aStar.addNode("C",52,7);
		aStar.addNode("D",27,32);
		aStar.addNode("E",52,32);
		aStar.addEdge("A","D");
		aStar.addEdge("D","E");
		aStar.addEdge("D","B");
		aStar.addEdge("E","C");
		*/

		/*
		aStar.addNode("rest_area",38.58,51.56);
		aStar.addNode("vending_machine",34.99,52.40);
		aStar.addNode("toilette","32.13","55.36");	
		aStar.addNode("water_font",29,52.44);
		aStar.addNode("reception" ,6.71,52.56);
		aStar.addNode("robotics_lab" ,12.29,46.43);
		aStar.addNode("classroom_1" ,26.72,47.75);
		aStar.addNode("classroom_2_1" ,26.83,41.33);
		aStar.addNode("classroom_2_2" ,26.78,39.43);
		aStar.addNode("classroom_3_1" ,26.7,33.05);
		aStar.addNode("classroom_3_2" ,26.56,31.40);
		aStar.addNode("exit_1" ,7.84,31.96);
		aStar.addNode("exit_2" ,24.39,26.63);
		aStar.addNode("coffe_area" ,32.34,51.15);
		aStar.addNode("enter_coffe_area" ,28.73,50.55);
		aStar.addNode("enter_corridor_1" ,7.73,50.58);
		aStar.addNode("enter_corridor_2" ,24.75,50.31);
		aStar.addNode("hall" ,5.53,51.86);
		aStar.addNode("enter_lab" ,7.88,47.54);
		aStar.addNode("whiteboard" ,11.12,47.60);
		aStar.addNode("corridor_2_a" ,24.56,47.51);
		aStar.addNode("corridor_2_b" ,24.56,47.51);
		aStar.addNode("corridor_2_c" ,24.31,32.10);

		aStar.addEdge("rest_area" ,"vending_machine");
		aStar.addEdge("rest_area" ,"water_font");
		aStar.addEdge("rest_area" ,"coffe_area");
		aStar.addEdge("rest_area" ,"enter_coffe_area");
		aStar.addEdge("vending_machine" ,"water_font");
		aStar.addEdge("vending_machine" ,"coffe_area");
		aStar.addEdge("vending_machine" ,"enter_coffe_area");
		aStar.addEdge("water_font" ,"coffe_area");
		aStar.addEdge("toilette","coffe_area");
		aStar.addEdge("water_font" ,"enter_coffe_area");
		aStar.addEdge("coffe_area" ,"enter_coffe_area");
		aStar.addEdge("enter_coffe_area" ,"enter_corridor_2");
		aStar.addEdge("enter_corridor_2" ,"corridor_2_a");
		aStar.addEdge("corridor_2_a" ,"classroom_1");
		aStar.addEdge("corridor_2_a" ,"corridor_2_b");
		aStar.addEdge("corridor_2_b" ,"classroom_2_1");
		aStar.addEdge("corridor_2_b" ,"classroom_2_2");
		aStar.addEdge("corridor_2_b" ,"corridor_2_c");
		aStar.addEdge("corridor_2_c" ,"classroom_3_1");
		aStar.addEdge("corridor_2_c" ,"classroom_3_2");
		aStar.addEdge("corridor_2_c" ,"exit_2");
		aStar.addEdge("enter_corridor_2" ,"enter_corridor_1");
		aStar.addEdge("enter_corridor_1" ,"enter_lab");
		aStar.addEdge("enter_lab" ,"whiteboard");
		aStar.addEdge("whiteboard" ,"robotics_lab");
		aStar.addEdge("enter_lab" ,"exit_1");
		aStar.addEdge("enter_corridor_1" ,"hall");
		aStar.addEdge("hall" ,"reception");
		*/
		
		

	};
	virtual ~AStarPathProvider() {}
	virtual utils::Vector2d& getNextPoint(const utils::Vector2d& position, const utils::Vector2d& goal, utils::Vector2d& nextPoint)
	{
		/*
		std::string start_id,goal_id;
		aStar.getClosestNode(position.getX(),position.getY(),start_id);
		aStar.getClosestNode(goal.getX(),goal.getY(),goal_id);
		std::list<std::string> path;		
		aStar.getPath(start_id,goal_id,path);
		if (path.size()<2) {
			nextPoint = goal;
			return nextPoint;
		}
		double x,y;
		auto it = path.begin();
		++it;
		aStar.getPos(*it,x,y);
		nextPoint.set(x,y);
		return nextPoint;
		*/


		static const utils::Vector2d B(27,7);
		static const utils::Vector2d C(52,7);
		static const utils::Vector2d D(27,32);
		static const utils::Vector2d E(52,32);
		
		int area =0 ;
		if (position.getX()<26.5 && position.getY()>=30) {
			area=1;
		} else if (position.getX()>27.5 && position.getX()<51.5 && position.getY()>=30) {
			area=3;
		} else if (position.getX()>40) {
			area=4;
		} else {
			area=2;
		}

		if ( (goal - B).norm()<0.01) {
			if (area==1 || area  ==3) {
				nextPoint = D;
			} else if (area==2) {
				nextPoint = B;
			} else {
				nextPoint = E;
			}
		} else if ((goal - C).norm()<0.01) {
			if (area == 1 || area==3) {
				nextPoint = E;
			} else if (area==4) {
				nextPoint = C;
			} else if (area == 2 && position.getY()<32.5) {
				nextPoint = D;
			} else {
				nextPoint = E;
			}
		} else {
			nextPoint = goal;
		}
		return nextPoint;

	}

private:

	


	//utils::AStar aStar;

};



class Simulator : public pomcp::Simulator<State,Observation,Action>
{
public:
	
	utils::Vector2d robot_pos;
	utils::Vector2d robot_vel;
	utils::Vector2d target_pos;
	utils::Vector2d target_vel;
	

	Simulator(double discount, double gridCellSize, const std::vector<utils::Vector2d>& goals, PathProvider& pathProvider,double trackingRange, double goalRadius, double runningTime);

	virtual double getDiscount() const  {return discount;}

	virtual bool simulate(const State& state, unsigned actionIndex, State& nextState, Observation& observation, double& reward) const; 
	virtual bool simulate(const State& state, unsigned actionIndex, State& nextState, double& reward) const;

	virtual State& sampleInitialState(State& state) const;
	virtual unsigned getNumActions() const {return 6;}
	virtual const Action& getAction(unsigned actionIndex) const {return actions[actionIndex];}
	virtual bool allActionsAreValid(const State& state) const {return false;}
	virtual bool isValidAction(const State& state, unsigned actionIndex) const;

private:
	void getObservation(const State& state, Observation& observation) const;
	double getReward(const State& state,double force) const;
	bool simulate(const State& state, unsigned actionIndex, State& nextState, double& reward, double dt) const;

	double discount;
	double gridCellSize;
	std::vector<utils::Vector2d> goals;	
	PathProvider& pathProvider;
	std::vector<Action> actions;
	double trackingRange;
	double goalRadius;
	double runningTime;
	double naiveGoalTime;
	double alphaFactor;
	double betaFactor;
	double gammaFactor; 

};

inline
Simulator::Simulator(double discount, double gridCellSize, const std::vector<utils::Vector2d>& goals, PathProvider& pathProvider, double trackingRange, double goalRadius, double runningTime)
: discount(discount),
  gridCellSize(gridCellSize),
  goals(goals),
  pathProvider(pathProvider),
  trackingRange(trackingRange),
  goalRadius(goalRadius),
  runningTime(runningTime),
  naiveGoalTime(1.0),
  alphaFactor(1.0), 
  betaFactor(-1.0),
  gammaFactor(-100.0)
{
	actions.resize(6);
	actions[0] = LEFT;
	actions[1] = RIGHT;
	actions[2] = BEHIND;
	actions[3] = FOLLOW_PATH;
	actions[4] = WAIT;
	actions[5] = SET_GOAL;
}

inline
State& Simulator::sampleInitialState(State& state) const
{
	state.robot_pos = robot_pos;
	state.robot_vel = robot_vel;
	state.target_pos = target_pos;
	state.target_vel = target_vel;

	if (goals.empty()) {
		state.goal = target_pos;
	} else {
		state.goal = goals[utils::RANDOM(goals.size())];
	}
	return state;
}

/*
inline
bool Simulator::simulate(const State& state, unsigned actionIndex, State& nextState, double& reward) const
{
	std::vector<sfm::Agent> agents;
	agents.resize(2);
	agents[0].position = state.robot_pos;
	agents[0].velocity = state.robot_vel;
	agents[0].yaw = state.robot_vel.angle();	

	const Action& action = getAction(actionIndex);

	sfm::Goal robotGoal;
	robotGoal.radius = goalRadius;
	if (action == LEFT) {
		robotGoal.center = state.target_pos + naiveGoalTime * state.target_vel + state.target_vel.normalized().leftNormalVector();
	} else if (action == RIGHT) {
		robotGoal.center = state.target_pos + naiveGoalTime * state.target_vel + state.target_vel.normalized().rightNormalVector();
	} else if (action == BEHIND) {
		robotGoal.center = state.target_pos + naiveGoalTime * state.target_vel - state.target_vel.normalized();
	} else if (action == WAIT) {
		// No goal
	} else if (action == FOLLOW_PATH) {
		// TODO
	}
	
	if (action == LEFT || action == RIGHT || action == BEHIND) {
		agents[0].goals.push_back(robotGoal);
	}
	agents[0].groupId = 0;

	agents[1].position = state.target_pos;
	agents[1].velocity = state.target_vel;
	agents[1].yaw = state.target_vel.angle();
	
	sfm::Goal targetLocalGoal;
	targetLocalGoal.radius = goalRadius;
	pathProvider.getNextPoint(state.target_pos,state.goal,targetLocalGoal.center);
	agents[1].goals.push_back(targetLocalGoal);
	
	agents[1].groupId = 0;

	sfm::Map *map = &sfm::MAP;
	sfm::SFM.computeForces(agents,map);
	sfm::SFM.updatePosition(agents,runningTime);
	
	
	nextState.robot_pos = agents[0].position; //+utils::Vector2d(utils::RANDOM(0,1),utils::RANDOM(0,1));
	nextState.robot_vel = agents[0].velocity;
	
	nextState.target_pos = agents[1].position; //+utils::Vector2d(utils::RANDOM(0,1),utils::RANDOM(0,1));
	nextState.target_vel = agents[1].velocity;
	
	
	if (utils::RANDOM()<0.1) {
		nextState.goal = goals[utils::RANDOM(goals.size())];
	} else {
		nextState.goal = state.goal;
	}
	
	reward = getReward(nextState,agents[1].forces.groupForce.norm());	
	
	return (nextState.target_pos - nextState.goal).norm() <= goalRadius;
	//return false;
}
*/


inline
bool Simulator::simulate(const State& state, unsigned actionIndex, State& nextState, double& reward) const
{
	State a,b;
	a=state;
	double r;
	reward=0;
	for (unsigned i = 0; i< 5; i++) {
		simulate(a,actionIndex,b,r,runningTime/5);
		reward+=r;
		a=b;
	}
	nextState=a;
	return (nextState.target_pos - nextState.goal).norm() <= goalRadius;
}

inline
bool Simulator::simulate(const State& state, unsigned actionIndex, State& nextState, double& reward, double dt) const
{
	std::vector<sfm::Agent> agents;
	agents.resize(2);
	agents[0].position = state.robot_pos;
	agents[0].velocity = state.robot_vel;
	agents[0].desiredVelocity = 0.6;
	agents[0].yaw = state.robot_vel.angle();	

	const Action& action = getAction(actionIndex);

	sfm::Goal robotGoal;
	robotGoal.radius = goalRadius;
	if (action == SET_GOAL) {
		pathProvider.getNextPoint(state.robot_pos,state.goal,robotGoal.center);
	} else if (action == LEFT) {
		robotGoal.center = state.target_pos + naiveGoalTime * state.target_vel + state.target_vel.normalized().leftNormalVector();
	} else if (action == RIGHT) {
		robotGoal.center = state.target_pos + naiveGoalTime * state.target_vel + state.target_vel.normalized().rightNormalVector();
	} else if (action == BEHIND) {
		robotGoal.center = state.target_pos + naiveGoalTime * state.target_vel - state.target_vel.normalized();
	} else if (action == WAIT) {
		// No goal
	} else if (action == FOLLOW_PATH) {
		// TODO
	}
	
	if (action == LEFT || action == RIGHT || action == BEHIND || action == SET_GOAL) {
		agents[0].goals.push_back(robotGoal);
	}
	agents[0].groupId = 0;

	agents[1].position = state.target_pos;
	agents[1].velocity = state.target_vel;
	agents[1].yaw = state.target_vel.angle();
	agents[1].desiredVelocity = 1.29;
	
	sfm::Goal targetLocalGoal;
	targetLocalGoal.radius = goalRadius;
	pathProvider.getNextPoint(state.target_pos,state.goal,targetLocalGoal.center);
	agents[1].goals.push_back(targetLocalGoal);
	
	agents[1].groupId = 0;

	sfm::Map *map = &sfm::MAP;
	sfm::SFM.computeForces(agents,map);
	sfm::SFM.updatePosition(agents,dt);
	
	
	nextState.robot_pos = agents[0].position; //+utils::Vector2d(utils::RANDOM(0,1),utils::RANDOM(0,1));
	nextState.robot_vel = agents[0].velocity;
	
	nextState.target_pos = agents[1].position; //+utils::Vector2d(utils::RANDOM(0,1),utils::RANDOM(0,1));
	nextState.target_vel = agents[1].velocity;
	
	
	if ((nextState.target_pos - state.goal).norm()<0.1) {
		nextState.goal = goals[utils::RANDOM(goals.size())];
	} else {
		nextState.goal = state.goal;
	}
	
	reward = getReward(nextState,agents[1].forces.groupForce.norm());	
	//reward = getReward(nextState,agents[1].forces.desiredForce.norm());	

	return (nextState.target_pos - nextState.goal).norm() <= goalRadius;
	//return false;
}


inline
double Simulator::getReward(const State& state,double force) const
{
	sfm::Map *map = &sfm::MAP;
	double alpha = map->getNearestObstacle(state.robot_pos).distance;
	double beta = (state.robot_pos - state.target_pos).norm();
	return alphaFactor*alpha + betaFactor*beta + gammaFactor*force;
	
}


inline
void Simulator::getObservation(const State& state, Observation& observation) const
{
	observation.robot_pos_grid_x = (int)std::round((state.robot_pos.getX()+utils::RANDOM(0,1))/gridCellSize);
	observation.robot_pos_grid_y = (int)std::round((state.robot_pos.getY()+utils::RANDOM(0,1))/gridCellSize);
	observation.target_pos_grid_x = (int)std::round((state.target_pos.getX()+utils::RANDOM(0,1))/gridCellSize);
	observation.target_pos_grid_y = (int)std::round((state.target_pos.getY()+utils::RANDOM(0,1))/gridCellSize);

	//observation.robot_pos_grid_x = (int)std::round(state.robot_pos.getX()/gridCellSize);
	//observation.robot_pos_grid_y = (int)std::round(state.robot_pos.getY()/gridCellSize);
	//observation.target_pos_grid_x = (int)std::round(state.target_pos.getX()/gridCellSize);
	//observation.target_pos_grid_y = (int)std::round(state.target_pos.getY()/gridCellSize);


	if (utils::RANDOM()<0.5 || (state.robot_pos - state.target_pos).norm() > trackingRange) {
		observation.target_hidden = true;
	} else {
		observation.target_hidden = false;
	}
}


inline
bool Simulator::simulate(const State& state, unsigned actionIndex, State& nextState, Observation& observation, double& reward) const
{
	bool r=simulate(state,actionIndex,nextState,reward);
	getObservation(nextState,observation); 
	return r;
}


inline
bool Simulator::isValidAction(const State& state, unsigned actionIndex) const
{
	const Action& action = getAction(actionIndex);
	
	if (action == FOLLOW_PATH  ) {
		return false;
	}
	if (action == BEHIND || action == SET_GOAL) {
		return true;
	}
	utils::Vector2d v;
	if (action == LEFT) {
		v = state.target_pos + naiveGoalTime * state.target_vel + state.target_vel.normalized().leftNormalVector();
	} else if (action == RIGHT) {
		v = state.target_pos + naiveGoalTime * state.target_vel + state.target_vel.normalized().rightNormalVector();
	} 
	
	sfm::Map *map = &sfm::MAP;
	return !map->isObstacle(v);
	

	
}


}

}




#endif
