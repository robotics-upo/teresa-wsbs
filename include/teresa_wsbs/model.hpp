#ifndef _MODEL_HPP_
#define _MODEL_HPP_

#include <tf/transform_listener.h>
#include <lightpomcp/MonteCarloSimulator.hpp>
#include <lightpomcp/Random.hpp>
#include <boost/functional/hash.hpp>
#include <lightsfm/sfm.hpp>
#include <lightsfm/map.hpp>

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
	WAIT		= 4
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


class Simulator : public pomcp::Simulator<State,Observation,Action>
{
public:
	
	utils::Vector2d robot_pos;
	utils::Vector2d robot_vel;
	utils::Vector2d target_pos;
	utils::Vector2d target_vel;
	

	Simulator(double discount, double gridCellSize, const std::vector<utils::Vector2d>& goals, PathProvider& pathProvider);

	virtual double getDiscount() const  {return discount;}

	virtual bool simulate(const State& state, unsigned actionIndex, State& nextState, Observation& observation, double& reward) const; 
	virtual bool simulate(const State& state, unsigned actionIndex, State& nextState, double& reward) const;

	virtual State& sampleInitialState(State& state) const;
	virtual unsigned getNumActions() const {return 5;}
	virtual const Action& getAction(unsigned actionIndex) const {return actions[actionIndex];}
	virtual bool allActionsAreValid(const State& state) const {return true;}
	virtual bool isValidAction(const State& state, unsigned actionIndex) const {return true;}

private:
	void getObservation(const State& state, Observation& observation) const;

	double discount;
	double gridCellSize;
	std::vector<utils::Vector2d> goals;	
	PathProvider& pathProvider;
	std::vector<Action> actions;
};

inline
Simulator::Simulator(double discount, double gridCellSize, const std::vector<utils::Vector2d>& goals, PathProvider& pathProvider)
: discount(discount),
  gridCellSize(gridCellSize),
  goals(goals),
  pathProvider(pathProvider)
{
	actions.resize(5);
	actions[0] = LEFT;
	actions[1] = RIGHT;
	actions[2] = BEHIND;
	actions[3] = FOLLOW_PATH;
	actions[4] = WAIT;
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


inline
bool Simulator::simulate(const State& state, unsigned actionIndex, State& nextState, double& reward) const
{
	// TODO

	return true;
}


inline
void Simulator::getObservation(const State& state, Observation& observation) const
{
	observation.robot_pos_grid_x = (int)std::round(state.robot_pos.getX()/gridCellSize);
	observation.robot_pos_grid_y = (int)std::round(state.robot_pos.getY()/gridCellSize);
	observation.target_pos_grid_x = (int)std::round(state.target_pos.getX()/gridCellSize);
	observation.target_pos_grid_y = (int)std::round(state.target_pos.getY()/gridCellSize);
	// TODO: is target hidden?
}


inline
bool Simulator::simulate(const State& state, unsigned actionIndex, State& nextState, Observation& observation, double& reward) const
{
	bool r=simulate(state,actionIndex,nextState,reward);
	getObservation(nextState,observation); 
	return r;
}



}

}




#endif
