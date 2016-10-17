#ifndef _MODEL_HPP_
#define _MODEL_HPP_

#include <lightpomcp/MonteCarloSimulator.hpp>
#include <lightpomcp/Random.hpp>
#include <boost/functional/hash.hpp>
#include "forces.hpp"

namespace wsbs
{



struct State
{
	Agent robot;
	Agent target;
	utils::Vector2d goal;

	bool operator == (const State& other) const
	{
		return robot == other.robot && target == other.target && goal == other.goal;
	}
};


struct Observation
{
	
};

class Simulator : public pomcp::Simulator<State,Observation,ControllerMode>
{
public:

	std::vector<utils::Vector2d> goals;
	Agent robot;
	Agent target;

	Simulator(double discount);
	virtual ~Simulator() {}

	virtual double getDiscount() const  {return discount;}

	virtual State& sampleInitialState(State& state) const;

	virtual bool simulate(const State& state, unsigned actionIndex, State& nextState, Observation& observation, double& reward) const {return true;}

	virtual bool simulate(const State& state, unsigned actionIndex, State& nextState, double& reward) const;

	virtual unsigned getNumActions() const {return 5;}

	virtual const ControllerMode& getAction(unsigned actionIndex) const {return actions[actionIndex];}

	virtual bool allActionsAreValid(const State& state) const {return false;}

	virtual bool isValidAction(const State& state, unsigned actionIndex) const {return true;}

	virtual void cleanup() {}


private:
	static utils::Vector2d Simulator::getGoal(const State& state, const ControllerMode& action);

	double discount;
	ControllerMode actions[5];
	

};

inline
Simulator::Simulator(double discount)
: discount(discount)
{
	actions[0] = LEFT;
	actions[1] = RIGHT;
	actions[2] = BEHIND;
	actions[3] = FOLLOW_PATH;
	actions[4] = WAIT;
}


inline
State& Simulator::sampleInitialState(State& state) const
{
	state.robot = robot;
	state.target = target;
	if (goals.empty()) {
		state.goal = target.position;
	} else {
		state.goal = goals[utils::RANDOM(goals.size())];
	}
	return state;
}


inline
utils::Vector2d Simulator::getGoal(const State& state, const ControllerMode& action)
{
	if (action == 

	if (action == LEFT) {


	} else if (action == RIGHT) {

	} 



}

inline
bool Simulator::simulate(const State& state, unsigned actionIndex, State& nextState, double& reward) const
{



}






}

namespace std
{

template <>
struct hash<wsbs::State>
{
	size_t operator()( const wsbs::State& state ) const
	{
		using boost::hash_value;
		using boost::hash_combine;

		std::size_t seed = 0;

		hash_combine(seed,hash_value(state.robot.position.getX()));
		hash_combine(seed,hash_value(state.robot.position.getY()));
		hash_combine(seed,hash_value(state.robot.velocity.getX()));
		hash_combine(seed,hash_value(state.robot.velocity.getY()));
		
		hash_combine(seed,hash_value(state.target.position.getX()));
		hash_combine(seed,hash_value(state.target.position.getY()));
		hash_combine(seed,hash_value(state.target.velocity.getX()));
		hash_combine(seed,hash_value(state.target.velocity.getY()));
		
		hash_combine(seed,hash_value(state.goal.getX()));
		hash_combine(seed,hash_value(state.goal.getY()));
		return seed;
	}
};

}




#endif
