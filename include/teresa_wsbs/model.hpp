#ifndef _MODEL_HPP_
#define _MODEL_HPP_


#include <teresa_wsbs/common.hpp>
#include <lightpomcp/MonteCarloSimulator.hpp>
#include <lightpomcp/Random.hpp>
#include <lightpomcp/Belief.hpp>
#include <lightsfm/rosmap.hpp>




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



class Simulator : public pomcp::Simulator<State,Observation,ControllerMode>
{
public:

	pomcp::VectorBelief<State> lastBelief;
	utils::Vector2d robot_pos;
	utils::Vector2d robot_vel;
	utils::Vector2d target_pos;
	utils::Vector2d target_vel;
	

	Simulator(GoalProvider& goalProvider, double discount, double robotGridCellSize, double targetGridCellSize, double trackingRange, double runningTime);
	~Simulator() {}
	virtual bool simulate(const State& state, unsigned actionIndex, State& nextState, Observation& observation, double& reward) const; 
	virtual bool simulate(const State& state, unsigned actionIndex, State& nextState, double& reward) const;
	virtual State& sampleInitialState(State& state) const;
	virtual double getDiscount() const  {return discount;}
	virtual unsigned getNumActions() const {return 7;}
	virtual const ControllerMode& getAction(unsigned actionIndex) const {return actions[actionIndex];}
	virtual bool allActionsAreValid(const State& state) const {return false;}
	virtual bool isValidAction(const State& state, unsigned actionIndex) const;

private:
	void getObservation(const State& state, Observation& observation) const;
	bool simulate(const State& state, unsigned actionIndex, State& nextState, double& reward,double dt) const;	
	double getReward(const State& state,double force) const;

	GoalProvider& goalProvider;
	double discount;
	double robotGridCellSize;
	double targetGridCellSize;
	double trackingRange;
	double runningTime;
	std::vector<ControllerMode> actions;
	double goalRadius;
	double alphaFactor;
	double betaFactor;
	double gammaFactor; 
	

};

inline
Simulator::Simulator(GoalProvider& goalProvider, double discount, double robotGridCellSize, double targetGridCellSize, double trackingRange, double runningTime)
: goalProvider(goalProvider),
  discount(discount),
  robotGridCellSize(robotGridCellSize),
  targetGridCellSize(targetGridCellSize),
  trackingRange(trackingRange),
  runningTime(runningTime),
  goalRadius(0.25),
  alphaFactor(1.0), 
  betaFactor(-1.0),
  gammaFactor(-100.0)
{
	actions.resize(7);
	actions[0] = HEURISTIC;
	actions[1] = LEFT;
	actions[2] = RIGHT;
	actions[3] = BEHIND;
	actions[4] = FOLLOW_PATH;
	actions[5] = WAIT;
	actions[6] = SET_GOAL;
}

inline
State& Simulator::sampleInitialState(State& state) const
{
	state.robot_pos = robot_pos; // Tip: noise in pos/vel
	state.robot_vel = robot_vel;
	state.target_pos = target_pos;
	state.target_vel = target_vel;

	if (lastBelief.empty()) {
		state.goal = goalProvider.getGoals()[utils::RANDOM(goalProvider.getGoals().size())];
	} else {
		state.goal = lastBelief.sample().goal;
	}
	return state;
}

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
	return false;	
	//return (nextState.target_pos - nextState.goal).norm() <= goalRadius;
}

inline
bool Simulator::simulate(const State& state, unsigned actionIndex, State& nextState, double& reward, double dt) const
{
	std::vector<sfm::Agent> agents;
	agents.resize(2);
	agents[0].position = state.robot_pos;
	agents[0].velocity = state.robot_vel;
	agents[0].desiredVelocity = state.target_vel.norm();	
	//agents[0].desiredVelocity = 0.6;
	agents[0].yaw = state.robot_vel.angle();	
	const ControllerMode& action = getAction(actionIndex);
	sfm::Goal robotGoal;
	robotGoal.radius = goalRadius;
	robotGoal.center = goalProvider.getRobotLocalGoal(action);
	agents[0].goals.push_back(robotGoal);
	
	agents[0].groupId = 0;

	agents[1].position = state.target_pos;
	agents[1].velocity = state.target_vel;
	agents[1].yaw = state.target_vel.angle();
	agents[1].desiredVelocity = 0.9;
 	
	sfm::Goal targetLocalGoal;
	targetLocalGoal.radius = goalRadius;
	goalProvider.getPathProvider().getNextPoint(state.target_pos,state.goal,targetLocalGoal.center);
	agents[1].goals.push_back(targetLocalGoal);
	
	agents[1].groupId = 0;

	sfm::Map *map = &sfm::MAP;
	sfm::SFM.computeForces(agents,map);
	sfm::SFM.updatePosition(agents,dt);
	
	
	nextState.robot_pos = agents[0].position+utils::Vector2d(utils::RANDOM(0,0.2),utils::RANDOM(0,0.2));
	nextState.robot_vel = agents[0].velocity;
	
	nextState.target_pos = agents[1].position+utils::Vector2d(utils::RANDOM(0,0.2),utils::RANDOM(0,0.2));
	nextState.target_vel = agents[1].velocity;
	
	double p = (nextState.target_pos - state.goal).norm()<1.0 ? 0.01 : 0.1; 

	// TODO: Cambiar goal con una probabilidad
	if (utils::RANDOM()<p /* 0.1 0.01*/) {
		nextState.goal = goalProvider.getGoals()[utils::RANDOM(goalProvider.getGoals().size())];
	} else {
		nextState.goal = state.goal;
	}	
	//if ((nextState.target_pos - state.goal).norm()<0.1 /* 0.1 0.01*/) {
	//	nextState.goal = goalProvider.getGoals()[utils::RANDOM(goalProvider.getGoals().size())];
	//} else {
	//	nextState.goal = state.goal;
	//}
	
	reward = getReward(nextState,agents[1].forces.groupForce.norm());	
	
	return false;
	//return (nextState.target_pos - nextState.goal).norm() <= goalRadius;
	



}

inline
void Simulator::getObservation(const State& state, Observation& observation) const
{
	// TODO: Poner dos parametros
	//observation.robot_pos_grid_x = (int)std::round((state.robot_pos.getX()+utils::RANDOM(0,1))/gridCellSize);
	//observation.robot_pos_grid_y = (int)std::round((state.robot_pos.getY()+utils::RANDOM(0,1))/gridCellSize);
	//observation.target_pos_grid_x = (int)std::round((state.target_pos.getX()+utils::RANDOM(0,0.25))/gridCellSize);
	//observation.target_pos_grid_y = (int)std::round((state.target_pos.getY()+utils::RANDOM(0,0.25))/gridCellSize);

	observation.robot_pos_grid_x = (int)std::round(state.robot_pos.getX()/robotGridCellSize);
	observation.robot_pos_grid_y = (int)std::round(state.robot_pos.getY()/robotGridCellSize);
	observation.target_pos_grid_x = (int)std::round(state.target_pos.getX()/targetGridCellSize);
	observation.target_pos_grid_y = (int)std::round(state.target_pos.getY()/targetGridCellSize);

	
	if (utils::RANDOM()<0.1 || (state.robot_pos - state.target_pos).norm() > trackingRange) {
		observation.target_hidden = true;
	} else {
		observation.target_hidden = false;
	}
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
bool Simulator::simulate(const State& state, unsigned actionIndex, State& nextState, Observation& observation, double& reward) const
{
	bool r=simulate(state,actionIndex,nextState,reward);
	getObservation(nextState,observation); 
	return r;
}



inline
bool Simulator::isValidAction(const State& state, unsigned actionIndex) const
{
	const ControllerMode& action = getAction(actionIndex);
	return action==HEURISTIC;	
	//return action==HEURISTIC || action == SET_GOAL;
	//return action!=FOLLOW_PATH && action!=WAIT;	
	/*
	if (action == FOLLOW_PATH || action==WAIT ) {
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
	*/
}

  



}


}
#endif
