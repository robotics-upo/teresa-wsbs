#ifndef _MODEL_HPP_
#define _MODEL_HPP_


#include <teresa_wsbs/common.hpp>
#include <lightpomcp/MonteCarloSimulator.hpp>
#include <lightpomcp/Random.hpp>
#include <lightsfm/rosmap.hpp>




namespace wsbs
{

enum ModelType
{
	ONLY_HEURISTIC,
	ONLY_GO_TO_GOAL,
	HEURISTIC_AND_GO_TO_GOAL,
	LEFT_RIGHT_BEHIND,
	LEFT_RIGHT_BEHIND_WAIT,
	LEFT_RIGHT_BEHIND_WAIT_GO_TO_GOAL
};

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

	utils::Vector2d robot_pos;
	utils::Vector2d robot_vel;
	utils::Vector2d target_pos;
	utils::Vector2d target_vel;

	std::vector<sfm::Agent> otherAgents;
	std::vector<std::vector<sfm::Agent>> futureAgents;
	

	Simulator(GoalProvider& goalProvider, double discount, double robotGridCellSize, double targetGridCellSize, double trackingRange, double runningTime, ModelType modelType);
	~Simulator() {}
	virtual bool simulate(const State& state, unsigned actionIndex, State& nextState, Observation& observation, double& reward,unsigned depth) const; 
	virtual bool simulate(const State& state, unsigned actionIndex, State& nextState, double& reward,unsigned depth) const;
	virtual State& sampleInitialState(State& state) const;
	virtual double getDiscount() const  {return discount;}
	virtual unsigned getNumActions() const {return 8;}
	virtual const ControllerMode& getAction(unsigned actionIndex) const {return actions[actionIndex];}
	virtual bool allActionsAreValid(const State& state) const {return false;}
	virtual bool isValidAction(const State& state, unsigned actionIndex) const;
	
	void addOtherAgent(double x, double y, double vel, double yaw);

private:
	void getObservation(const State& state, Observation& observation) const;
	bool simulate(const State& state, unsigned actionIndex, State& nextState, double& reward,double dt,unsigned depth) const;	
	double getReward(const State& state,double force) const;
	static bool intimateDistance(const utils::Vector2d& x, const utils::Vector2d& p, const utils::Angle& yaw, double factor);
	static double getConfort(const State& state);

	GoalProvider& goalProvider;
	double discount;
	double robotGridCellSize;
	double targetGridCellSize;
	double trackingRange;
	double runningTime;
	ModelType modelType;	
	std::vector<ControllerMode> actions;
	double goalRadius;
	double alphaFactor;
	double betaFactor;
	double gammaFactor; 
	
	unsigned root_index;
};

inline
Simulator::Simulator(GoalProvider& goalProvider, double discount, double robotGridCellSize, double targetGridCellSize, double trackingRange, double runningTime, ModelType modelType)
: goalProvider(goalProvider),
  discount(discount),
  robotGridCellSize(robotGridCellSize),
  targetGridCellSize(targetGridCellSize),
  trackingRange(trackingRange),
  runningTime(runningTime),
  modelType(modelType),
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
	actions[7] = SET_FINAL_GOAL;
}

inline
State& Simulator::sampleInitialState(State& state) const
{
	state.robot_pos = robot_pos; // Tip: noise in pos/vel
	state.robot_vel = robot_vel;
	state.target_pos = target_pos;
	state.target_vel = target_vel;
	state.goal = goalProvider.getGoals()[utils::RANDOM(goalProvider.getGoals().size())];
	
	return state;
}

inline
bool Simulator::simulate(const State& state, unsigned actionIndex, State& nextState, double& reward,unsigned depth) const
{
	State a,b;
	a=state;
	double r;
	reward=0;
	for (unsigned i = 0; i< 5; i++) {
		simulate(a,actionIndex,b,r,runningTime/5,depth);
		reward+=r;
		a=b;
	}
	nextState=a;
	return false;	
	//return (nextState.target_pos - nextState.goal).norm() <= goalRadius;
}

inline
bool Simulator::simulate(const State& state, unsigned actionIndex, State& nextState, double& reward, double dt,unsigned depth) const
{
	GoalProvider tmpGoalProvider(0.5,100,2.0,2.0,1.0,"map",goalProvider.getPathProvider(),false);
	
	std::vector<sfm::Agent> agents;
	agents.resize(2 + otherAgents.size());
	agents[0].position = state.robot_pos;
	agents[0].velocity = state.robot_vel;
	if ( (state.robot_pos - state.target_pos).norm() >= 2.0) {
		agents[0].desiredVelocity = 0.6;	
	} else {
		agents[0].desiredVelocity = state.target_vel.norm();
	}	
	//agents[0].desiredVelocity = 0.6;
	agents[0].yaw = state.robot_vel.angle();	
	const ControllerMode& action = getAction(actionIndex);
	sfm::Goal robotGoal;
	robotGoal.radius = goalRadius;
	tmpGoalProvider.update(state.robot_pos,state.target_pos,state.target_vel,state.goal);
	robotGoal.center = tmpGoalProvider.getRobotLocalGoal(action);
	
	agents[0].goals.push_back(robotGoal);
	
	agents[0].groupId = 0;

	agents[1].position = state.target_pos;
	agents[1].velocity = state.target_vel;
	agents[1].yaw = state.target_vel.angle();
	agents[1].params.forceFactorDesired = 4.0;
	agents[1].desiredVelocity = 0.9;
 	
	sfm::Goal targetLocalGoal;
	targetLocalGoal.radius = goalRadius;
	goalProvider.getPathProvider().getNextPoint(state.target_pos,state.goal,targetLocalGoal.center);
	
	agents[1].goals.push_back(targetLocalGoal);
	
	agents[1].groupId = 0;
	
	for(unsigned int i=0; i< otherAgents.size();i++)
	{
		if (depth==0) {
			agents[2+i] = otherAgents[i];
		} else if (depth<11) {
			agents[2+i] = futureAgents[depth-1][i];
		} else {
			agents[2+i] = futureAgents[9][i];
		}
	}
	
	/*
	for (unsigned i=2; i<agents.size();i++) {
		sfm::Goal g;
		g.center = agents[i].position + 2.0*agents[i].velocity;
		g.radius = 0.25;
		agents[i].goals.clear();
		agents[i].goals.push_back(g);
	}
	*/
	sfm::Map *map = &sfm::MAP;
	sfm::SFM.computeForces(agents,map);
	sfm::SFM.updatePosition(agents,dt);
	double targetSocialWork = agents[0].forces.groupForce.dot(agents[1].movement);
	double peopleSocialWork = 0;	
	for (unsigned i=2;i<agents.size();i++) {
		peopleSocialWork += agents[i].forces.robotSocialForce.dot(agents[i].movement);
	}	

	//std::cout<<targetSocialWork<<" "<<peopleSocialWork<<std::endl;
	
	nextState.robot_pos = agents[0].position+utils::Vector2d(utils::RANDOM(0,0.2),utils::RANDOM(0,0.2));
	nextState.robot_vel = agents[0].velocity;
	
	nextState.target_pos = agents[1].position+utils::Vector2d(utils::RANDOM(0,0.2),utils::RANDOM(0,0.2));
	nextState.target_vel = agents[1].velocity;
	
	
	double p = (nextState.target_pos - state.goal).norm()<0.5 ? 0.1 : 0.01; 
	
	
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
	

	reward = 0.5 * targetSocialWork + 0.5 * peopleSocialWork;
	//reward = getReward(nextState,agents[1].forces.groupForce.norm());	
	//reward = getConfort(nextState);

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

// return true if x is in inside the intimate distance(*factor) of person p with yaw
inline
bool Simulator::intimateDistance(const utils::Vector2d& x, const utils::Vector2d& p, const utils::Angle& yaw, double factor)
{
	const double lambda = 0.4;
	utils::Vector2d n = (p - x).normalized();
	utils::Vector2d e(yaw.cos(),yaw.sin());
	double cos_phi = -n.dot(e);
	double w = lambda + (1-lambda)*(1+cos_phi)*0.5;
	w *= factor;
	double dis = (p-x).norm();
	return dis < w;	
}



inline
double Simulator::getConfort(const State& state) 
{
	utils::Vector2d x;
	const double robot_area_radius = 0.5;
	const double squared_robot_area_radius = 0.25;
	const int confort_particles = 1;
	double score=0;
	for (int i = 0; i< confort_particles; i++) {
		// Select a random particle in the robot area
		do {
			x.set(utils::RANDOM()*robot_area_radius, utils::RANDOM()*robot_area_radius);
		} while (x[0]*x[0] + x[1]*x[1] > squared_robot_area_radius);
		unsigned quadrant = utils::RANDOM(4);
		if (quadrant==2 || quadrant==3) {
			x.setX(-x[0]);
		}
		if (quadrant==1 || quadrant==2) {
			x.setY(-x[1]); 
		}
		x+=state.robot_pos;
		if (intimateDistance(x, state.target_pos, state.target_vel.angle(),1.0)) {
			continue;
		}
				
		if (intimateDistance(x, state.target_pos, state.target_vel.angle(),2.0)) {
			score += 1.0;	
		} else if ((x - state.target_pos).norm() < 2.0) {
			score += 0.5;
		} 
	}
	score /= (double)confort_particles;
	return score;

}





inline
double Simulator::getReward(const State& state,double force) const
{
	sfm::Map *map = &sfm::MAP;
	double alpha = map->getNearestObstacle(state.robot_pos).distance;
	double beta = (state.robot_pos - state.target_pos).norm();
	return alphaFactor*alpha + betaFactor*beta; /* + gammaFactor*force;*/
	
}

inline
bool Simulator::simulate(const State& state, unsigned actionIndex, State& nextState, Observation& observation, double& reward,unsigned depth) const
{
	bool r=simulate(state,actionIndex,nextState,reward,depth);
	getObservation(nextState,observation); 
	return r;
}

inline
bool Simulator::isValidAction(const State& state, unsigned actionIndex) const
{
	const ControllerMode& action = getAction(actionIndex);
	switch(modelType) {
		case ONLY_HEURISTIC:
			return action==HEURISTIC;
		case ONLY_GO_TO_GOAL:
			return action==SET_FINAL_GOAL;
		case HEURISTIC_AND_GO_TO_GOAL:
			return action==HEURISTIC || action == SET_GOAL;
		case LEFT_RIGHT_BEHIND:
			return action==LEFT || action ==RIGHT || action == BEHIND;
		case LEFT_RIGHT_BEHIND_WAIT:
			return action==LEFT || action ==RIGHT || action == BEHIND || action==WAIT;
		default:
			return action==LEFT || action ==RIGHT || action == BEHIND || action==WAIT || action == SET_GOAL;

	}


	/*if (modelType == ONLY_HEURISTIC) {
		return action==HEURISTIC;
	}
	if (modelType == ONLY_GO_TO_GOAL) {
		return action==SET_FINAL_GOAL;
	}
	if (modelType == HEURISTIC_AND_GO_TO_GOAL) {
		return action==HEURISTIC || action == SET_GOAL;
	}
	if (action == LEFT || action == RIGHT || action == BEHIND) {
		utils::Vector2d v;
		if (action == LEFT) {
			v = state.target_pos + 2.0 * state.target_vel + state.target_vel.normalized().leftNormalVector();
		} else if (action == RIGHT) {
			v = state.target_pos + 2.0 * state.target_vel + state.target_vel.normalized().rightNormalVector();
		} else {
			v = state.target_pos + 2.0 * state.target_vel - state.target_vel.normalized();
		} 
	
		sfm::Map *map = &sfm::MAP;
		return map->getNearestObstacle(v).distance > 0.2;
	}
	if (modelType == LEFT_RIGHT_BEHIND) {
		return false;
	}
	if (modelType == LEFT_RIGHT_BEHIND_WAIT) {
		return action==WAIT;
	}

	if (modelType == LEFT_RIGHT_BEHIND_WAIT_GO_TO_GOAL) {
		return action==WAIT || action==SET_GOAL;
	}

	return true;
	*/
}

  
void Simulator::addOtherAgent(double x, double y, double vel, double yaw)
{
	sfm::Agent a;

	a.position = utils::Vector2d(x,y);
	a.velocity = utils::Vector2d(vel*cos(yaw),vel*sin(yaw));
	a.yaw = utils::Angle::fromRadian(yaw);
	a.desiredVelocity = vel;
	a.groupId = -1;

	otherAgents.push_back(a);

}


}


}
#endif
