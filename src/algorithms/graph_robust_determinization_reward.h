/*
   ppddl-planner - client for IPPC'08

   Copyright (C) 2008 Florent Teichteil-Koenigsbuch and Guillaume Infantes and Ugur Kuter

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

*/

#ifndef GRAPH_ROBUST_DETERMINIZATION_REWARD_H_
#define GRAPH_ROBUST_DETERMINIZATION_REWARD_H_

#include "determinization_algorithm.h"
#include "particle_algorithm.h"
#include "planners/deterministic_planner.h"

class BaseDeterminizationWithFluents;

class GraphRobustDeterminizationReward : public GraphAlgorithm<GraphStateWT<> >, public DeterminizationAlgorithm, public ParticleAlgorithm<GraphStateWT<>*>
{
public :
	typedef std::map<GraphStateWT<>*, double> states_map_t;
	typedef std::set<GraphStateWT<>*> states_set_t;
	typedef ParticleAlgorithm<GraphStateWT<>*>::particles_list_t particles_list_t;

protected :
    BaseDeterminizationWithFluents* cast_determinization_;

    double probability_weight_;
    double reward_weight_;

    unsigned int timeout_;
    unsigned int max_trials_;

	GraphStateWT<>* initial_state_;
	states_map_t reachable_tip_states_;
	double exit_probability_;

	virtual void compute_reachability();
	void expand_tip_states();
	void generate_trajectory(GraphStateWT<>& st);

	void expand_state_trajectory(GraphStateWT<>& st);
	double goal_objective(double log_probability, double reward) const;

	virtual void find_plan(GraphStateWT<>& st, DeterministicPlanner::plan_t& plan) =0;

public :
	/* Constructs an algorithm */
	GraphRobustDeterminizationReward(const Problem& pb, double epsilon, double discount_factor,
			unsigned int plan_length, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
			double probabilistic_threshold = 0.1, unsigned int nb_of_particles = 10, unsigned int trajectories_max_depth = 100,
			double probability_weight = 1.0, double reward_weight = 10.0, unsigned int timeout = 5, unsigned int max_trials = 10); // throws

	/* Destructs this algorithm */
	virtual ~GraphRobustDeterminizationReward() {}

	/* Returns TRUE if the policy (even not complete) is defined in the given state */
	virtual bool policy_defined(const PddlState& st);

	/* Initialize the optimization process with a given intial state */
	virtual void solve_initialize(const PddlState& st);

	/* Progress in the optimization: perform one step of the main algorithm's loop */
	virtual void solve_progress();

	/* Return TRUE if the algorithm has converged */
	virtual bool has_converged();
};

class GraphRobustDeterminizationOptimizedReward : public virtual GraphRobustDeterminizationReward
{
protected :
    states_set_t reachable_states_;

    virtual void compute_reachability();

public :
    /* Constructs an algorithm */
	GraphRobustDeterminizationOptimizedReward(const Problem& pb, double epsilon, double discount_factor,
			unsigned int plan_length, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
			double probabilistic_threshold = 0.1, unsigned int nb_of_particles = 10, unsigned int trajectories_max_depth = 100,
			double probability_weight = 1.0, double reward_weight = 10.0, unsigned int timeout = 5, unsigned int max_trials = 10); // throws

    /* Destructs this algorithm */
	virtual ~GraphRobustDeterminizationOptimizedReward() {}

	/* Progress in the optimization: perform one step of the main algorithm's loop */
	virtual void solve_progress();

};

class GraphRobustDeterminizationRewardObjectiveDichotomy : public virtual GraphRobustDeterminizationReward
{
private :
    virtual void find_plan(GraphStateWT<>& st, DeterministicPlanner::plan_t& plan);
    void make_problem(GraphStateWT<>& st, double obj);

public :
    /* Constructs an algorithm */
	GraphRobustDeterminizationRewardObjectiveDichotomy(const Problem& pb, double epsilon, double discount_factor,
			unsigned int plan_length, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
			double probabilistic_threshold = 0.1, unsigned int nb_of_particles = 10, unsigned int trajectories_max_depth = 100,
			double probability_weight = 1.0, double reward_weight = 10.0, unsigned int timeout = 5, unsigned int max_trials = 10); // throws

    virtual ~GraphRobustDeterminizationRewardObjectiveDichotomy() {}
};

class GraphRobustDeterminizationOptimizedRewardObjectiveDichotomy : public GraphRobustDeterminizationOptimizedReward, public GraphRobustDeterminizationRewardObjectiveDichotomy
{
public :
    /* Constructs an algorithm */
	GraphRobustDeterminizationOptimizedRewardObjectiveDichotomy(const Problem& pb, double epsilon, double discount_factor,
			unsigned int plan_length, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
			double probabilistic_threshold = 0.1, unsigned int nb_of_particles = 10, unsigned int trajectories_max_depth = 100,
			double probability_weight = 1.0, double reward_weight = 10.0, unsigned int timeout = 5, unsigned int max_trials = 10); // throws

    virtual ~GraphRobustDeterminizationOptimizedRewardObjectiveDichotomy() {}
};

class GraphRobustDeterminizationRewardObjectiveOptimization : public virtual GraphRobustDeterminizationReward
{
private :
    virtual void find_plan(GraphStateWT<>& st, DeterministicPlanner::plan_t& plan);
    void make_problem(GraphStateWT<>& st);

public :
    /* Constructs an algorithm */
	GraphRobustDeterminizationRewardObjectiveOptimization(const Problem& pb, double epsilon, double discount_factor,
			unsigned int plan_length, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
			double probabilistic_threshold = 0.1, unsigned int nb_of_particles = 10, unsigned int trajectories_max_depth = 100,
			double probability_weight = 1.0, double reward_weight = 10.0, unsigned int timeout = 5, unsigned int max_trials = 10); // throws

    virtual ~GraphRobustDeterminizationRewardObjectiveOptimization() {}
};

class GraphRobustDeterminizationOptimizedRewardObjectiveOptimization : public GraphRobustDeterminizationOptimizedReward, public GraphRobustDeterminizationRewardObjectiveOptimization
{
public :
    /* Constructs an algorithm */
	GraphRobustDeterminizationOptimizedRewardObjectiveOptimization(const Problem& pb, double epsilon, double discount_factor,
			unsigned int plan_length, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
			double probabilistic_threshold = 0.1, unsigned int nb_of_particles = 10, unsigned int trajectories_max_depth = 100,
			double probability_weight = 1.0, double reward_weight = 10.0, unsigned int timeout = 5, unsigned int max_trials = 10); // throws

    virtual ~GraphRobustDeterminizationOptimizedRewardObjectiveOptimization() {}
};

#endif /*GRAPH_ROBUST_DETERMINIZATION_REWARD_H_*/
