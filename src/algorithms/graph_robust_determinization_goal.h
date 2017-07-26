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

#ifndef GRAPH_ROBUST_DETERMINIZATION_GOAL_H_
#define GRAPH_ROBUST_DETERMINIZATION_GOAL_H_

#include "determinization_algorithm.h"
#include "particle_algorithm.h"

class GraphRobustDeterminizationGoal : public GraphAlgorithm<GraphStateWT<> >, public DeterminizationAlgorithm, public ParticleAlgorithm<GraphStateWT<>*>
{
public :
	typedef std::map<GraphStateWT<>*, double> states_map_t;
	typedef std::set<GraphStateWT<>*> states_set_t;
	typedef ParticleAlgorithm<GraphStateWT<>*>::particles_list_t particles_list_t;

protected :
	GraphStateWT<>* initial_state_;
	states_map_t reachable_tip_states_;
	double exit_probability_;

	void compute_reachability();
	void expand_tip_states();
	void generate_trajectory(GraphStateWT<>& st);

	virtual void make_problem(GraphStateWT<>& st) =0;
	virtual void expand_state_trajectory(GraphStateWT<>& st) =0;

public :
	/* Constructs an algorithm */
	GraphRobustDeterminizationGoal(const Problem& pb, double epsilon, double discount_factor,
			unsigned int plan_length, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
			double probabilistic_threshold = 0.1, unsigned int nb_of_particles = 10, unsigned int trajectories_max_depth = 100); // throws

	/* Destructs this algorithm */
	virtual ~GraphRobustDeterminizationGoal() {}

	/* Returns TRUE if the policy (even not complete) is defined in the given state */
	virtual bool policy_defined(const PddlState& st);

	/* Initialize the optimization process with a given intial state */
	virtual void solve_initialize(const PddlState& st);

	/* Progress in the optimization: perform one step of the main algorithm's loop */
	virtual void solve_progress();

	/* Return TRUE if the algorithm has converged */
	virtual bool has_converged();
};


class GraphRobustDeterminizationProblemGoals : public GraphRobustDeterminizationGoal
{
private :
	virtual void make_problem(GraphStateWT<>& st);
	virtual void expand_state_trajectory(GraphStateWT<>& st);

public :
	/* Constructs an algorithm */
	GraphRobustDeterminizationProblemGoals(const Problem& pb, double epsilon, double discount_factor,
			unsigned int plan_length, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
			double probabilistic_threshold = 0.1, unsigned int nb_of_particles = 10, unsigned int trajectories_max_depth = 100); // throws

	/* Destructs this algorithm */
	virtual ~GraphRobustDeterminizationProblemGoals() {}
};


class GraphRobustDeterminizationRandomGoals : public GraphRobustDeterminizationGoal
{
private :
	unsigned int goals_limit_;
	states_set_t expanded_states_;

	virtual void make_problem(GraphStateWT<>& st);
	virtual void expand_state_trajectory(GraphStateWT<>& st);

public :
	/* Constructs an algorithm */
	GraphRobustDeterminizationRandomGoals(const Problem& pb, double epsilon, double discount_factor,
			unsigned int plan_length, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
			double probabilistic_threshold = 0.1, unsigned int nb_of_particles = 10, unsigned int trajectories_max_depth = 100, unsigned int goals_limit = 100); // throws

	/* Destructs this algorithm */
	virtual ~GraphRobustDeterminizationRandomGoals() {}
};


class GraphRobustDeterminizationBestGoals : public GraphRobustDeterminizationGoal
{
private :
	unsigned int goals_limit_;
	states_set_t expanded_states_;

	virtual void make_problem(GraphStateWT<>& st);
	virtual void expand_state_trajectory(GraphStateWT<>& st);

public :
	/* Constructs an algorithm */
	GraphRobustDeterminizationBestGoals(const Problem& pb, double epsilon, double discount_factor,
			unsigned int plan_length, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
			double probabilistic_threshold = 0.1, unsigned int nb_of_particles = 10, unsigned int trajectories_max_depth = 100, unsigned int goals_limit = 100); // throws

	/* Destructs this algorithm */
	virtual ~GraphRobustDeterminizationBestGoals() {}
};

#endif /*GRAPH_ROBUST_DETERMINIZATION_GOAL_H_*/
