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

#ifndef BASE_ALGORITHM_H_
#define BASE_ALGORITHM_H_

#include <ostream>
#include <list>
#include <set>

#include "models/transitions_blackbox.h"
#include "models/graph_mdp.h"
#include "models/symbolic_mdp.h"
#include "planners/base_planner.h"

class BaseMDP;

class BaseAlgorithm
{
protected :
	TransitionsBlackbox* tb_;
	double epsilon_;
	double discount_factor_;
	double bellman_error_;

public :
	/* Constructs an algorithm */
	BaseAlgorithm(const Problem& pb, double epsilon, double discount_factor); // throws

	/* Destructs this algorithm */
	virtual ~BaseAlgorithm() {delete tb_;}

	void set_epsilon(double epsilon); // throws
	void set_discount_factor(double discount_factor); // throws

	inline double get_epsilon() const {return epsilon_;}
	inline double get_discount_factor() const {return discount_factor_;}
	inline double get_bellman_error() const {return bellman_error_;}

	inline const TransitionsBlackbox& get_transitions_blackbox() const {return (*tb_);}

	/* Returns TRUE if the policy (even not complete) is defined in the given state */
	virtual bool policy_defined(const PddlState& st) =0;

	/* Initialize the optimization process with a given intial state */
	virtual void solve_initialize(const PddlState& st) =0;

	/* Progress in the optimization: perform one step of the main algorithm's loop */
	virtual void solve_progress() =0;

	/* Return TRUE if the algorithm has converged */
	virtual bool has_converged() =0;

	/* Returns the underlying MDP */
	virtual const BaseMDP& get_mdp() const =0;

    /**
     * Computes the goal-probability and goal-cost values using a given algorithm for/from a given state
     * @param alg Algorithm used to compute the goal statistics
     * @param st State for/from which the goal statistics has to be computed
     * @return Pair containing the goal-probability (first) and the goal-cost (second) for/from the given state
     */
    virtual std::pair<double, double> compute_goal_statistics(algorithm_t alg, const PddlState& st) =0;

    /* Returns the size of the reachable state space */
    virtual unsigned int reachable_state_space_size() const =0;
};

/* Prints the model and the solution of the given algorithm on the given stream */
std::ostream& operator<<(std::ostream& os, const BaseAlgorithm& a);


template <class Tstate>
class GraphAlgorithm : public virtual BaseAlgorithm
{
public :
	typedef std::set<Tstate*> states_set_t;
	typedef std::list<Tstate*> states_list_t;
	typedef typename GraphMDP<Tstate>::actions_map_t actions_map_t;
	typedef std::list<GraphTransition<Tstate>*> transitions_list_t;

protected :
	GraphMDP<Tstate>* mdp_;

	template <class states_container_t> void optimize(states_container_t& states_subset);
	void improve_state_value(Tstate& st);
	void improve_state_value(Tstate& st, const actions_map_t& transitions);
	double compute_q_value(const transitions_list_t& effects);

public :
	GraphAlgorithm(const Problem& pb, double epsilon, double discount_factor); // throws
	virtual ~GraphAlgorithm();

	/* Returns the underlying MDP */
	virtual const BaseMDP& get_mdp() const {return (*mdp_);}
    const GraphMDP<Tstate>& mdp() const {return (*mdp_);}
    GraphMDP<Tstate>& _mdp_() {return (*mdp_);}

    /**
     * Computes the goal-probability and goal-cost values using a given algorithm for/from a given state
     * @param alg Algorithm used to compute the goal statistics
     * @param st State for/from which the goal statistics has to be computed
     * @return Pair containing the goal-probability (first) and the goal-cost (second) for/from the given state
     */
    virtual std::pair<double, double> compute_goal_statistics(algorithm_t alg, const PddlState& st);

    /* Returns the size of the reachable state space */
    virtual unsigned int reachable_state_space_size() const;
};


class SymbolicAlgorithm : public virtual BaseAlgorithm
{
protected :
	SymbolicMDP* mdp_;
	dd_node_ptr discount_factor_dd_;

	void optimize(const dd_node_ptr& states_subset);
	void compute_fringe(const dd_node_ptr& states_subset, dd_node_ptr& states_fringe) const;
	void compute_local_transitions(const dd_node_ptr& states_subset, SymbolicMDP::transitions_map_t& local_transitions) const;
	dd_node_ptr compute_action_value(const dd_node_ptr& primed_value_function, const SymbolicTransition& transitions) const;
	void compute_policy(dd_node_ptr& policy, dd_node_ptr& value_function, const dd_node_ptr& primed_value_function, const SymbolicMDP::transitions_map_t& transitions) const;
	void compute_bellman_error(const dd_node_ptr& previous_value_function, const dd_node_ptr& current_value_function);

public :
	SymbolicAlgorithm(const Problem& pb, double epsilon, double discount_factor); // throws
	virtual ~SymbolicAlgorithm();

	/* Returns the underlying MDP */
	virtual const BaseMDP& get_mdp() const {return (*mdp_);}
    const SymbolicMDP& mdp() const {return (*mdp_);}
    SymbolicMDP& _mdp_() {return (*mdp_);}

    /**
     * Computes the goal-probability and goal-cost values using a given algorithm for/from a given state
     * @param alg Algorithm used to compute the goal statistics
     * @param st State for/from which the goal statistics has to be computed
     * @return Pair containing the goal-probability (first) and the goal-cost (second) for/from the given state
     */
    virtual std::pair<double, double> compute_goal_statistics(algorithm_t alg, const PddlState& st);

    /* Returns the size of the reachable state space */
    virtual unsigned int reachable_state_space_size() const;
};


class GoalStatisticsAlgorithm : public virtual BaseAlgorithm
{
public :
    GoalStatisticsAlgorithm(const Problem& pb, double epsilon) : BaseAlgorithm(pb, epsilon, 1.0) {}
    virtual ~GoalStatisticsAlgorithm() {}

    virtual double goal_probability(const PddlState& st) const =0;
    virtual double goal_cost(const PddlState& st) const =0;
};

#endif /*BASE_ALGORITHM_H_*/
