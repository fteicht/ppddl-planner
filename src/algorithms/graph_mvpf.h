/* 
   ppddl-planner - client for IPPC'08

   Copyright (C) 2013 Florent Teichteil-Koenigsbuch and Andrey Kolobov and Caroline Carvalho and Blai Bonet

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

#ifndef GRAPH_MVPF_H_
#define GRAPH_MVPF_H_

#include "base_algorithm.h"
#include "models/graph_mdp.h"

/**
 * Mean-variance approximate Pareto frontier generation
 */
template <typename Tstate>
class GraphMVPF : public GraphAlgorithm<Tstate>
{
protected :
	const Tstate* initial_state_;
	unsigned int nb_states_;
	std::list<double> record_values_;
	
	/**
	 * Compute an initial policy whose mean value is the worst of the best one
     */
	virtual void initial_mean_level_policy() =0;
	
	/**
	 * Generate a policy on the next mean level
     */
	virtual void next_mean_level_policy() =0;
	
	/**
	 * Test whether the current mean level is the last one
     */
	virtual bool last_mean_level() const =0;
	
	/**
	 * Minimize the variance at the current mean level using value iteration
	 */
	void minimize_variance_vi();
	
	/**
	 * Minimize the variance at the current mean level using policy iteration (and starting at the worst one to cover the full range at the current mean level)
	 */
	void minimize_variance_pi();
	
	/**
	 * Evalute the variance of the given policy
	 */
	void evaluate_variance();
	
	/**
	 * Compute the variance q_value of a state-action pair given in the form of its effects, for the current mean level
	 * @param effets state-action pair's effects
	 * @return variance q_value
	 */
	double compute_variance_q_value(const typename GraphAlgorithm<Tstate>::transitions_list_t& effects);
	
	/**
	 * Print the current point of the Pareto frontier on the standard output
     */
	void print_pareto_point() const;

public :
	/* Constructs a MVPF algorithm */
	GraphMVPF(const Problem& pb, double epsilon, double discount_factor); // throws
	
	/* Destructs this algorithm */
	virtual ~GraphMVPF() {}
	
	/* Returns TRUE if the policy (even not complete) is defined in the given state */
	virtual bool policy_defined(const PddlState& st);
	
	/* Initialize the optimization process with a given intial state */
	virtual void solve_initialize(const PddlState& st);
	
	/* Progress in the optimization: perform one step of the main algorithm's loop */
	virtual void solve_progress();
	
	/* Return TRUE if the algorithm has converged */
	virtual bool has_converged();
};

#endif // GRAPH_MVPF_H_
