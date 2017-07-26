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

#ifndef GRAPH_MVRT_H_
#define GRAPH_MVRT_H_

#include "base_algorithm.h"
#include "models/graph_mdp.h"

/**
 * Mean-Variance optimization with relaxed-thresholds
 */
class GraphMVRT : public GraphAlgorithm<GraphStateWT<> >
{
public :
	typedef std::map<const GraphStateWT<>*, double> states_map_t;
	
private :
	states_map_t variance_values_;
	std::list<double> record_values_;
	const GraphStateWT<>* initial_state_;
	unsigned int nb_states_;
	bool converged_;
	std::list<std::set<const Action*> > policy_set_;
	static const double probability_enlarge_state_ = 0.1;
	static const double probability_enlarge_action_ = 0.1;

	void minimize_variance();
	double compute_variance_q_value(const transitions_list_t& effects);
	bool enlarge_policy_set();
	
public :
	/* Constructs a MVRT algorithm */
	GraphMVRT(const Problem& pb, double epsilon, double discount_factor); // throws
	
	/* Destructs this algorithm */
	virtual ~GraphMVRT() {}
	
	/* Returns TRUE if the policy (even not complete) is defined in the given state */
	virtual bool policy_defined(const PddlState& st);
	
	/* Initialize the optimization process with a given intial state */
	virtual void solve_initialize(const PddlState& st);
	
	/* Progress in the optimization: perform one step of the main algorithm's loop */
	virtual void solve_progress();
	
	/* Return TRUE if the algorithm has converged */
	virtual bool has_converged();
};

#endif // GRAPH_MVRT_H_
