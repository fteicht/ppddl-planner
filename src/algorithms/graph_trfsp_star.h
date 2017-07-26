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

#ifndef GRAPH_TRFSP_STAR_H_
#define GRAPH_TRFSP_STAR_H_

#include <set>
#include "heuristic_algorithm.h"

class GraphTrFSPstar : public GraphHeuristicAlgorithm<GraphStateWTA<> >
{
public :
	typedef std::set<GraphStateWTA<> *> states_set_t;
	typedef std::map<GraphStateWTA<> *, double> states_rprobabilities_t;
	typedef std::map<GraphStateWTA<> *, states_rprobabilities_t> states_uprobabilities_t;

private :
	double probabilistic_threshold_;
	states_set_t reachable_states_;
	states_set_t explored_states_;
	bool continue_search_;

	void compute_reachability();
	void initialize_probabilities(states_uprobabilities_t& u_probabilities, states_rprobabilities_t& r_probabilities);
	void update_rprobabilities(const states_uprobabilities_t& u_probabilities, states_rprobabilities_t& r_probabilities);
	void update_uprobabilities(states_uprobabilities_t& u_probabilities, const states_rprobabilities_t& r_probabilities);
	double compute_exit_probability(const states_set_t& previous_reachable_states, const states_set_t& states_frontier) const;

public :
	/* Constructs an algorithm */
	GraphTrFSPstar(const Problem& pb, double epsilon, double discount_factor, unsigned int plan_length, heuristic_t heuristic_type,
					determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
					double probabilistic_threshold = 0.1); // throws

	/* Destructs this algorithm */
	virtual ~GraphTrFSPstar() {}

	/* Initialize the optimization process with a given intial state */
	virtual void solve_initialize(const PddlState& st);

	/* Progress in the optimization: perform one step of the main algorithm's loop */
	virtual void solve_progress();

	/* Return TRUE if the algorithm has converged */
	virtual bool has_converged();

	/* Sets the probabilistic threshold */
	void set_probabilistic_threshold(double probabilistic_threshold); // throws

	/* Gets the probabilistic threshold */
	inline double get_probabilistic_threshold() const {return probabilistic_threshold_;}
};

#endif /*GRAPH_TRFSP_STAR_H_*/
