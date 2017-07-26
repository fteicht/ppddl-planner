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

#ifndef GRAPH_BFDT_H_
#define GRAPH_BFDT_H_

#include <set>
#include "heuristic_algorithm.h"

class GraphBFDT : public GraphHeuristicAlgorithm<GraphStateWT<> >
{
public :
	typedef std::set<GraphStateWT<> *> states_set_t;
	typedef std::list<GraphStateWT<> *> states_list_t;

private :
	unsigned int lookahead_;
	states_set_t reachable_states_;
	states_set_t reachable_tip_states_;
	states_list_t optimized_states_;
	bool continue_search_;

	void compute_reachability();
	void compute_lookahead();

public :
	/* Constructs an algorithm */
	GraphBFDT(const Problem& pb, double epsilon, double discount_factor, unsigned int plan_length, heuristic_t heuristic_type,
				determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
				unsigned int lookahead); // throws

	/* Destructs this algorithm */
	virtual ~GraphBFDT() {}

	/* Initialize the optimization process with a given intial state */
	virtual void solve_initialize(const PddlState& st);

	/* Progress in the optimization: perform one step of the main algorithm's loop */
	virtual void solve_progress();

	/* Return TRUE if the algorithm has converged */
	virtual bool has_converged();
};

#endif /*GRAPH_BFDT_H_*/
