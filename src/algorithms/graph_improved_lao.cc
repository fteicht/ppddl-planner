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

#include <cmath>
#include "graph_improved_lao.h"

GraphImprovedLAO::GraphImprovedLAO(const Problem& pb, double epsilon, double discount_factor, unsigned int plan_length, heuristic_t heuristic_type,
									determinization_t determinization_type, deterministic_planner_t deterministic_planner_type)
try : BaseAlgorithm(pb, epsilon, discount_factor),
		GraphHeuristicAlgorithm<GraphStateWT<> >(pb, epsilon, discount_factor, plan_length, heuristic_type, determinization_type, deterministic_planner_type)
{
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphImprovedLAO::GraphImprovedLAO");
	throw;
}


void GraphImprovedLAO::solve_initialize(const PddlState& st)
{
	reachable_states_.clear();
	GraphHeuristicAlgorithm<GraphStateWT<> >::solve_initialize(st);
	reachable_states_.insert(initial_state_);
}


void GraphImprovedLAO::solve_progress()
{
	std::list<double> record_values(reachable_states_.size());
	std::list<double>::iterator Ivl = record_values.begin();

	for (states_set_t::const_iterator Ist = reachable_states_.begin() ; Ist != reachable_states_.end() ; ++Ist)
	{
		*Ivl = (*Ist)->get_value();
		++Ivl;
	}

	optimize(reachable_states_);

	bellman_error_ = 0.0;
	Ivl = record_values.begin();

	for (states_set_t::const_iterator Ist = reachable_states_.begin() ; Ist != reachable_states_.end() ; ++Ist)
	{
		bellman_error_ = std::max(bellman_error_, std::fabs((*Ivl) - ((*Ist)->get_value())));
		++Ivl;
	}

	if (bellman_error_ < epsilon_)
		continue_search_ = false;
	else
	{
		continue_search_ = true;
		compute_reachability();
	}
}


bool GraphImprovedLAO::has_converged()
{
	extern int verbosity; // defined in main.cc

	if (verbosity >= 1)
	{
		std::cerr << "Bellman error: " << bellman_error_ << std::endl;
		std::cerr << dynamic_cast<const GraphMDP<GraphStateWT<> >&>(get_mdp()).get_states().size() << " explored states" << std::endl;
		std::cerr << reachable_states_.size() << " reachable states" << std::endl;
	}

	return (!continue_search_);
}


void GraphImprovedLAO::compute_reachability()
{
	reachable_states_.clear();
	reachable_states_.insert(initial_state_);
	states_set_t forward_frontier;
	forward_frontier.insert(initial_state_);
	states_set_t reachable_tip_states;

	while (!forward_frontier.empty())
	{
		states_set_t new_forward_frontier;

		for (states_set_t::iterator Ist = forward_frontier.begin() ; Ist != forward_frontier.end() ; ++Ist)
		{
			std::list<GraphTransition<GraphStateWT<> >*>& effects = (*Ist)->_transitions_().find(&((*Ist)->get_policy()))->second;

			for (std::list<GraphTransition<GraphStateWT<> >*>::iterator Ieff = effects.begin() ; Ieff != effects.end() ; ++Ieff)
			{
				GraphStateWT<>& nst = (*Ieff)->_outcome_();
				std::pair<states_set_t::iterator, bool> res = reachable_states_.insert(&nst);

				if (res.second)
				{
					if (nst.expanded())
						new_forward_frontier.insert(&nst);
					else
						reachable_tip_states.insert(&nst);
				}
			}
		}

		forward_frontier = new_forward_frontier;
	}

	// Phase 2 of improved LAO* (Table 7 of AIJ paper)

	for (states_set_t::iterator Ist = reachable_tip_states.begin() ; Ist != reachable_tip_states.end() ; ++Ist)
		initialize(**Ist);
}
