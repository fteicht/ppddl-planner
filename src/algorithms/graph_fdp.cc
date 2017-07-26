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

#include <algorithm>
#include <cmath>

#include "graph_fdp.h"

GraphFDP::GraphFDP(const Problem& pb, double epsilon, double discount_factor, unsigned int plan_length, heuristic_t heuristic_type,
					determinization_t determinization_type, deterministic_planner_t deterministic_planner_type)
try : BaseAlgorithm(pb, epsilon, discount_factor),
		GraphHeuristicAlgorithm<GraphStateWTA<> >(pb, epsilon, discount_factor, plan_length, heuristic_type, determinization_type, deterministic_planner_type)
{
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphFDP::GraphFDP");
	throw;
}


void GraphFDP::solve_initialize(const PddlState& st)
{
	explored_states_.clear();
	initial_state_ = &(mdp_->add_state(st));
	explored_states_.insert(initial_state_);
	compute_initial_policy();
}


void GraphFDP::solve_progress()
{
	compute_reachability();

	std::list<double> record_values(forward_reachable_states_.size());
	std::list<double>::iterator Ivl = record_values.begin();

	for (states_set_t::const_iterator Ist = forward_reachable_states_.begin() ; Ist != forward_reachable_states_.end() ; ++Ist)
	{
		*Ivl = (*Ist)->get_value();
		++Ivl;
	}

	optimize(forward_reachable_states_);

	bellman_error_ = 0.0;
	Ivl = record_values.begin();

	for (states_set_t::const_iterator Ist = forward_reachable_states_.begin() ; Ist != forward_reachable_states_.end() ; ++Ist)
	{
		bellman_error_ = std::max(bellman_error_, std::fabs((*Ivl) - ((*Ist)->get_value())));
		++Ivl;
	}

	if (bellman_error_ < epsilon_)
		continue_search_ = false;
	else
		continue_search_ = true;
}


bool GraphFDP::has_converged()
{
	return (!continue_search_);
}


void GraphFDP::compute_initial_policy()
{
	compute_initial_policy_forward_reachability();
	compute_initial_policy_backward_reachability();
	compute_initial_policy_tip_states();
	optimize(explored_states_);
	states_set_t temp_states;
	std::set_union(explored_states_.begin(), explored_states_.end(),
			tip_states_.begin(), tip_states_.end(),
			std::inserter(temp_states, temp_states.begin()));
	explored_states_ = temp_states;
}


void GraphFDP::compute_initial_policy_forward_reachability()
{
	reachable_goal_states_.clear();
	forward_reachable_states_.clear();
	forward_reachable_states_.insert(initial_state_);
	states_set_t forward_frontier;
	forward_frontier.insert(initial_state_);

	while ((!forward_frontier.empty()) && reachable_goal_states_.empty())
	{
		states_set_t new_frontier;

		for (states_set_t::iterator Ist = forward_frontier.begin() ; Ist != forward_frontier.end() ; ++Ist)
		{
			const GraphStateWTA<>::actions_map_t* transitions = mdp_->transitions(**Ist);

			for (GraphStateWTA<>::actions_map_t::const_iterator Itr = transitions->begin() ; Itr != transitions->end() ; ++Itr)
			{
				for (std::list<GraphTransition<GraphStateWTA<> >*>::const_iterator Ieff = Itr->second.begin() ; Ieff != Itr->second.end() ; ++Ieff)
				{
					GraphStateWTA<>& nst = (*Ieff)->_outcome_();

					if (forward_reachable_states_.insert(&nst).second)
					{
						if (mdp_->goal(nst))
							reachable_goal_states_.insert(&nst);
						else
							new_frontier.insert(&nst);
					}
				}
			}
		}

		forward_frontier = new_frontier;
	}
}


void GraphFDP::compute_initial_policy_backward_reachability()
{
	backward_reachable_states_.clear();
	backward_reachable_states_ = reachable_goal_states_;
	states_set_t backward_frontier;
	backward_frontier = reachable_goal_states_;
	bool initial_state_reached = false;

	while ((!backward_frontier.empty()) && (!initial_state_reached))
	{
		states_set_t new_frontier;

		for (states_set_t::iterator Ist = backward_frontier.begin() ; Ist != backward_frontier.end() ; ++Ist)
		{
			for (GraphStateWTA<>::ancestors_map_t::const_iterator Ian = (*Ist)->get_ancestors().begin() ; Ian != (*Ist)->get_ancestors().end() ; ++Ian)
			{
				GraphStateWTA<>& pst = *(Ian->first);

				if (backward_reachable_states_.insert(&pst).second)
				{
					if (initial_state_ != (&pst))
						new_frontier.insert(&pst);
					else
						initial_state_reached = true;
				}
			}
		}

		backward_frontier = new_frontier;
	}

	explored_states_ = backward_reachable_states_;
}


void GraphFDP::compute_initial_policy_tip_states()
{
	tip_states_.clear();

	for (states_set_t::iterator Ist = explored_states_.begin() ; Ist != explored_states_.end() ; ++Ist)
	{
		const GraphStateWTA<>::actions_map_t* transitions = mdp_->transitions(**Ist);

		for (GraphStateWTA<>::actions_map_t::const_iterator Itr = transitions->begin() ; Itr != transitions->end() ; ++Itr)
		{
			for (std::list<GraphTransition<GraphStateWTA<> >*>::const_iterator Ieff = Itr->second.begin() ; Ieff != Itr->second.end() ; ++Ieff)
			{
				GraphStateWTA<>& nst = (*Ieff)->_outcome_();

				if (explored_states_.find(&nst) == explored_states_.end())
				{
					tip_states_.insert(&nst);
					heuristic_->compute(nst);
				}
			}
		}
	}
}


void GraphFDP::compute_reachability()
{
	forward_reachable_states_.clear();
	forward_reachable_states_.insert(initial_state_);
	states_set_t forward_frontier;
	forward_frontier.insert(initial_state_);
	states_set_t reachable_tip_states;

	while (!forward_frontier.empty())
	{
		states_set_t new_forward_frontier;

		for (states_set_t::iterator Ist = forward_frontier.begin() ; Ist != forward_frontier.end() ; ++Ist)
		{
			std::list<GraphTransition<GraphStateWTA<> >*>& effects = (*Ist)->_transitions_().find(&((*Ist)->get_policy()))->second;

			for (std::list<GraphTransition<GraphStateWTA<> >*>::iterator Ieff = effects.begin() ; Ieff != effects.end() ; ++Ieff)
			{
				GraphStateWTA<>& nst = (*Ieff)->_outcome_();

				if (forward_reachable_states_.insert(&nst).second)
				{
					if (tip_states_.find(&nst) == tip_states_.end())
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
