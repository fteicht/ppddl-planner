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
#include "graph_trfsp_star.h"

GraphTrFSPstar::GraphTrFSPstar(const Problem& pb, double epsilon, double discount_factor, unsigned int plan_length, heuristic_t heuristic_type,
								determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
								double probabilistic_threshold)
try : BaseAlgorithm(pb, epsilon, discount_factor),
		GraphHeuristicAlgorithm<GraphStateWTA<> >(pb, epsilon, discount_factor, plan_length, heuristic_type, determinization_type, deterministic_planner_type)
{
	set_probabilistic_threshold(probabilistic_threshold);
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphTrFSPstar::GraphTrFSPstar");
	throw;
}


void GraphTrFSPstar::set_probabilistic_threshold(double probabilistic_threshold)
{
	if ((probabilistic_threshold < 0.0) || (probabilistic_threshold > 1.0))
		throw BaseException("probabilistic threshold must be in range [0;1]", "GraphTrFSPstar::set_probabilistic_threshold");

	probabilistic_threshold_ = probabilistic_threshold;
}


void GraphTrFSPstar::solve_initialize(const PddlState& st)
{
	explored_states_.clear();
	reachable_states_.clear();
	GraphHeuristicAlgorithm<GraphStateWTA<> >::solve_initialize(st);
	reachable_states_.insert(initial_state_);
	explored_states_.insert(initial_state_);
}


void GraphTrFSPstar::solve_progress()
{
	optimize(reachable_states_);
	compute_reachability();
}


bool GraphTrFSPstar::has_converged()
{
	return (!continue_search_);
}


void GraphTrFSPstar::compute_reachability()
{
	states_set_t new_reachable_states;
	new_reachable_states.insert(initial_state_);
	states_set_t forward_frontier;
	forward_frontier.insert(initial_state_);
	states_set_t reachable_tip_states;
	states_set_t states_frontier;
	states_set_t previous_reachable_states;
	previous_reachable_states.insert(initial_state_);

	while (!forward_frontier.empty())
	{
		states_set_t new_forward_frontier;

		for (states_set_t::iterator Ist = forward_frontier.begin() ; Ist != forward_frontier.end() ; ++Ist)
		{
			std::list<GraphTransition<GraphStateWTA<> >*>& effects = (*Ist)->_transitions_().find(&((*Ist)->get_policy()))->second;

			for (std::list<GraphTransition<GraphStateWTA<> >*>::iterator Ieff = effects.begin() ; Ieff != effects.end() ; ++Ieff)
			{
				GraphStateWTA<>& nst = (*Ieff)->_outcome_();

				if (new_reachable_states.insert(&nst).second)
				{
					if (reachable_states_.find(&nst) != reachable_states_.end())
					{
						new_forward_frontier.insert(&nst);
						previous_reachable_states.insert(&nst);
					}
					else
					{
						states_frontier.insert(&nst);

						if (nst.expanded())
							new_forward_frontier.insert(&nst);
						else
						{
							reachable_tip_states.insert(&nst);
							explored_states_.insert(&nst);
						}
					}
				}
			}
		}

		forward_frontier = new_forward_frontier;
	}

	if (compute_exit_probability(previous_reachable_states, states_frontier) < probabilistic_threshold_)
		continue_search_ = false;
	else
	{
		continue_search_ = true;
		reachable_states_ = new_reachable_states;

		for (states_set_t::iterator Ist = reachable_tip_states.begin() ; Ist != reachable_tip_states.end() ; ++Ist)
			initialize(**Ist);
	}
}


//void GraphTrFSPstar::compute_reachability()
//{
//	states_uprobabilities_t u_probabilities;
//	states_rprobabilities_t r_probabilities;
//	initialize_probabilities(u_probabilities, r_probabilities);
//	std::list<double> record_probabilities(r_probabilities.size());
//	double relaxed_epsilon = std::exp(std::log(epsilon_) / 2.0);
//
//	while (true)
//	{
//		// record probabilities
//		std::list<double>::iterator Ipb = record_probabilities.begin();
//
//		for (states_rprobabilities_t::const_iterator Irpb = r_probabilities.begin() ; Irpb != r_probabilities.end() ; ++Irpb)
//		{
//			*Ipb = Irpb->second;
//			++Ipb;
//		}
//
//		update_rprobabilities(u_probabilities, r_probabilities);
//		double norm = 0.0;
//		Ipb = record_probabilities.begin();
//
//		for (states_rprobabilities_t::const_iterator Irpb = r_probabilities.begin() ; Irpb != r_probabilities.end() ; ++Irpb)
//		{
//			norm = std::max(norm, std::fabs((Irpb->second) - (*Ipb)));
//			++Ipb;
//		}
//
//		if (norm < relaxed_epsilon)
//			break;
//		else
//			update_uprobabilities(u_probabilities, r_probabilities);
//	}
//
//	states_set_t new_reachable_states;
//	states_set_t reachable_tip_states;
//	continue_search_ = false;
//
//	for (states_rprobabilities_t::const_iterator Irp = r_probabilities.begin() ; Irp != r_probabilities.end() ; ++Irp)
//	{
//		if ((Irp->second) > probabilistic_threshold_)
//		{
//			new_reachable_states.insert(Irp->first);
//
//			if (reachable_states_.find(Irp->first) == reachable_states_.end())
//				continue_search_ = true;
//
//			if (tip_states_.find(Irp->first) != tip_states_.end())
//				reachable_tip_states.insert(Irp->first);
//		}
//	}
//
//	reachable_states_ = new_reachable_states;
//
//	for (states_set_t::iterator Ist = reachable_tip_states.begin() ; Ist != reachable_tip_states.end() ; ++Ist)
//		initialize(**Ist);
//}


void GraphTrFSPstar::initialize_probabilities(states_uprobabilities_t& u_probabilities, states_rprobabilities_t& r_probabilities)
{
	for (states_set_t::const_iterator Ist = explored_states_.begin() ; Ist != explored_states_.end() ; ++Ist)
	{
		r_probabilities.insert(std::make_pair(*Ist, 0.0));
		states_rprobabilities_t& temp_prob = u_probabilities.insert(std::make_pair(*Ist, states_rprobabilities_t())).first->second;

		for (states_set_t::const_iterator Inst = explored_states_.begin() ; Inst != explored_states_.end() ; ++Inst)
		{
			for (GraphStateWTA<>::ancestors_map_t::const_iterator Ipst = (*Inst)->get_ancestors().begin() ; Ipst != (*Inst)->get_ancestors().end() ; ++Ipst)
			{
				if ((Ipst->first) == initial_state_)
				{
					std::map<const Action*, const GraphTransition<GraphStateWTA<> >*>::const_iterator Iaction = Ipst->second.find(&(initial_state_->get_policy()));

					if (Iaction != (Ipst->second.end()))
						temp_prob.insert(std::make_pair(*Inst, Iaction->second->get_probability()));
					else
						temp_prob.insert(std::make_pair(*Inst, 0.0));
				}
				else
					temp_prob.insert(std::make_pair(*Inst, 0.0));
			}
		}
	}

	r_probabilities[initial_state_] = 1.0;
}


void GraphTrFSPstar::update_rprobabilities(const states_uprobabilities_t& u_probabilities, states_rprobabilities_t& r_probabilities)
{
	states_uprobabilities_t::const_iterator Iup = u_probabilities.begin();

	for (states_rprobabilities_t::iterator Irp = r_probabilities.begin() ; Irp != r_probabilities.end() ; ++Irp)
	{
		Irp->second = 1.0 - ((1.0 - (Iup->second.find(Irp->first)->second)) * (1.0 - (Irp->second)));
		++Iup;
	}
}


void GraphTrFSPstar::update_uprobabilities(states_uprobabilities_t& u_probabilities, const states_rprobabilities_t& r_probabilities)
{
	states_uprobabilities_t record_u_probabilities = u_probabilities;
	states_uprobabilities_t::const_iterator Irup = record_u_probabilities.begin();

	for (states_uprobabilities_t::iterator Iup = u_probabilities.begin() ; Iup != u_probabilities.end() ; ++Iup)
	{
		double denum = 1.0 - (Irup->second.find(Irup->first)->second);

		if (denum > 0.0)
		{
			for (states_rprobabilities_t::iterator Irp = Iup->second.begin() ; Irp != Iup->second.end() ; ++Irp)
			{
				Irp->second = 0.0;

				for (GraphStateWTA<>::ancestors_map_t::const_iterator Ipst = Irp->first->get_ancestors().begin() ; Ipst != Irp->first->get_ancestors().end() ; ++Ipst)
				{
					if ((Ipst->first) != (Iup->first))
					{
						std::map<const Action*, const GraphTransition<GraphStateWTA<> >*>::const_iterator Iaction = Ipst->second.find(&(Ipst->first->get_policy()));

						if (Iaction != (Ipst->second.end()))
							Irp->second += (Iaction->second->get_probability()) * (Irup->second.find(Ipst->first)->second);
					}
				}

				Irp->second /= denum;
			}
		}
		else
		{
			for (states_rprobabilities_t::iterator Irp = Iup->second.begin() ; Irp != Iup->second.end() ; ++Irp)
				Irp->second = 0.0;
		}

		++Irup;
	}
}


double GraphTrFSPstar::compute_exit_probability(const states_set_t& previous_reachable_states, const states_set_t& states_frontier) const
{
	std::map<GraphStateWTA<>*, double> probabilities, record_probabilities;
	double relaxed_epsilon = std::exp(std::log(epsilon_) / 2.0);

	for (states_set_t::const_iterator Irs = previous_reachable_states.begin() ; Irs != previous_reachable_states.end() ; ++Irs)
	{
		probabilities.insert(std::make_pair(*Irs, 0.0));
		record_probabilities.insert(std::make_pair(*Irs, 0.0));
	}

	while (true)
	{
		for (std::map<GraphStateWTA<>*, double>::iterator Ip = probabilities.begin() ; Ip != probabilities.end() ; ++Ip)
		{
			const std::list<GraphTransition<GraphStateWTA<> >*>& effects = Ip->first->get_transitions().find(&(Ip->first->get_policy()))->second;
			Ip->second = 0.0;

			for (std::list<GraphTransition<GraphStateWTA<> >*>::const_iterator Ieff = effects.begin() ; Ieff != effects.end() ; ++Ieff)
			{
				GraphStateWTA<>& nst = (*Ieff)->_outcome_();

				if (states_frontier.find(&nst) != states_frontier.end())
					Ip->second += (*Ieff)->get_probability();
				else
					Ip->second += ((*Ieff)->get_probability()) * (record_probabilities.find(&nst)->second);
			}
		}

		std::map<GraphStateWTA<>*, double>::iterator Irp = record_probabilities.begin();
		double norm = 0.0;

		for (std::map<GraphStateWTA<>*, double>::const_iterator Ip = probabilities.begin() ; Ip != probabilities.end() ; ++Ip)
		{
			norm = std::max(norm, std::fabs((Ip->second) - (Irp->second)));
			Irp->second = Ip->second;
			++Irp;
		}

		if (norm < relaxed_epsilon)
			break;
	}

	return (probabilities.find(initial_state_)->second);
}
