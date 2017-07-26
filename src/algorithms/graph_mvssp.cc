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

#include <cmath>
#include <limits>
#include "graph_mvssp.h"

GraphMVSSP::GraphMVSSP(double proportion_updated_states)
	: proportion_updated_states_(proportion_updated_states)
{
}


GraphMVPFSSP::GraphMVPFSSP(const Problem& pb, double epsilon, double proportion_updated_states)
try	: BaseAlgorithm(pb, epsilon, 1.0),
		GraphMVSSP(proportion_updated_states),
		GraphMVPF<GraphStateWTA<GraphStateVariance> >(pb, epsilon, 1.0)
{
	tb_->set_metric(TransitionsBlackbox::METRIC_GOAL);
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphMVPFSSP::GraphMVPFSSP");
	throw;
}


void GraphMVPFSSP::initial_mean_level_policy()
{
	record_actions_.resize(nb_states_, NULL);
	
	// Compute a proper policy by recursively connecting states to the goal, starting at the goal
	std::set<GraphStateWTA<GraphStateVariance>*> explored_states = mdp_->get_goal_states();
	std::set<GraphStateWTA<GraphStateVariance>*> search_frontier = explored_states;
	std::map<GraphStateWTA<GraphStateVariance>*, std::list<GraphStateWTA<GraphStateVariance>::actions_map_t::const_iterator> > candidate_policies;
	
	for (std::set<GraphStateWTA<GraphStateVariance>*>::const_iterator Igs = mdp_->get_goal_states().begin() ; Igs != mdp_->get_goal_states().end() ; ++Igs)
		candidate_policies.insert(std::make_pair(*Igs, std::list<GraphStateWTA<GraphStateVariance>::actions_map_t::const_iterator>(1, (*Igs)->get_transitions().begin())));
	
	while (!search_frontier.empty())
	{
		std::set<GraphStateWTA<GraphStateVariance>*> new_search_frontier;
		
		for (std::set<GraphStateWTA<GraphStateVariance>*>::iterator Isf = search_frontier.begin() ; Isf != search_frontier.end() ; ++Isf)
		{
			for (GraphStateWTA<GraphStateVariance>::ancestors_map_t::const_iterator Ias = (*Isf)->get_ancestors().begin() ; Ias != (*Isf)->get_ancestors().end() ; ++Ias)
			{
				if (explored_states.find(Ias->first) == explored_states.end()) // new states
				{
					new_search_frontier.insert(Ias->first);
					explored_states.insert(Ias->first);
					std::map<GraphStateWTA<GraphStateVariance>*, std::list<GraphStateWTA<GraphStateVariance>::actions_map_t::const_iterator> >::iterator Icp = 
						candidate_policies.insert(std::make_pair(Ias->first, std::list<GraphStateWTA<GraphStateVariance>::actions_map_t::const_iterator>())).first;
					
					for (std::map<const Action*, const GraphTransition<GraphStateWTA<GraphStateVariance> >*>::const_iterator It = Ias->second.begin() ; It != Ias->second.end() ; ++It)
						Icp->second.push_back(Ias->first->get_transitions().find(It->first));
				}
			}
		}
		
		search_frontier = new_search_frontier;
	}
	
	// Now that we have a set of proper policies, find the worst one as first point of the Pareto frontier
	do
	{
		// record values
		std::list<double>::iterator Ivl = record_values_.begin();
		
		for (std::map<GraphStateWTA<GraphStateVariance>*, std::list<GraphStateWTA<GraphStateVariance>::actions_map_t::const_iterator> >::const_iterator Ist = candidate_policies.begin() ; Ist != candidate_policies.end() ; ++Ist)
		{
			*Ivl = Ist->first->get_value();
			++Ivl;
		}
		
		// degrade state values
		Ivl = record_values_.begin();
		bellman_error_ = 0.0;
		
		for (std::map<GraphStateWTA<GraphStateVariance>*, std::list<GraphStateWTA<GraphStateVariance>::actions_map_t::const_iterator> >::iterator Ist = candidate_policies.begin() ; Ist != candidate_policies.end() ; ++Ist)
		{
			double worst_q_value = std::numeric_limits<double>::max();
			const Action* worst_action = NULL;
			
			for (std::list<GraphStateWTA<GraphStateVariance>::actions_map_t::const_iterator>::const_iterator Iac = Ist->second.begin() ; Iac != Ist->second.end() ; ++Iac)
			{
				double q_value = compute_q_value((*Iac)->second);
				
				if (q_value < worst_q_value)
				{
					worst_q_value = q_value;
					worst_action = (*Iac)->first;
				}
			}
			
			Ist->first->set_value(worst_q_value);
			Ist->first->set_policy(*worst_action);
			
			bellman_error_ = std::max(bellman_error_, std::fabs((Ist->first->get_value()) - (*Ivl)));
			++Ivl;
		}
	}
	while (bellman_error_ >= epsilon_);
	
	// We have now a policy with worst cost-to-go to start from
	policy_changed_ = true;
	
	// The improvement of the policy begins from the initial state
	state_update_begin_ = mdp_->get_states().begin();
}


void GraphMVPFSSP::next_mean_level_policy()
{
	// Record the policy if we improve the policy from the initial state (new optimization cycle)
	if (state_update_begin_ == mdp_->get_states().begin())
	{
		std::list<const Action*>::iterator Iac = record_actions_.begin();
		
		for (GraphMDP<GraphStateWTA<GraphStateVariance> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
		{
			*Iac = &(Ist->second->get_policy());
			++Iac;
		}
	}
	
	// Improve the policy
	unsigned int nbs = 0;
	GraphMDP<GraphStateWTA<GraphStateVariance> >::states_map_t::const_iterator Is;
	std::map<GraphStateWTA<GraphStateVariance>*, double> record_val;
	
	for (Is = state_update_begin_ ; Is != mdp_->get_states().end() ; ++Is)
	{
		if (nbs > static_cast<unsigned int>(std::floor(nb_states_ * proportion_updated_states_)))
		{
			state_update_begin_ = Is;
			break;
		}
		
		//improve_state_value(*(Is->second));
		if (Is->second->get_transitions().empty())
			return;
		
		double best_q_value = compute_q_value(Is->second->get_transitions().begin()->second);
		const Action* best_action = Is->second->get_transitions().begin()->first;
		
		for (actions_map_t::const_iterator Iac = ++(Is->second->get_transitions().begin()) ; Iac != Is->second->get_transitions().end() ; ++Iac)
		{
			double q_value = compute_q_value(Iac->second);
			
			if (q_value > best_q_value)
			{
				best_q_value = q_value;
				best_action = Iac->first;
			}
		}
		
		record_val.insert(std::make_pair(Is->second, best_q_value));
		Is->second->set_policy(*best_action);
		nbs++;
	}
	
	for (std::map<GraphStateWTA<GraphStateVariance>*, double>::const_iterator Irvl = record_val.begin() ; Irvl != record_val.end() ; ++Irvl)
		Irvl->first->set_value(Irvl->second);

	if (Is == mdp_->get_states().end())
		state_update_begin_ = mdp_->get_states().begin();

	// Evaluate the policy
	do
	{
		std::list<double>::iterator Ivl = record_values_.begin();
		
		for (GraphMDP<GraphStateWTA<GraphStateVariance> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
		{
			*Ivl = Ist->second->get_value();
			++Ivl;
		}
		
		Ivl = record_values_.begin();
		bellman_error_ = 0.0;
		
		for (GraphMDP<GraphStateWTA<GraphStateVariance> >::states_map_t::iterator Ist = mdp_->_states_().begin() ; Ist != mdp_->_states_().end() ; ++Ist)
		{
			Ist->second->set_value(compute_q_value(Ist->second->get_transitions().find(&(Ist->second->get_policy()))->second));
			bellman_error_ = std::max(bellman_error_, std::fabs((Ist->second->get_value()) - (*Ivl)));
			++Ivl;
		}
	}
	while(bellman_error_ >= epsilon_);
	
	// Did the policy change? (checked only after the last cycle of updated states is reached
	if (state_update_begin_ == mdp_->get_states().begin())
	{
		policy_changed_ = false;
		std::list<const Action*>::iterator Iac = record_actions_.begin();
		
		for (GraphMDP<GraphStateWTA<GraphStateVariance> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
		{
			if ((*Iac) != &(Ist->second->get_policy()))
			{
				policy_changed_ = true;
				break;
			}
			
			++Iac;
		}
	}
}


bool GraphMVPFSSP::last_mean_level() const
{
	if (policy_changed_)
		return false;
	else
		return true;
}

