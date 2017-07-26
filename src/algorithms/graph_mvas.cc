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
#include <cstdlib>
#include "graph_mvas.h"

GraphMVAS::GraphMVAS()
{
}




////////////////////////////////
GraphMVPFAS::GraphMVPFAS(const Problem& pb, double epsilon, double discount_factor)
try	: BaseAlgorithm(pb, epsilon, discount_factor),
		GraphMVAS(),
		GraphMVPF<GraphStateWT<GraphStateVariance> >(pb, epsilon, discount_factor)
{
	tb_->set_metric(TransitionsBlackbox::METRIC_REWARD);
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphMVPFAS::GraphMVPFAS");
	throw;
}


void GraphMVPFAS::initial_mean_level_policy()
{
	// compute a policy with minimum value function (the worst possible one)
	do
	{
		// record values
		std::list<double>::iterator Ivl = record_values_.begin();
		
		for (GraphMDP<GraphStateWT<GraphStateVariance> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
		{
			*Ivl = Ist->second->get_value();
			++Ivl;
		}
		
		// degrade state values
		Ivl = record_values_.begin();
		bellman_error_ = 0.0;
		
		for (GraphMDP<GraphStateWT<GraphStateVariance> >::states_map_t::iterator Ist = mdp_->_states_().begin() ; Ist != mdp_->_states_().end() ; ++Ist)
		{
			if (Ist->second->get_transitions().empty())
				return;
			
			double worst_q_value = compute_q_value(Ist->second->get_transitions().begin()->second);
			const Action* worst_action = Ist->second->get_transitions().begin()->first;
			
			for (actions_map_t::const_iterator Iac = ++(Ist->second->get_transitions().begin()) ; Iac != Ist->second->get_transitions().end() ; ++Iac)
			{
				double q_value = compute_q_value(Iac->second);
				
				if (q_value < worst_q_value)
				{
					worst_q_value = q_value;
					worst_action = Iac->first;
				}
			}
			
			Ist->second->set_value(worst_q_value);
			Ist->second->set_policy(*worst_action);
			
			bellman_error_ = std::max(bellman_error_, std::fabs((Ist->second->get_value()) - (*Ivl)));
			++Ivl;
		}
	}
	while (bellman_error_ >= epsilon_);
	
	// Compute the initial set of policies
	action_sets_.resize(nb_states_, ActionSetFactory());
	unconverged_action_sets_.resize(nb_states_);
	std::vector<ActionSetFactory>::iterator Ias = action_sets_.begin();
	std::vector<unsigned int>::iterator Iuas = unconverged_action_sets_.begin();
	unsigned int snb = 0;
	
	for (GraphMDP<GraphStateWT<GraphStateVariance> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
	{
		for (actions_map_t::const_iterator Iac = Ist->second->get_transitions().begin() ; Iac != Ist->second->get_transitions().end() ; ++Iac)
		{
			if (std::fabs(compute_q_value(Iac->second) - (Ist->second->get_value())) < epsilon_)
				Ias->policy_set_.push_back(Iac);
			else
				Ias->candidate_set_.push_back(Iac);
		}
		
		if (Ias->candidate_set_.empty())
			Iuas = unconverged_action_sets_.erase(Iuas);
		else
		{
			*Iuas = snb;
			++Iuas;
		}

		++Ias;
		snb++;
	}
}


void GraphMVPFAS::next_mean_level_policy()
{
	// Non-deterministically increase the policy set
	unsigned int pick = std::rand() % (unconverged_action_sets_.size());
	ActionSetFactory& unconverged_state = action_sets_[unconverged_action_sets_[pick]];
	unsigned int candidate_action = std::rand() % (unconverged_state.candidate_set_.size());
	unconverged_state.policy_set_.push_back(unconverged_state.candidate_set_[candidate_action]);
	unconverged_state.candidate_set_.erase(unconverged_state.candidate_set_.begin() + candidate_action);
	if (unconverged_state.candidate_set_.empty()) unconverged_action_sets_.erase(unconverged_action_sets_.begin() + pick);
	
	// Compute a maximum mean policy on the enlarged policy set
	do
	{
		// record values
		std::list<double>::iterator Ivl = record_values_.begin();
		
		for (GraphMDP<GraphStateWT<GraphStateVariance> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
		{
			*Ivl = Ist->second->get_value();
			++Ivl;
		}
		
		// improve state values
		Ivl = record_values_.begin();
		bellman_error_ = 0.0;
		std::vector<ActionSetFactory>::const_iterator Ias = action_sets_.begin();
		
		for (GraphMDP<GraphStateWT<GraphStateVariance> >::states_map_t::iterator Ist = mdp_->_states_().begin() ; Ist != mdp_->_states_().end() ; ++Ist)
		{
			if (Ias->policy_set_.empty())
				return;
			
			double best_q_value = - std::numeric_limits<double>::max();
			const Action* best_action = NULL;
			
			for (std::list<GraphMDP<GraphStateWT<GraphStateVariance> >::actions_map_t::const_iterator>::const_iterator Iac = Ias->policy_set_.begin() ; Iac != Ias->policy_set_.end() ; ++Iac)
			{
				double q_value = compute_q_value((*Iac)->second);
				
				if (q_value > best_q_value)
				{
					best_q_value = q_value;
					best_action = (*Iac)->first;
				}
			}
			
			Ist->second->set_value(best_q_value);
			Ist->second->set_policy(*best_action);
			
			bellman_error_ = std::max(bellman_error_, std::fabs((Ist->second->get_value()) - (*Ivl)));
			++Ivl;
			++Ias;
		}
	}
	while (bellman_error_ >= epsilon_);
}


bool GraphMVPFAS::last_mean_level() const
{
	return unconverged_action_sets_.empty();
}




///////////////////////////////
GraphMVHOAS::GraphMVHOAS(const Problem& pb, double minimum_mean, double maximum_variance, double epsilon, double discount_factor)
try	: BaseAlgorithm(pb, epsilon, discount_factor),
		GraphMVAS(),
		GraphMVHO<GraphStateWT<GraphStateVariance> >(pb, minimum_mean, maximum_variance, epsilon, discount_factor)
{
	tb_->set_metric(TransitionsBlackbox::METRIC_REWARD);
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphMVHOAS::GraphMVHOAS");
	throw;
}


void GraphMVHOAS::initial_mean_level_policy()
{
	// compute a policy with maximum value function (the best possible one)
	do
	{
		// record values
		std::list<double>::iterator Ivl = record_values_.begin();
		
		for (GraphMDP<GraphStateWT<GraphStateVariance> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
		{
			*Ivl = Ist->second->get_value();
			++Ivl;
		}
		
		// improve state values
		Ivl = record_values_.begin();
		bellman_error_ = 0.0;
		
		for (GraphMDP<GraphStateWT<GraphStateVariance> >::states_map_t::iterator Ist = mdp_->_states_().begin() ; Ist != mdp_->_states_().end() ; ++Ist)
		{
			improve_state_value(*(Ist->second));
			bellman_error_ = std::max(bellman_error_, std::fabs((Ist->second->get_value()) - (*Ivl)));
			++Ivl;
		}
	}
	while (bellman_error_ >= epsilon_);
	
	// Compute the initial set of policies
	action_sets_.clear();
	action_sets_.resize(nb_states_, ActionSetFactory());
	unconverged_action_sets_.clear();
	unconverged_action_sets_.resize(nb_states_);
	std::vector<ActionSetFactory>::iterator Ias = action_sets_.begin();
	std::vector<unsigned int>::iterator Iuas = unconverged_action_sets_.begin();
	unsigned int snb = 0;
	
	for (GraphMDP<GraphStateWT<GraphStateVariance> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
	{
		for (actions_map_t::const_iterator Iac = Ist->second->get_transitions().begin() ; Iac != Ist->second->get_transitions().end() ; ++Iac)
		{
			if (std::fabs(compute_q_value(Iac->second) - (Ist->second->get_value())) < epsilon_)
				Ias->policy_set_.push_back(Iac);
			else
				Ias->candidate_set_.push_back(Iac);
		}
		
		if (Ias->candidate_set_.empty())
			Iuas = unconverged_action_sets_.erase(Iuas);
		else
		{
			*Iuas = snb;
			++Iuas;
		}

		++Ias;
		snb++;
	}
}


void GraphMVHOAS::next_mean_level_policy()
{
	// Non-deterministically increase the policy set
	unsigned int pick = std::rand() % (unconverged_action_sets_.size());
	ActionSetFactory& unconverged_state = action_sets_[unconverged_action_sets_[pick]];
	unsigned int candidate_action = std::rand() % (unconverged_state.candidate_set_.size());
	unconverged_state.policy_set_.push_back(unconverged_state.candidate_set_[candidate_action]);
	unconverged_state.candidate_set_.erase(unconverged_state.candidate_set_.begin() + candidate_action);
	if (unconverged_state.candidate_set_.empty()) unconverged_action_sets_.erase(unconverged_action_sets_.begin() + pick);
	
	// Compute a minimum mean policy on the enlarged policy set
	do
	{
		// record values
		std::list<double>::iterator Ivl = record_values_.begin();
		
		for (GraphMDP<GraphStateWT<GraphStateVariance> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
		{
			*Ivl = Ist->second->get_value();
			++Ivl;
		}
		
		// degrade state values
		Ivl = record_values_.begin();
		bellman_error_ = 0.0;
		std::vector<ActionSetFactory>::const_iterator Ias = action_sets_.begin();
		
		for (GraphMDP<GraphStateWT<GraphStateVariance> >::states_map_t::iterator Ist = mdp_->_states_().begin() ; Ist != mdp_->_states_().end() ; ++Ist)
		{
			if (Ias->policy_set_.empty())
				return;
			
			double worst_q_value = std::numeric_limits<double>::max();
			const Action* best_action = NULL;
			
			for (std::list<GraphMDP<GraphStateWT<GraphStateVariance> >::actions_map_t::const_iterator>::const_iterator Iac = Ias->policy_set_.begin() ; Iac != Ias->policy_set_.end() ; ++Iac)
			{
				double q_value = compute_q_value((*Iac)->second);
				
				if (q_value < worst_q_value)
				{
					worst_q_value = q_value;
					best_action = (*Iac)->first;
				}
			}
			
			Ist->second->set_value(worst_q_value);
			Ist->second->set_policy(*best_action);
			
			bellman_error_ = std::max(bellman_error_, std::fabs((Ist->second->get_value()) - (*Ivl)));
			++Ivl;
			++Ias;
		}
	}
	while (bellman_error_ >= epsilon_);
}


bool GraphMVHOAS::last_mean_level() const
{
	return unconverged_action_sets_.empty();
}
