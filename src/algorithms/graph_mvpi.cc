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
#include "graph_mvpi.h"

GraphMVPI::GraphMVPI(double proportion_updated_states)
	: proportion_updated_states_(proportion_updated_states)
{
}




//////////////////////////////
GraphMVPFPI::GraphMVPFPI(const Problem& pb, double epsilon, double discount_factor, double proportion_updated_states)
try	: BaseAlgorithm(pb, epsilon, discount_factor),
		GraphMVPI(proportion_updated_states),
		GraphMVPF<GraphStateWT<GraphStateVariance> >(pb, epsilon, discount_factor)
{
	tb_->set_metric(TransitionsBlackbox::METRIC_REWARD);
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphMVPFPI::GraphMVPFPI");
	throw;
}


void GraphMVPFPI::initial_mean_level_policy()
{
	record_actions_.resize(nb_states_, NULL);

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
	
	// We have now a policy with worst value function to start from
	policy_changed_ = true;
	
	// The improvement of the policy begins from the initial state
	state_update_begin_ = mdp_->get_states().begin();
}


void GraphMVPFPI::next_mean_level_policy()
{
	// Record the policy if we improve the policy from the initial state (new optimization cycle)
	if (state_update_begin_ == mdp_->get_states().begin())
	{
		std::list<const Action*>::iterator Iac = record_actions_.begin();
		
		for (GraphMDP<GraphStateWT<GraphStateVariance> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
		{
			*Iac = &(Ist->second->get_policy());
			++Iac;
		}
	}
	
	// Improve the policy
	unsigned int nbs = 0;
	GraphMDP<GraphStateWT<GraphStateVariance> >::states_map_t::const_iterator Is;
	std::map<GraphStateWT<GraphStateVariance>*, double> record_val;
	
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
	
	for (std::map<GraphStateWT<GraphStateVariance>*, double>::const_iterator Irvl = record_val.begin() ; Irvl != record_val.end() ; ++Irvl)
		Irvl->first->set_value(Irvl->second);

	if (Is == mdp_->get_states().end())
		state_update_begin_ = mdp_->get_states().begin();

	// Evaluate the policy
	do
	{
		std::list<double>::iterator Ivl = record_values_.begin();
		
		for (GraphMDP<GraphStateWT<GraphStateVariance> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
		{
			*Ivl = Ist->second->get_value();
			++Ivl;
		}
		
		Ivl = record_values_.begin();
		bellman_error_ = 0.0;
		
		for (GraphMDP<GraphStateWT<GraphStateVariance> >::states_map_t::iterator Ist = mdp_->_states_().begin() ; Ist != mdp_->_states_().end() ; ++Ist)
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
		
		for (GraphMDP<GraphStateWT<GraphStateVariance> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
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


bool GraphMVPFPI::last_mean_level() const
{
	if (policy_changed_)
		return false;
	else
		return true;
}



/////////////////////////
GraphMVHOPI::GraphMVHOPI(const Problem& pb, double minimum_mean, double maximum_variance, double epsilon, double discount_factor, double proportion_updated_states)
try	: BaseAlgorithm(pb, epsilon, discount_factor),
		GraphMVPI(proportion_updated_states),
		GraphMVHO<GraphStateWT<GraphStateVariance> >(pb, minimum_mean, maximum_variance, epsilon, discount_factor)
{
	tb_->set_metric(TransitionsBlackbox::METRIC_REWARD);
	current_proportion_updated_states_ = proportion_updated_states_ * 10.0;
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphMVHOPI::GraphMVHOPI");
	throw;
}


void GraphMVHOPI::initial_mean_level_policy()
{
	current_proportion_updated_states_ /= 10.0;
	record_actions_.resize(nb_states_, NULL);

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
	
	// We have now a policy with worst value function to start from
	policy_changed_ = true;
	
	// The improvement of the policy begins from the initial state
	state_update_begin_ = mdp_->get_states().begin();
}


void GraphMVHOPI::next_mean_level_policy()
{
	// Record the policy if we improve the policy from the initial state (new optimization cycle)
	if (state_update_begin_ == mdp_->get_states().begin())
	{
		std::list<const Action*>::iterator Iac = record_actions_.begin();
		
		for (GraphMDP<GraphStateWT<GraphStateVariance> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
		{
			*Iac = &(Ist->second->get_policy());
			++Iac;
		}
	}
	
	// Degrade the policy
	unsigned int nbs = 0;
	GraphMDP<GraphStateWT<GraphStateVariance> >::states_map_t::const_iterator Is;
	for (Is = state_update_begin_ ; Is != mdp_->get_states().end() ; ++Is)
	{
		if (nbs > static_cast<unsigned int>(std::floor(nb_states_ * proportion_updated_states_)))
		{
			state_update_begin_ = Is;
			break;
		}
		
		if (Is->second->get_transitions().empty())
				return;

		double worst_q_value = compute_q_value(Is->second->get_transitions().begin()->second);
		const Action* worst_action = Is->second->get_transitions().begin()->first;

		for (actions_map_t::const_iterator Iac = ++(Is->second->get_transitions().begin()) ; Iac != Is->second->get_transitions().end() ; ++Iac)
		{
			double q_value = compute_q_value(Iac->second);

			if (q_value < worst_q_value)
			{
				worst_q_value = q_value;
				worst_action = Iac->first;
			}
		}

		Is->second->set_value(worst_q_value);
		Is->second->set_policy(*worst_action);
		
		nbs++;
	}

	if (Is == mdp_->get_states().end())
		state_update_begin_ = mdp_->get_states().begin();

	// Evaluate the policy
	do
	{
		std::list<double>::iterator Ivl = record_values_.begin();
		
		for (GraphMDP<GraphStateWT<GraphStateVariance> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
		{
			*Ivl = Ist->second->get_value();
			++Ivl;
		}
		
		Ivl = record_values_.begin();
		bellman_error_ = 0.0;
		
		for (GraphMDP<GraphStateWT<GraphStateVariance> >::states_map_t::iterator Ist = mdp_->_states_().begin() ; Ist != mdp_->_states_().end() ; ++Ist)
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
		
		for (GraphMDP<GraphStateWT<GraphStateVariance> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
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


bool GraphMVHOPI::last_mean_level() const
{
	if (policy_changed_)
		return false;
	else
		return true;
}
