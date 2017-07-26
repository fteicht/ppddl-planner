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
#include "graph_mvrt.h"

GraphMVRT::GraphMVRT(const Problem& pb, double epsilon, double discount_factor)
try : BaseAlgorithm(pb, epsilon, discount_factor), GraphAlgorithm<GraphStateWT<> >(pb, epsilon, discount_factor)
{
    tb_->set_metric(TransitionsBlackbox::METRIC_REWARD);
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphMVRT::GraphMVRT");
	throw;
}


bool GraphMVRT::policy_defined(const PddlState& st)
{
	return ((mdp_->explored(st)) && (mdp_->get_state(st).policy_defined()));
}


void GraphMVRT::solve_initialize(const PddlState& st)
{
	// generate all reachable states
	mdp_->add_state(st);
	GraphMDP<GraphStateWT<> >::new_states_list_t new_states;
	new_states.push_back(mdp_->_states_().find(&st));
	initial_state_ = mdp_->_states_().find(&st)->second;
	
	while (!new_states.empty())
	{
		mdp_->transitions(*(new_states.front()->second));
		new_states.pop_front();
		new_states.insert(new_states.end(), mdp_->new_states().begin(), mdp_->new_states().end());
	}

	nb_states_ = 0;
	for (GraphMDP<GraphStateWT<> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
	{
		variance_values_.insert(std::make_pair(Ist->second, 0.0));
		nb_states_++;
	}
	
	record_values_.resize(nb_states_, 0.0);
	policy_set_.resize(nb_states_);

	// compute a policy with maximum value function
	do
	{
		// record values
		std::list<double>::iterator Ivl = record_values_.begin();
		
		for (GraphMDP<GraphStateWT<> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
		{
			*Ivl = Ist->second->get_value();
			++Ivl;
		}

		// improve values
		Ivl = record_values_.begin();
		bellman_error_ = 0.0;
		
		for (GraphMDP<GraphStateWT<> >::states_map_t::iterator Ist = mdp_->_states_().begin() ; Ist != mdp_->_states_().end() ; ++Ist)
		{
			improve_state_value(*(Ist->second));
			bellman_error_ = std::max(bellman_error_, std::fabs((Ist->second->get_value()) - (*Ivl)));
			++Ivl;
		}
	}
	while (bellman_error_ >= epsilon_);

	// Compute the initial set of policies
	std::list<std::set<const Action*> >::iterator Ips = policy_set_.begin();
	
	for (GraphMDP<GraphStateWT<> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
	{
		for (typename actions_map_t::const_iterator Iac = Ist->second->get_transitions().begin() ; Iac != Ist->second->get_transitions().end() ; ++Iac)
		{
			if (std::fabs(compute_q_value(Iac->second) - (Ist->second->get_value())) < epsilon_)
				*Ips->insert(Iac->first);
		}

		++Ips;
	}

	converged_ = false;
}


void GraphMVRT::solve_progress()
{
	if ((initial_state_->get_value()) < minimum_mean_)
	{
		std::cerr << "unsolvable problem: MLB < R" << std::endl;
		converged_ = true;
		return;
	}

	// minimize the variance
	minimize_variance();
	std::cout << "current solution: M=" << (initial_state_->get_value()) << " V=" << variance_values_[initial_state_] << std::endl;

	if (variance_values_[initial_state_] <= maximum_variance_)
	{
		std::cout << "result found: M=" << (initial_state_->get_value()) << " V=" << variance_values_[initial_state_] << std::endl;
		converged_ = true;
		return;
	}

	// enlarge policy set
	if (!enlarge_policy_set())
	{
		std::cerr << "unsolvable problem: policy set cannot be enlarged anymore" << std::endl;
		converged_ = true;
		return;
	}
	
	// compute a policy with minimum value function over the current policy set (the worst possible one)
	do
	{
		// record values
		std::list<double>::iterator Ivl = record_values_.begin();
		
		for (GraphMDP<GraphStateWT<> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
		{
			*Ivl = Ist->second->get_value();
			++Ivl;
		}
		
		// degrade state values
		std::list<std::set<const Action*> >::iterator Ips = policy_set_.begin();
		Ivl = record_values_.begin();
		bellman_error_ = 0.0;
		
		for (GraphMDP<GraphStateWT<> >::states_map_t::iterator Ist = mdp_->_states_().begin() ; Ist != mdp_->_states_().end() ; ++Ist)
		{
			
			
			if (Ist->second->get_transitions().empty())
				return;
			
			double worst_q_value = std::numeric_limits<double>::max();
			const Action* worst_action = NULL;
			
			for (typename actions_map_t::const_iterator Iac = Ist->second->get_transitions().begin() ; Iac != Ist->second->get_transitions().end() ; ++Iac)
			{
				if (Ips->find(Iac->first) != Ips->end())
				{
					double q_value = compute_q_value(Iac->second);

					if (q_value < worst_q_value)
					{
						worst_q_value = q_value;
						worst_action = Iac->first;
					}
				}
			}
			
			Ist->second->set_value(worst_q_value);
			Ist->second->set_policy(*worst_action);
			
			bellman_error_ = std::max(bellman_error_, std::fabs((Ist->second->get_value()) - (*Ivl)));
			++Ivl;
			++Ips;
		}
	}
	while (bellman_error_ > epsilon_);
}


bool GraphMVRT::has_converged()
{
	return converged_;
}


void GraphMVRT::minimize_variance()
{
	// compute a policy with minimum variance for the current value function memorized in states
	do
	{
		// record values
		std::list<double>::iterator Ivl = record_values_.begin();
		
		for (GraphMDP<GraphStateWT<> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
		{
			*Ivl = variance_values_[Ist->second];
			++Ivl;
		}
		
		// degrade state values
		Ivl = record_values_.begin();
		bellman_error_ = 0.0;
		
		for (GraphMDP<GraphStateWT<> >::states_map_t::iterator Ist = mdp_->_states_().begin() ; Ist != mdp_->_states_().end() ; ++Ist)
		{
			if (Ist->second->get_transitions().empty())
				return;
			
			double best_q_value = std::numeric_limits<double>::max();
			//const Action* best_action = NULL;
			
			for (typename actions_map_t::const_iterator Iac = Ist->second->get_transitions().begin() ; Iac != Ist->second->get_transitions().end() ; ++Iac)
			{
				if (std::fabs(compute_q_value(Iac->second) - (Ist->second->get_value())) < epsilon_)
				{
					double q_value = compute_variance_q_value(Iac->second);
					
					if (q_value < best_q_value)
					{
						best_q_value = q_value;
						//best_action = Iac->first;
					}
				}
			}
			
			variance_values_[Ist->second] = best_q_value;
			bellman_error_ = std::max(bellman_error_, std::fabs(best_q_value - (*Ivl)));
			++Ivl;
		}
	}
	while (bellman_error_ > epsilon_);
}


double GraphMVRT::compute_variance_q_value(const transitions_list_t& effects)
{
	double s1 = 0.0;
	double s2 = 0.0;
	double s3 = 0.0;
	
	for (typename transitions_list_t::const_iterator Ieff = effects.begin() ; Ieff != effects.end() ; ++Ieff)
	{
		s1 += ((*Ieff)->get_probability()) * ((*Ieff)->get_outcome().get_value()) * ((*Ieff)->get_outcome().get_value());
		s2 += ((*Ieff)->get_probability()) * ((*Ieff)->get_outcome().get_value());
		s3 += ((*Ieff)->get_probability()) * variance_values_[&((*Ieff)->get_outcome())];
	}
	
	return discount_factor_ * discount_factor_ * (s1 - (s2 * s2) + s3);
}


bool GraphMVRT::enlarge_policy_set()
{
	std::list<std::set<const Action*> >::iterator Ips = policy_set_.begin();

	for (GraphMDP<GraphStateWT<> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
	{
		if (std::rand() <= probability_enlarge_state_ * RAND_MAX)
		{
			for (typename actions_map_t::const_iterator Iac = Ist->second->get_transitions().begin() ; Iac != Ist->second->get_transitions().end() ; ++Iac)
			{
				if (std::rand() <= probability_enlarge_action_ * RAND_MAX)
				{
				}
			}
		}
		
		++Ips;
	}
}
