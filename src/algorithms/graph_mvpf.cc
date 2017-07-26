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
#include "graph_mvpf.h"
#include "models/graph_mdp.h"

template <typename Tstate>
GraphMVPF<Tstate>::GraphMVPF(const Problem& pb, double epsilon, double discount_factor)
try : BaseAlgorithm(pb, epsilon, discount_factor), GraphAlgorithm<Tstate>(pb, epsilon, discount_factor)
{
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphMVPF::GraphMVPF");
	throw;
}


template <typename Tstate>
bool GraphMVPF<Tstate>::policy_defined(const PddlState& st)
{
	return ((this->mdp_->explored(st)) && (this->mdp_->get_state(st).policy_defined()));
}


template <typename Tstate>
void GraphMVPF<Tstate>::solve_initialize(const PddlState& st)
{
	// generate all reachable states
	this->mdp_->add_state(st);
	typename GraphMDP<Tstate>::new_states_list_t new_states;
	new_states.push_back(this->mdp_->_states_().find(&st));
	initial_state_ = this->mdp_->_states_().find(&st)->second;
	
	while (!new_states.empty())
	{
		this->mdp_->transitions(*(new_states.front()->second));
		new_states.pop_front();
		new_states.insert(new_states.end(), this->mdp_->new_states().begin(), this->mdp_->new_states().end());
	}

	nb_states_ = this->mdp_->get_states().size();
	record_values_.resize(nb_states_, 0.0);
	
	initial_mean_level_policy();
	minimize_variance_pi();
	print_pareto_point();
}


template <typename Tstate>
void GraphMVPF<Tstate>::solve_progress()
{
	next_mean_level_policy();
	minimize_variance_pi();
	print_pareto_point();
}


template <typename Tstate>
bool GraphMVPF<Tstate>::has_converged()
{
	if (last_mean_level())
		return true;
	else
		return false;
}


template <typename Tstate>
void GraphMVPF<Tstate>::minimize_variance_vi()
{
	std::cout << "BEGIN VARIANCE VI" << std::endl;
	evaluate_variance(); // only to begin with a variance corresponding of an existing policy (for printing purposes only)
	
	// compute a policy with minimum variance for the current mean value function memorized in states
	do
	{
		// record values
		std::list<double>::iterator Ivl = record_values_.begin();
		
		for (typename GraphMDP<Tstate>::states_map_t::const_iterator Ist = this->mdp_->get_states().begin() ; Ist != this->mdp_->get_states().end() ; ++Ist)
		{
			*Ivl = Ist->second->get_variance();
			++Ivl;
		}
		
		// decrease state variance values
		Ivl = record_values_.begin();
		this->bellman_error_ = 0.0;
		
		for (typename GraphMDP<Tstate>::states_map_t::iterator Ist = this->mdp_->_states_().begin() ; Ist != this->mdp_->_states_().end() ; ++Ist)
		{
			if (Ist->second->get_transitions().empty())
				return;
			
			double best_q_value = std::numeric_limits<double>::max();
			//const Action* best_action = NULL;
			
			for (typename GraphAlgorithm<Tstate>::actions_map_t::const_iterator Iac = Ist->second->get_transitions().begin() ; Iac != Ist->second->get_transitions().end() ; ++Iac)
			{
				if (std::fabs((this->compute_q_value(Iac->second)) - (Ist->second->get_value())) < (this->epsilon_))
				{
					double q_value = compute_variance_q_value(Iac->second);
					
					if (q_value < best_q_value)
					{
						best_q_value = q_value;
						//best_action = Iac->first;
					}
				}
			}
			
			Ist->second->set_variance(best_q_value);
			this->bellman_error_ = std::max(this->bellman_error_, std::fabs(best_q_value - (*Ivl)));
			++Ivl;
		}
		
		// print Pareto point (does not need to correspond to an existing policy!)
		print_pareto_point();
	}
	while ((this->bellman_error_) >= (this->epsilon_));
	
	std::cout << "END VARIANCE VI" << std::endl;
}


template <typename Tstate>
void GraphMVPF<Tstate>::minimize_variance_pi()
{
	// Compute the worst variance policy
	do
	{
		// record values
		std::list<double>::iterator Ivl = record_values_.begin();
		
		for (typename GraphMDP<Tstate>::states_map_t::const_iterator Ist = this->mdp_->get_states().begin() ; Ist != this->mdp_->get_states().end() ; ++Ist)
		{
			*Ivl = Ist->second->get_variance();
			++Ivl;
		}
		
		// increase state variance values
		Ivl = record_values_.begin();
		this->bellman_error_ = 0.0;
		
		for (typename GraphMDP<Tstate>::states_map_t::iterator Ist = this->mdp_->_states_().begin() ; Ist != this->mdp_->_states_().end() ; ++Ist)
		{
			if (Ist->second->get_transitions().empty())
				return;
			
			double worst_q_value = -std::numeric_limits<double>::max();
			const Action* worst_action = NULL;
			
			for (typename GraphAlgorithm<Tstate>::actions_map_t::const_iterator Iac = Ist->second->get_transitions().begin() ; Iac != Ist->second->get_transitions().end() ; ++Iac)
			{
				if (std::fabs((this->compute_q_value(Iac->second)) - (Ist->second->get_value())) < (this->epsilon_))
				{
					double q_value = compute_variance_q_value(Iac->second);
					
					if (q_value > worst_q_value)
					{
						worst_q_value = q_value;
						worst_action = Iac->first;
					}
				}
			}
			
			Ist->second->set_variance(worst_q_value);
			Ist->second->set_policy(*worst_action);
			this->bellman_error_ = std::max(this->bellman_error_, std::fabs(worst_q_value - (*Ivl)));
			++Ivl;
		}
	}
	while ((this->bellman_error_) >= (this->epsilon_));
	
	// Perform policy iteration on the variance
	//std::list<const Action*> record_actions;
	//record_actions.resize(nb_states_, NULL);
	bool variance_policy_converged = false;
	
	std::cout << "BEGIN VARIANCE PI" << std::endl;
	
	do
	{
		// evaluate policy
		evaluate_variance();
		print_pareto_point();
		
		// record policy
		//std::list<const Action*>::iterator Iac = record_actions.begin();
		//for (typename GraphMDP<Tstate>::states_map_t::const_iterator Ist = this->mdp_->get_states().begin() ; Ist != this->mdp_->get_states().end() ; ++Ist)
		//{
		//	*Iac = &(Ist->second->get_policy());
		//	++Iac;
		//}
		
		// improve policy (decrease variance)
		variance_policy_converged = true;
		std::list<double>::iterator Ivl = record_values_.begin();
		
		for (typename GraphMDP<Tstate>::states_map_t::const_iterator Ist = this->mdp_->get_states().begin() ; Ist != this->mdp_->get_states().end() ; ++Ist)
		{
			if (Ist->second->get_transitions().empty())
				return;
			
			double best_q_value = std::numeric_limits<double>::max();
			const Action* best_action = NULL;
			
			for (typename GraphAlgorithm<Tstate>::actions_map_t::const_iterator Iac = Ist->second->get_transitions().begin() ; Iac != Ist->second->get_transitions().end() ; ++Iac)
			{
				if (std::fabs((this->compute_q_value(Iac->second)) - (Ist->second->get_value())) < (this->epsilon_))
				{
					double q_value = compute_variance_q_value(Iac->second);
					
					if (q_value < best_q_value)
					{
						best_q_value = q_value;
						best_action = Iac->first;
					}
				}
			}
			
			//Ist->second->set_variance(best_q_value);
			*Ivl = best_q_value;
			variance_policy_converged = variance_policy_converged && (best_action == &(Ist->second->get_policy()));
			Ist->second->set_policy(*best_action);
			++Ivl;
		}
		
		// record values
		Ivl = record_values_.begin();
		for (typename GraphMDP<Tstate>::states_map_t::const_iterator Ist = this->mdp_->get_states().begin() ; Ist != this->mdp_->get_states().end() ; ++Ist)
		{
			Ist->second->set_variance(*Ivl);
			++Ivl;
		}
	}
	while (!variance_policy_converged);
	
	std::cout << "END VARIANCE PI" << std::endl;
}


template <typename Tstate>
void GraphMVPF<Tstate>::evaluate_variance()
{
	do
	{
		// record values
		std::list<double>::iterator Ivl = record_values_.begin();
		
		for (typename GraphMDP<Tstate>::states_map_t::const_iterator Ist = this->mdp_->get_states().begin() ; Ist != this->mdp_->get_states().end() ; ++Ist)
		{
			*Ivl = Ist->second->get_variance();
			++Ivl;
		}
		
		// update state variance values
		Ivl = record_values_.begin();
		this->bellman_error_ = 0.0;
		
		for (typename GraphMDP<Tstate>::states_map_t::iterator Ist = this->mdp_->_states_().begin() ; Ist != this->mdp_->_states_().end() ; ++Ist)
		{
			Ist->second->set_variance(compute_variance_q_value(Ist->second->get_transitions().find(&(Ist->second->get_policy()))->second));
			this->bellman_error_ = std::max(this->bellman_error_, std::fabs(Ist->second->get_variance() - (*Ivl)));
			++Ivl;
		}
	}
	while ((this->bellman_error_) >= (this->epsilon_));
}


template <typename Tstate>
double GraphMVPF<Tstate>::compute_variance_q_value(const typename GraphAlgorithm<Tstate>::transitions_list_t& effects)
{
	double s1 = 0.0;
	double s2 = 0.0;
	double s3 = 0.0;
	
	for (typename GraphAlgorithm<Tstate>::transitions_list_t::const_iterator Ieff = effects.begin() ; Ieff != effects.end() ; ++Ieff)
	{
		s1 += ((*Ieff)->get_probability()) * ((*Ieff)->get_outcome().get_value()) * ((*Ieff)->get_outcome().get_value());
		s2 += ((*Ieff)->get_probability()) * ((*Ieff)->get_outcome().get_value());
		s3 += ((*Ieff)->get_probability()) * ((*Ieff)->get_outcome().get_variance());
	}
	
	return ((this->discount_factor_) * (this->discount_factor_) * (s1 - (s2 * s2) + s3));
}


template <typename Tstate>
void GraphMVPF<Tstate>::print_pareto_point() const
{
	double average_mean = 0.0;
	double average_standard_deviation = 0.0;
	
	for (typename GraphMDP<Tstate>::states_map_t::const_iterator Ist = this->mdp_->get_states().begin() ; Ist != this->mdp_->get_states().end() ; ++Ist)
	{
		average_mean += (Ist->second->get_value()) / ((double) nb_states_);
		
		if ((Ist->second->get_variance()) > 0.0) // Zero variance can be negative with an extremely little magnitude!
			average_standard_deviation += std::sqrt(Ist->second->get_variance()) / ((double) nb_states_);
	}
	
	std::cout << (initial_state_->get_value()) << "   " << (((initial_state_->get_variance()) > 0.0)?(std::sqrt(initial_state_->get_variance())):0.0) << "        " << average_mean << "    " << average_standard_deviation << std::endl;
}


// Explicit template instantiations

template class GraphMVPF<GraphStateWT<GraphStateVariance> >;
template class GraphMVPF<GraphStateWTA<GraphStateVariance> >;
