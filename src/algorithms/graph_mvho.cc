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
#include "graph_mvho.h"
#include "models/graph_mdp.h"

template <typename Tstate>
GraphMVHO<Tstate>::GraphMVHO(const Problem& pb, double minimum_mean, double maximum_variance, double epsilon, double discount_factor)
try : BaseAlgorithm(pb, epsilon, discount_factor), GraphAlgorithm<Tstate>(pb, epsilon, discount_factor),
	minimum_mean_(minimum_mean), maximum_variance_(maximum_variance)
{
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphMVHO::GraphMVHO");
	throw;
}


template <typename Tstate>
bool GraphMVHO<Tstate>::policy_defined(const PddlState& st)
{
	return ((this->mdp_->explored(st)) && (this->mdp_->get_state(st).policy_defined()));
}


template <typename Tstate>
void GraphMVHO<Tstate>::solve_initialize(const PddlState& st)
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
	
	std::cerr << "FIRST ATTEMPT" << std::endl;
	restart_ = true;
	success_ = false;
}


template <typename Tstate>
void GraphMVHO<Tstate>::solve_progress()
{
	if (restart_)
		initial_mean_level_policy();
	else
		next_mean_level_policy();
	
	restart_ = false;
	minimize_variance();
	print_pareto_point();
	
	if ((initial_state_->get_value()) < minimum_mean_)
	{
		restart_ = true;
		std::cerr << "NEW ATTEMPT (mean constraint unsatisfied)" << std::endl;
	}
	else if ((initial_state_->get_variance()) <= maximum_variance_)
		success_ = true;
	else if (last_mean_level())
	{
		restart_ = true;
		std::cerr << "NEW ATTEMPT (last mean level reached)" << std::endl;
	}
}


template <typename Tstate>
bool GraphMVHO<Tstate>::has_converged()
{
	return success_;
}


template <typename Tstate>
void GraphMVHO<Tstate>::minimize_variance()
{
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
		
		// degrade state variance values
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
	}
	while ((this->bellman_error_) >= (this->epsilon_));
}


template <typename Tstate>
double GraphMVHO<Tstate>::compute_variance_q_value(const typename GraphAlgorithm<Tstate>::transitions_list_t& effects)
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
void GraphMVHO<Tstate>::print_pareto_point() const
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

template class GraphMVHO<GraphStateWT<GraphStateVariance> >;
template class GraphMVHO<GraphStateWTA<GraphStateVariance> >;
