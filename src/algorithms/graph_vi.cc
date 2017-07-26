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
#include "graph_vi.h"


GraphVI::GraphVI(const Problem& pb, double epsilon, double discount_factor)
try : BaseAlgorithm(pb, epsilon, discount_factor), GraphAlgorithm<GraphStateWT<> >(pb, epsilon, discount_factor)
{
    tb_->set_metric(TransitionsBlackbox::METRIC_REWARD);
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphVI::GraphVI");
	throw;
}


bool GraphVI::policy_defined(const PddlState& st)
{
	return ((mdp_->explored(st)) && (mdp_->get_state(st).policy_defined()));
}


void GraphVI::solve_initialize(const PddlState& st)
{
	// generate all reachable states
	mdp_->add_state(st);
	GraphMDP<GraphStateWT<> >::new_states_list_t new_states;
	new_states.push_back(mdp_->_states_().find(&st));
	
	while (!new_states.empty())
	{
		mdp_->transitions(*(new_states.front()->second));
		new_states.pop_front();
		new_states.insert(new_states.end(), mdp_->new_states().begin(), mdp_->new_states().end());
	}
	
	record_values_.resize(mdp_->get_states().size(), 0.0);
}


void GraphVI::solve_progress()
{
	// record values
	std::list<double>::iterator Ivl = record_values_.begin();
	
	for (GraphMDP<GraphStateWT<> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
	{
		*Ivl = Ist->second->get_value();
		++Ivl;
	}
	
	// improve state values
	Ivl = record_values_.begin();
	bellman_error_ = 0.0;
	
	for (GraphMDP<GraphStateWT<> >::states_map_t::iterator Ist = mdp_->_states_().begin() ; Ist != mdp_->_states_().end() ; ++Ist)
	{
		improve_state_value(*(Ist->second));
		bellman_error_ = std::max(bellman_error_, std::fabs((Ist->second->get_value()) - (*Ivl)));
		++Ivl;
	}
}


bool GraphVI::has_converged()
{
	if (bellman_error_ < epsilon_)
		return true;
	else
		return false;
}
