/*
   ppddl-planner - client for IPPC'08

   Copyright (C) 2011 Florent Teichteil-Koenigsbuch and Guillaume Infantes and Ugur Kuter

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

#include "hdet.h"
#include "models/base_determinization.h"
#include "planners/deterministic_planner.h"
#include "commons/base_exceptions.h"

#include <limits>
#include <cmath>

// CLASS HeuristicHdet

HeuristicHdet::HeuristicHdet(const Problem& pb, double discount_factor, unsigned int plan_length, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type)
try : BaseHeuristic(discount_factor), DeterminizationHeuristic(pb, discount_factor, plan_length, determinization_type, deterministic_planner_type)
{
}
catch (BaseException & error)
{
	error.push_function_backtrace("HeuristicHdet::HeuristicHdet");
	throw;
}


double HeuristicHdet::value(const PddlState& st)
{
	if (problem_.goal().holds(problem_.terms(), st.atoms(), st.values()))
		return 0.0;

	determinization_->make_problem(st.atoms(), st.values(), problem_.goal());
	DeterministicPlanner::plan_t plan;
	deterministic_planner_->solve(plan);
	std::size_t plan_length = plan.size();

	if (discount_factor_ < 1.0)
	{
		if (plan_length == 0) // dead-end state
			return (-1.0 / (1.0 - discount_factor_));
		else
			return (plan_length == 1)?(0.0):((1.0 - std::pow(discount_factor_, plan_length - 1))/(discount_factor_ - 1.0));
	}
	else
	{
		if (plan_length == 0) // dead-end state
			return -std::numeric_limits<double>::max();
		else
			return (plan_length == 1)?(0.0):(1 - plan_length);
	}
}




// CLASS GraphHeuristicHdet

template <class Tstate>
GraphHeuristicHdet<Tstate>::GraphHeuristicHdet(GraphMDP<Tstate>& mdp, const Problem& pb, double discount_factor, unsigned int plan_length, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type)
try : BaseHeuristic(discount_factor), GraphHeuristic<Tstate>(mdp, discount_factor), HeuristicHdet(pb, discount_factor, plan_length, determinization_type, deterministic_planner_type)
{
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphHeuristicHdet::GraphHeuristicHdet");
	throw;
}


template <class Tstate>
void GraphHeuristicHdet<Tstate>::compute(Tstate& st)
{
	st.set_value(value(st.get_pddl_state()));
}


// Explicit template instantiations
INSTANTIATE_STATE_TEMPLATE(GraphHeuristicHdet)




// CLASS SymbolicHeuristicHdet

SymbolicHeuristicHdet::SymbolicHeuristicHdet(SymbolicMDP& mdp, const Problem& pb, double discount_factor, unsigned int plan_length, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type)
try : BaseHeuristic(discount_factor), SymbolicHeuristic(mdp, discount_factor), HeuristicHdet(pb, discount_factor, plan_length, determinization_type, deterministic_planner_type)
{
}
catch (BaseException& error)
{
	error.push_function_backtrace("SymbolicHeuristicHdet::SymbolicHeuristicHdet");
	throw;
}


void SymbolicHeuristicHdet::compute(const dd_node_ptr& states)
{
	dd_node_ptr heuristic_values(Cudd_ReadZero(dd_node_ptr::get_cudd_manager()));

	std::list<PddlState> pddl_states;
	mdp_.decompose(states, pddl_states);

	for (std::list<PddlState>::const_iterator Ist = pddl_states.begin() ; Ist != pddl_states.end() ; ++Ist)
	{
		dd_node_ptr valdd(Cudd_addConst(dd_node_ptr::get_cudd_manager(), value(*Ist)));
		dd_node_ptr cudd_state = mdp_.get_state(*Ist);
		cudd_state = dd_node_ptr(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), cudd_state.get()));
		cudd_state = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, cudd_state.get(), valdd.get()));
		heuristic_values = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addPlus, heuristic_values.get(), cudd_state.get()));
	}

	dd_node_ptr states_cmpl = dd_node_ptr(Cudd_Not(states.get()));
	states_cmpl = dd_node_ptr(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), states_cmpl.get()));
	mdp_._value_function_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, mdp_.get_value_function().get(), states_cmpl.get()));
	mdp_._value_function_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addPlus, mdp_.get_value_function().get(), heuristic_values.get()));
}
