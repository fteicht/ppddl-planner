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
#include <iostream>

#include "relaxed_planning_graph.h"
#include "commons/base_exceptions.h"

// CLASS RelaxedPlanningGraph

RelaxedPlanningGraph::RelaxedPlanningGraph(double discount_factor, const Problem& pb)
try : BaseHeuristic(discount_factor), problem_(pb)
{
	mdp_pg = new RelaxedMDPGraph(discount_factor, pb);
}
catch (BaseException& error)
{
	error.push_function_backtrace("RelaxedPlanningGraph::RelaxedPlanningGraph");
	throw;
}


RelaxedPlanningGraph::~RelaxedPlanningGraph() {
	if (mdp_pg)
		delete mdp_pg;
}

double RelaxedPlanningGraph::value(const PddlState& st)
{
	AtomSet ats = st.atoms();
	int level = mdp_pg->find_level(ats);
//	cout << "Level: " << level << endl;
	if (level == -1) {
		AtomSet previous;
		previous.clear();
//		cout << "Starting RPG Computation...." <<endl;
		while (! reached(ats,previous))
		{
/*
			std::cout << "Looping in the RBH..." << endl;
 				cout << "Starting to expand the following atom set: " << endl;
 	for (AtomSet::iterator itAt = ats.begin(); itAt != ats.end(); ++itAt)
 	   cout << "Atom: " << **itAt << endl;
*/			
			previous.clear();
			for(AtomSet::iterator itAS = ats.begin(); itAS != ats.end(); ++itAS)
			{
				previous.insert(*itAS); //pointer
			}
			ats = mdp_pg->expand(ats);
			/*
			int level = mdp_pg-> getCurrentLevel(); //find_level(ats);
			
			ats = mdp_pg->generate_atoms(level);
			*/
			/*
				for (AtomSet::iterator itAt = ats.begin(); itAt != ats.end(); ++itAt)
 	   cout << "Atom: " << **itAt << endl;
	*/
		}
		//cout << "Finishing RPG Computation...." <<endl;
		mdp_pg->computeRewards(mdp_pg->getCurrentLevel());
	}
	double rew =  mdp_pg->getStateValue(st);
//	std::cout << " Graph Size: " << mdp_pg->getCurrentLevel() << endl;
//    cout << "Heuristic Value of the state: " << rew << endl;
    return rew;
}
/*
double RelaxedPlanningGraph::value(const PddlState& st)
{
	AtomSet ats = st.atoms();
	AtomSet previous;
	previous.clear();
	cout << "Starting RPG Computation...." <<endl;
	while (! reached(ats,previous))
	{
		mdp_pg->expand(ats);
		int level = mdp_pg-> getCurrentLevel(); //find_level(ats);
		cout << "Number PG Levels: " << level << endl;
		
		previous.clear();
		for(AtomSet::iterator itAS = ats.begin(); itAS != ats.end(); ++itAS)
		{
			previous.insert(*itAS); //pointer
		}

		cout << "Generate next level atoms... " << endl;
		ats = mdp_pg->generate_atoms(level);
		cout << "Next level atoms are generated... " << endl;	
	}
	cout << "Finishing RPG Computation...." <<endl;
	mdp_pg->computeRewards(mdp_pg->getCurrentLevel());
	double rew =  mdp_pg->getStateValue(st);
    cout << "HEuristic Value of the state: " << rew << endl;
    getchar();
    return rew;
}
*/


bool RelaxedPlanningGraph::reached(const AtomSet& current, AtomSet& previous)
{
	AtomSet temp_atoms;
	std::set_difference(current.begin(), current.end(),
			previous.begin(), previous.end(),
			std::inserter(temp_atoms, temp_atoms.begin()));
	return (temp_atoms.empty());
}




// CLASS GraphRelaxedPlanningGraph

template <class Tstate>
GraphRelaxedPlanningGraph<Tstate>::GraphRelaxedPlanningGraph(GraphMDP<Tstate>& mdp, double discount_factor, const Problem& pb)
try : BaseHeuristic(discount_factor), GraphHeuristic<Tstate>(mdp, discount_factor), RelaxedPlanningGraph(discount_factor, pb)
{
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphRelaxedPlanningGraph::GraphRelaxedPlanningGraph");
	throw;
}


template <class Tstate>
void GraphRelaxedPlanningGraph<Tstate>::compute(Tstate& st)
{
	st.set_value(value(st.get_pddl_state()));
}


// Explicit template instantiations
INSTANTIATE_STATE_TEMPLATE(GraphRelaxedPlanningGraph)




// CLASS SymbolicRelaxedPlanningGraph

SymbolicRelaxedPlanningGraph::SymbolicRelaxedPlanningGraph(SymbolicMDP& mdp, double discount_factor, const Problem& pb)
try : BaseHeuristic(discount_factor), SymbolicHeuristic(mdp, discount_factor), RelaxedPlanningGraph(discount_factor, pb)
{
}
catch (BaseException& error)
{
	error.push_function_backtrace("SymbolicRelaxedPlanningGraph::SymbolicRelaxedPlanningGraph");
	throw;
}


void SymbolicRelaxedPlanningGraph::compute(const dd_node_ptr& states)
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
