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

#include "symbolic_bounds.h"
#include <mdpsim/states.h>
#include <mdpsim/actions.h>

#include "models/base_mdp.h"

/* 
 ******************************************************************************
 * Public Methods
 ******************************************************************************
 */

symbolic_bounds::symbolic_bounds(SymbolicMDP& m, double discount_factor) 
 : BaseHeuristic(discount_factor), SymbolicHeuristic(m, discount_factor)
{
  agraph = new symbolic_actiongraph();
  builder = new symbolic_fwdsearch();
}

symbolic_bounds::~symbolic_bounds(){
  if(agraph){
    delete agraph; delete builder;
    agraph = NULL; builder = NULL;
  }
}

const symbolic_actiongraph& symbolic_bounds::get_agraph() const {
  return (*agraph);
}

// Creates an ACTION GRAPH, in which each node is an action $a$ and a set $S_a$ of states
// such that the action $a$ is applicable in each state $s$ in $S_a$ and has the same one-step
// cost-to-go value in all of the states in $S_a$. An edge between two nodes of the graph exists,
// if there is a transtion induced by applying the action $a$ in a node in one of the states in $S_a$ and
// generating at least one state in the other node.
void symbolic_bounds::compute_single_state(const dd_node_ptr& state) {
  builder->initialize(&mdp_, *agraph, state);
  agraph = builder->generate_actiongraph();
}

void symbolic_bounds::compute_bounds() {
  // This is another search over the Action Graph. Do I need it? 
  // Couldn't I do this computation when I ran the SEARCHER?
  SYM_ACTION_NODELIST_TYPE current_nodes = agraph->get_endnodes();
  SYM_ACTION_NODELIST_TYPE preimage; 
  for(SYM_ACTION_NODELIST_TYPE::iterator n = current_nodes.begin(); n != current_nodes.end(); ++n) {
    const SYM_ACTION_NODESET_TYPE& pre1 = n->get_parents();
    for(SYM_ACTION_NODESET_TYPE::iterator n1 = pre1.begin(); n1 != pre1.end(); ++n1) {
      double candidate_lb = ((*n1)->get_onestep_lowerbound()) + (n->get_goalvalue_lowerbound());
      double candidate_ub = ((*n1)->get_onestep_upperbound()) + (n->get_goalvalue_upperbound());
      if (candidate_lb < ((*n1)->get_goalvalue_lowerbound()))
	(*n1)->set_goalvalue_lowerbound(candidate_lb);
      if (candidate_ub > ((*n1)->get_goalvalue_upperbound()))
	(*n1)->set_goalvalue_upperbound(candidate_ub);
    }
    for (SYM_ACTION_NODESET_TYPE::const_iterator n1 = pre1.begin(); n1 != pre1.end(); ++n1) {
    	preimage.push_back(**n1);
    }
    //preimage.insert(preimage.end(),pre1.begin(),pre1.end()); //my_append(preimage, pre1); // need to take care of duplicates properly.
  }
}

double symbolic_bounds::get_lowerbound_value (const Action& a) const {
  return (agraph->get_node(a))->get_goalvalue_lowerbound();
}
  
double symbolic_bounds::get_upperbound_value (const Action& a) const {
  return (agraph->get_node(a)->get_goalvalue_upperbound());
}

void symbolic_bounds::compute_distance() {
  SYM_ACTION_NODELIST_TYPE current_nodes = agraph->get_endnodes();
  SYM_ACTION_NODELIST_TYPE preimage; 
  for(SYM_ACTION_NODELIST_TYPE::iterator n = current_nodes.begin(); n != current_nodes.end(); ++n) {
    const SYM_ACTION_NODESET_TYPE& pre1 = n->get_parents();
    for(SYM_ACTION_NODESET_TYPE::const_iterator n1 = pre1.begin(); n1 != pre1.end(); ++n1) {
      int candidate_distance = 1 + (n->get_goaldistance());
      if (candidate_distance < ((*n1)->get_goaldistance()))
	(*n1)->set_goaldistance(candidate_distance);
    }
    for (SYM_ACTION_NODESET_TYPE::const_iterator n1 = pre1.begin(); n1 != pre1.end(); ++n1) {
    	preimage.push_back(**n1);
    }
    //preimage.insert(preimage.end(),pre1.begin(),pre1.end()); //my_append(preimage, pre1); // need to take care of duplicates properly.
  }
}

double symbolic_bounds::get_distance_value (const Action& a) const {
  return (agraph->get_node(a)->get_goaldistance());
}

void symbolic_bounds::compute(const dd_node_ptr& states)
{
	std::list<PddlState> pddl_states;
	mdp_.decompose(states, pddl_states);
	
	for (std::list<PddlState>::const_iterator Ist = pddl_states.begin() ; Ist != pddl_states.end() ; ++Ist)
		compute_single_state(mdp_.get_state(*Ist));
}


