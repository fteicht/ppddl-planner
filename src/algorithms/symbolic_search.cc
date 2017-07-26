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

#include "algorithms/symbolic_search.h"


symbolic_fwdsearch::symbolic_fwdsearch() {
  bound_type = BASE_LOWER;
  init = dd_node_ptr(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
  mdp = NULL;
  agraph = NULL;
}

symbolic_fwdsearch::~symbolic_fwdsearch() {
  bound_type = BASE_LOWER;
}

void symbolic_fwdsearch::initialize(SymbolicMDP* m, symbolic_actiongraph& ag, const dd_node_ptr& current_state){
  init.copy(current_state);
  agraph = &ag;
  mdp = m;
}

int symbolic_fwdsearch::get_boundtype() const {
  return bound_type;
}

void symbolic_fwdsearch::set_boundtype(int btype) {
  bound_type = btype;
}

// This is the main search algorithm. THIS FUNCTION IS DESTRUCTIVE!!!
// symbolic_actiongraph symbolic_fwdsearch::generate_actiongraph() {
//   delete agraph;
//   agraph = new symbolic_actiongraph();
//   dd_node_ptr fringe = init;
//   dd_node_ptr next_fringe = get_new_fringe();

//   while (!(reached(goal, fringe) || end_exploration(fringe))) {
//     SYM_ACTION_NODELIST_TYPE action_map = generate_applicable_actions(fringe);
//     for (typename SYM_ACTION_NODELIST_TYPE::const_iterator m = action_map.begin(); 
// 	 m != action_map.end(); ++m) {
//       Action a = m->get_action();
//       symbolic_actionnode* node = agraph.get_node(a);
//       if (node != NULL) {
// 	if ( (m->get_onestep_lowerbound()) < node.get_onestep_lowerbound())
// 	  node.set_onestep_lowerbound(m->get_onestep_lowerbound());
// 	if (node.get_onestep_upperbound() < (m->get_onestep_upperbound()))
// 	  node.set_onestep_upperbound(m->get_onestep_upperbound());
//       }
//       else {
// 	agraph.add_node(a, m->get_onestep_lowerbound(), m->get_onestep_upperbound());
//       }
//       dd_node_ptr action_fringe = generate_successors(fringe, a);
//       SYM_ACTION_NODELIST_TYPE child_actions = generate_applicable_actions(action_fringe);
//       for (typename SYM_ACTION_NODELIST_TYPE::const_iterator it = child_actions.begin(); 
// 	   it != child_actions.end(); ++it) {
// 	Action a1 = it->get_action();
// 	agraph.add_node(a1);
// 	symbolic_actionnode* node1 = agraph.get_node(a1);
// 	node1->add_parent(a);
// 	symbolic_actionnode* node = agraph.get_node(a);
// 	node->add_child(a1);
//       }
//       next_fringe = dd_node_ptr (Cudd_bddOr(dd_node_ptr::cudd_manager(),next_fringe.get(), action_fringe.get()));
//     }
//     fringe = next_fringe;
//   }
//   return agraph;
// }

symbolic_actiongraph* symbolic_fwdsearch::generate_actiongraph() {
  delete agraph;
  agraph = new symbolic_actiongraph();
  dd_node_ptr fringe;
  fringe.copy(init);
  
  SymbolicMDP::transitions_map_t prev_transitions;
  prev_transitions.copy(mdp->get_transitions());
  SymbolicMDP::transitions_map_t new_transitions; 
  while (! donep(fringe))  {
    new_transitions.clear();
    mdp->_transitions_().clear();
    dd_node_ptr action_fringe  = mdp->expand(fringe);

    for (SymbolicMDP::transitions_map_t::const_iterator it = mdp->get_transitions().begin();
	 it != (mdp->get_transitions()).end(); ++it) {
      const Action& a =*(it->first);
      SymbolicTransition st = it->second;
      symbolic_actionnode* node = agraph->get_node(a);
      dd_node_ptr current_states = dd_node_ptr (Cudd_bddAnd(dd_node_ptr::get_cudd_manager(),
							   fringe.get(),
							   st.get_policy_mask().get()));
      dd_node_ptr current_costs = dd_node_ptr (Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), current_states.get()));
      dd_node_ptr minus_infinity_dd(Cudd_ReadMinusInfinity(dd_node_ptr::get_cudd_manager()));
      dd_node_ptr plus_infinity_dd(Cudd_ReadPlusInfinity(dd_node_ptr::get_cudd_manager()));
      current_costs = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, 	
				  current_costs.get(), minus_infinity_dd.get()));
      dd_node_ptr min_cost_dd = dd_node_ptr (Cudd_addFindMin (dd_node_ptr::get_cudd_manager(), current_costs.get()));
      double min_cost = Cudd_V(min_cost_dd.get());
      current_costs = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, 	
				  current_states.get(), plus_infinity_dd.get()));
      dd_node_ptr max_cost_dd = dd_node_ptr (Cudd_addFindMax (dd_node_ptr::get_cudd_manager(), current_costs.get()));
      double max_cost = Cudd_V(max_cost_dd.get());
	
      if (node != NULL) {
	if (min_cost < node->get_onestep_lowerbound())
	  node->set_onestep_lowerbound(min_cost);
	if (node->get_onestep_upperbound() < max_cost)
	  node->set_onestep_upperbound(max_cost);
      }
      else {
	agraph->add_node(a, min_cost, max_cost);
      }

      dd_node_ptr next_states = mdp->expand(dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(),
								   fringe.get(), current_states.get())));
      new_transitions.insert((mdp->get_transitions()).begin(), (mdp->get_transitions()).end());
      ((SymbolicMDP::transitions_map_t) mdp->get_transitions()).clear();
      dd_node_ptr child_fringe  = mdp->expand(next_states);
      for (SymbolicMDP::transitions_map_t::const_iterator it = mdp->get_transitions().begin();
	   it != mdp->get_transitions().end(); ++it) {
	Action a1 = *(it->first);
	agraph->add_node(a1);
	symbolic_actionnode* node1 = agraph->get_node(a1);
	node1->add_parent(*node);
	symbolic_actionnode* node = agraph->get_node(a);
	node->add_child(*node1);
      }
      mdp->_transitions_().clear();
    }
    prev_transitions.insert(new_transitions.begin(), new_transitions.end());
    fringe = dd_node_ptr (Cudd_bddOr(dd_node_ptr::get_cudd_manager(), fringe.get(), action_fringe.get()));
  }
  mdp->_transitions_().clear();
  mdp->_transitions_().insert(prev_transitions.begin(), prev_transitions.end());

  return agraph;
}


/* ****************************************************************************** */
// Private Functions. 

bool symbolic_fwdsearch::donep (const dd_node_ptr& fringe) {
  dd_node_ptr fringe_compl = dd_node_ptr (Cudd_Not(fringe));
  dd_node_ptr intersection = dd_node_ptr (Cudd_bddAnd(dd_node_ptr::get_cudd_manager(),
						      fringe_compl.get(),
						      (mdp->get_goal_states()).get()));
  return (dd_node_ptr (Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager())) == intersection);

  dd_node_ptr intersection2 = dd_node_ptr (Cudd_bddAnd(dd_node_ptr::get_cudd_manager(),
						      fringe_compl.get(),
						      (mdp->get_states()).get()));
  dd_node_ptr states_compl = dd_node_ptr (Cudd_Not(mdp->get_states()));
  dd_node_ptr intersection3 = dd_node_ptr (Cudd_bddAnd(dd_node_ptr::get_cudd_manager(),
						      fringe.get(),
						      states_compl.get()));
  return (dd_node_ptr (Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager())) == intersection2) &&
         (dd_node_ptr (Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager())) == intersection3);
}

