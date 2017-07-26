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

#include "heuristics/graph_bounds.h"


template <class Tstate>
graph_bounds<Tstate>::graph_bounds(GraphMDP<Tstate>& m, double discount_factor, const Problem& p, const ActionSet& all_acts)
: BaseHeuristic(discount_factor), GraphHeuristic<Tstate>(m, discount_factor), prob(p), all_actions(all_acts) {
  agraph = new ActionGraph<Tstate>();
  builder = new graph_fwdsearch<Tstate>(&m,&prob);
  
}

template <class Tstate>
graph_bounds<Tstate>::~graph_bounds(){
/*
  if(agraph){
    delete agraph; delete builder;
    agraph = NULL; builder = NULL;
  }
  */
}

template <class Tstate>
ActionGraph<Tstate>* graph_bounds<Tstate>::get_agraph() {
  return agraph;
}

// Creates an ACTION GRAPH, in which each node is an action $a$ and a set $S_a$ of states
// such that the action $a$ is applicable in each state $s$ in $S_a$ and has the same one-step
// cost-to-go value in all of the states in $S_a$. An edge between two nodes of the graph exists,
// if there is a transtion induced by applying the action $a$ in a node in one of the states in $S_a$ and
// generating at least one state in the other node. 

// Returns the new ACTION GRAPH.

template <class Tstate>
void graph_bounds<Tstate>::initialize(Tstate* current_state) {
  builder->initialize(agraph, current_state);
  agraph = builder->generate_actiongraph();
}

template <class Tstate>
void graph_bounds<Tstate>::compute_bounds() {
  // Got the starting and finishing equivalence classes, so now do the search that 
  // generates the lower and upper bounds
  ACTION_NODESET_TYPE preimage; 
  ACTION_NODELIST_TYPE nlist = agraph->get_endnodes();
  for(typename ACTION_NODELIST_TYPE::iterator n = nlist.begin(); n != nlist.end(); ++n) {
      ActionNode<Tstate>* node = *n;
      const ACTION_NODESET_TYPE& pre1 = node->get_parents();
      for(typename ACTION_NODESET_TYPE::iterator n1 = pre1.begin(); n1 != pre1.end(); ++n1) {
        ActionNode<Tstate>* node1 = *n1;
        double candidate_lb = (node1->get_onestep_lowerbound()) + (node->get_goalvalue_lowerbound());
        double candidate_ub = (node1->get_onestep_upperbound()) + (node->get_goalvalue_upperbound());
        if (candidate_lb < (node1->get_goalvalue_lowerbound()))
    	  node1->set_goalvalue_lowerbound(candidate_lb);
        if (candidate_ub > (node1->get_goalvalue_upperbound()))
	  node1->set_goalvalue_upperbound(candidate_ub);
      }
      preimage.insert(pre1.begin(), pre1.end()); // need to take care of duplicates properly.
  }
}  

template <class Tstate>
double graph_bounds<Tstate>::get_lowerbound_value (const Action& a) const {
  return (agraph->get_node(a)->get_goalvalue_lowerbound());
}

template <class Tstate>
double graph_bounds<Tstate>::get_upperbound_value (const Action& a) const {
  return (agraph->get_node(a)->get_goalvalue_upperbound());
}

// These are the real interface functions.
template <class Tstate>
void graph_bounds<Tstate>::compute (Tstate& state) {
  std::set<const Action*> actions = get_actions_for (state);
//  if (actions.empty()) { // means we have not computed the heuristic for this state before.
    this->initialize(&state);
    this->compute_bounds();
    actions = get_actions_for (state);
//  }

  state.set_value(choose_best_value(actions));
}

template <class Tstate>
std::set<const Action*> graph_bounds<Tstate>::get_actions_for (Tstate& state) const {
  const PddlState& s = state.get_pddl_state();
  ActionList app_actions_list; 
  prob.enabled_actions(app_actions_list,s.atoms(),s.values());
  std::set<const Action*> applicable_actions;
  for (ActionList::const_iterator a = app_actions_list.begin(); a != app_actions_list.end(); ++a){
    const Action* aa = *a;
    applicable_actions.insert(aa);
  }
  return applicable_actions;
}
    

template <class Tstate>
double graph_bounds<Tstate>::choose_best_value(const std::set<const Action*>& actions) const {
  double max = 0.0;
  for (std::set<const Action*>::const_iterator a = actions.begin(); a != actions.end(); ++a) {
    const Action* ax = *a;
    ActionNode<Tstate>* node = agraph->get_node(*ax);
    if (node->get_goalvalue_upperbound() > max) {
      max = node->get_goalvalue_upperbound();
    }
  }
  return max;
}

template <class Tstate>
std::set<const Action*> graph_bounds<Tstate>::prune(Tstate& state) {
  std::set<const Action*> actions = get_actions_for (state);
  ACTION_NODELIST_TYPE suboptimals; 
  for (std::set<const Action*>::const_iterator a = actions.begin(); a != actions.end(); ++a) {
    const Action* ax = *a;
    ActionNode<Tstate>* node = agraph->get_node(*ax);
    if (!action_in(suboptimals, *node)) {
      for (std::set<const Action*>::const_iterator a1 = actions.begin(); a1 != actions.end(); ++a1) {
        const Action* ax1 = *a1;
	ActionNode<Tstate>* node1 = agraph->get_node(*ax1);
	if (((**a).name() != (**a1).name()) && !action_in(suboptimals, *node1)){
	  if ((node->get_goalvalue_upperbound()) < (node1->get_goalvalue_lowerbound()))
	    suboptimals.push_front(node1);
        }
      }
    }
  }

  for (std::set<const Action*>::const_iterator a = actions.begin(); a != actions.end(); ++a) {
    const Action* ax = *a;
    ActionNode<Tstate>* node = agraph->get_node(*ax);
    if (action_in(suboptimals,*node))
      actions.erase(a);
  }
  return actions;
}

template <class Tstate>
bool graph_bounds<Tstate>::action_in(const ACTION_NODELIST_TYPE& action_list, const ActionNode<Tstate>& act) const {
  for (typename ACTION_NODELIST_TYPE::const_iterator a = action_list.begin(); a != action_list.end(); ++a)
    if ((*a)->get_action().name() == act.get_action().name())
      return true;
  return false;
}
    
bool operator< (const Action& a, const Action& b) {
 return true;
}




// Explicit template instantiations
INSTANTIATE_STATE_TEMPLATE(graph_bounds)
