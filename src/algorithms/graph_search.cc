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

#include "graph_search.h"

template <class Tstate>
graph_fwdsearch<Tstate>::graph_fwdsearch(GraphMDP<Tstate>* m, const Problem* p) : prob(p)
{
  bound_type = BASE_LOWER;  prob = p;
  init = NULL;
  mdp = m;
  agraph = new ActionGraph<Tstate>();
}

template <class Tstate>
graph_fwdsearch<Tstate>::~graph_fwdsearch() {
  bound_type = BASE_LOWER;
  if (init) { delete init; }
//  if (agraph) { delete agraph; }
}

template <class Tstate>	
void graph_fwdsearch<Tstate>::initialize(ActionGraph<Tstate>* ag, Tstate* current){
  init = current;
  agraph = ag;
}

template <class Tstate>	
int graph_fwdsearch<Tstate>::get_boundtype() {
  return bound_type;
}

template <class Tstate>	
void graph_fwdsearch<Tstate>::set_boundtype(int btype) {
  bound_type = btype;
}

// This is the main search algorithm.
template <class Tstate>
ActionGraph<Tstate>* graph_fwdsearch<Tstate>::generate_actiongraph() {
  delete agraph;
  agraph = new ActionGraph<Tstate>();
  ss_states_set_t fringe;
  fringe.insert(init);
  ss_states_set_t next_fringe;
  ss_states_set_t prev_fringe;

  while (!donep(fringe, prev_fringe)) {
    prev_fringe.clear();
    for (typename ss_states_set_t::const_iterator s = fringe.begin(); s != fringe.end(); ++s) {
      const actions_map_t* transitions = this->mdp->transitions(**s);	
      for (typename actions_map_t::const_iterator it = (*transitions).begin(); it != (*transitions).end(); ++it) {
		const Action& a = *(it->first);
        transition_list_t transs = it->second;
		ActionNode<Tstate>* node = agraph->get_node(a);
		if (node != NULL) {
	  		if ( this->get_minreward(transs) < node->get_onestep_lowerbound())
	    		node->set_onestep_lowerbound(this->get_minreward(transs));
	  		if (node->get_onestep_upperbound() < this->get_maxreward(transs))
	    		node->set_onestep_upperbound(this->get_maxreward(transs));
		}
		else {
		  	agraph->add_node(a, this->get_minreward(transs), this->get_maxreward(transs));
		}

		const new_states_list_t& action_fringe = mdp->new_states();
		for(typename new_states_list_t::const_iterator it1 = action_fringe.begin(); it1 != action_fringe.end(); ++it1) {
	  		Tstate* child_state = (*it1)->second;
	  		PddlState state = child_state->get_pddl_state();
	  		ActionList child_actions;
	  		prob->enabled_actions(child_actions, state.atoms(), state.values());
	   		for (ActionList::iterator it2 = (child_actions).begin(); it2 != (child_actions).end(); ++it2) {
	    		const Action& a1 = **(it2);//->first);
	    		agraph->add_node(a1);
	    		ActionNode<Tstate>* node1 = agraph->get_node(a1);
	    		ActionNode<Tstate>* newparent = new ActionNode<Tstate>(a);
	    		node1->add_parent(*newparent);
	    		ActionNode<Tstate>* node = agraph->get_node(a);
	    		ActionNode<Tstate>* newchild = new ActionNode<Tstate>(a1);
	    		node->add_child(*newchild);
	  		}
	  		fringe.insert(child_state);
		}
      }
      this->_delete_transitions_(transitions);
    }
    prev_fringe.insert(fringe.begin(), fringe.end());
  }
  return agraph;
}

// Private Functions
	
template <class Tstate>
bool graph_fwdsearch<Tstate>::donep(const ss_states_set_t& fringe, const ss_states_set_t& prev_fringe) {
  ss_states_set_t temp;
  std::set_difference(fringe.begin(), fringe.end(),
                      prev_fringe.begin(), prev_fringe.end(),
                      std::inserter(temp, temp.begin()));
  if (!(temp.empty())) return false;
  temp.clear();
  std::set_difference(prev_fringe.begin(), prev_fringe.end(),
                      fringe.begin(), fringe.end(),
                      std::inserter(temp, temp.begin()));
  if (!(temp.empty())) return false;
  
  return true;
}
    
template <class Tstate>
double graph_fwdsearch<Tstate>::get_minreward(const transition_list_t& transs) {
 double min = BASE_INFTY;
 for (typename transition_list_t::const_iterator it = transs.begin(); it != transs.end(); ++it) 
  if ((*it)->get_reward() < min)
    min = (*it)->get_reward();
 return min;
}

template <class Tstate>
double graph_fwdsearch<Tstate>::get_maxreward(const transition_list_t& transs) {
 double max = (0.0 - BASE_INFTY);
 for (typename transition_list_t::const_iterator it = transs.begin(); it != transs.end(); ++it) 
  if ((*it)->get_reward() > max)
    max = (*it)->get_reward();
 return max;
}


// Explicit template instantiations
INSTANTIATE_STATE_TEMPLATE(graph_fwdsearch)

#define INSTANTIATE_GRAPH_FWDSEARCH_DELETE_TRANSITIONS_GS(StateT) \
template <> \
void graph_fwdsearch<StateT>::_delete_transitions_(const actions_map_t* transitions) \
{ \
	delete transitions; \
}

INSTANTIATE_GRAPH_FWDSEARCH_DELETE_TRANSITIONS_GS(GraphState)
INSTANTIATE_GRAPH_FWDSEARCH_DELETE_TRANSITIONS_GS(GraphStateProbability)
INSTANTIATE_GRAPH_FWDSEARCH_DELETE_TRANSITIONS_GS(GraphStateVariance)
INSTANTIATE_GRAPH_FWDSEARCH_DELETE_TRANSITIONS_GS(GraphStateSolveBit)
INSTANTIATE_GRAPH_FWDSEARCH_DELETE_TRANSITIONS_GS(GraphStateConnection)
INSTANTIATE_GRAPH_FWDSEARCH_DELETE_TRANSITIONS_GS(GraphStateSolveBitConnection)


#define INSTANTIATE_GRAPH_FWDSEARCH_DELETE_TRANSITIONS_WT(StateT) \
template <> \
void graph_fwdsearch<GraphStateWT<StateT> >::_delete_transitions_(const actions_map_t* transitions) \
{ \
}

INSTANTIATE_GRAPH_FWDSEARCH_DELETE_TRANSITIONS_WT(GraphState)
INSTANTIATE_GRAPH_FWDSEARCH_DELETE_TRANSITIONS_WT(GraphStateProbability)
INSTANTIATE_GRAPH_FWDSEARCH_DELETE_TRANSITIONS_WT(GraphStateVariance)
INSTANTIATE_GRAPH_FWDSEARCH_DELETE_TRANSITIONS_WT(GraphStateSolveBit)
INSTANTIATE_GRAPH_FWDSEARCH_DELETE_TRANSITIONS_WT(GraphStateConnection)
INSTANTIATE_GRAPH_FWDSEARCH_DELETE_TRANSITIONS_WT(GraphStateSolveBitConnection)


#define INSTANTIATE_GRAPH_FWDSEARCH_DELETE_TRANSITIONS_WA(StateT) \
template <> \
void graph_fwdsearch<GraphStateWA<StateT> >::_delete_transitions_(const actions_map_t* transitions) \
{ \
	delete transitions; \
}

INSTANTIATE_GRAPH_FWDSEARCH_DELETE_TRANSITIONS_WA(GraphState)
INSTANTIATE_GRAPH_FWDSEARCH_DELETE_TRANSITIONS_WA(GraphStateProbability)
INSTANTIATE_GRAPH_FWDSEARCH_DELETE_TRANSITIONS_WA(GraphStateVariance)
INSTANTIATE_GRAPH_FWDSEARCH_DELETE_TRANSITIONS_WA(GraphStateSolveBit)
INSTANTIATE_GRAPH_FWDSEARCH_DELETE_TRANSITIONS_WA(GraphStateConnection)
INSTANTIATE_GRAPH_FWDSEARCH_DELETE_TRANSITIONS_WA(GraphStateSolveBitConnection)


#define INSTANTIATE_GRAPH_FWDSEARCH_DELETE_TRANSITIONS_WTA(StateT) \
template <> \
void graph_fwdsearch<GraphStateWTA<StateT> >::_delete_transitions_(const actions_map_t* transitions) \
{ \
}

INSTANTIATE_GRAPH_FWDSEARCH_DELETE_TRANSITIONS_WTA(GraphState)
INSTANTIATE_GRAPH_FWDSEARCH_DELETE_TRANSITIONS_WTA(GraphStateProbability)
INSTANTIATE_GRAPH_FWDSEARCH_DELETE_TRANSITIONS_WTA(GraphStateVariance)
INSTANTIATE_GRAPH_FWDSEARCH_DELETE_TRANSITIONS_WTA(GraphStateSolveBit)
INSTANTIATE_GRAPH_FWDSEARCH_DELETE_TRANSITIONS_WTA(GraphStateConnection)
INSTANTIATE_GRAPH_FWDSEARCH_DELETE_TRANSITIONS_WTA(GraphStateSolveBitConnection)
