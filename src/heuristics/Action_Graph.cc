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

#include "Action_Graph.h"
#include "models/graph_mdp.h"

// *****  ActionNode ******

// Constructors and destructor
template<class Tstate>
ActionNode<Tstate>::ActionNode() {
  act = NULL;
  onestep_lowerbound = onestep_upperbound = 0.0;
  goalvalue_lowerbound = goalvalue_upperbound = 0.0;
  goal_distance = 0;
  parents.clear();
  children.clear();
}

template<class Tstate>
ActionNode<Tstate>::ActionNode(const Action& a) {
  act = &a;
  onestep_lowerbound = onestep_upperbound = 0.0;
  goalvalue_lowerbound = goalvalue_upperbound = 0.0;
  goal_distance = 0;
  parents.clear();
  children.clear();
}

template<class Tstate>
ActionNode<Tstate>::ActionNode(const Action& a, double lb,
			       double ub) {
  act = &a;
  onestep_lowerbound = lb;
  onestep_upperbound = ub;
  goalvalue_lowerbound = goalvalue_upperbound = 0.0;
  goal_distance = 0;
  parents.clear();
  children.clear();
}

template<class Tstate>
ActionNode<Tstate>::~ActionNode() {
  //delete act;
  act = NULL;
  onestep_lowerbound = onestep_upperbound = 0.0;
  goalvalue_lowerbound = goalvalue_upperbound = 0.0;
  goal_distance = 0;
  parents.clear();
  children.clear(); 
}

// Methods
template<class Tstate>
const Action& ActionNode<Tstate>::get_action() const {
  return (*act);
}

/*
template<class Tstate>
EquivalenceClass<Tstate> ActionNode<Tstate>::get_context() {
  return context;
}

template<class Tstate>
void ActionNode<Tstate>::set_context(EquivalenceClass<Tstate> c) {
  context = c; //???
}
*/
template<class Tstate>
void ActionNode<Tstate>::add_parent(ActionNode<Tstate>& p) {
  parents.insert(&p);
}

template<class Tstate>
bool ActionNode<Tstate>::is_parent(ActionNode<Tstate>& p) const {
  if (parents.find(&p) == parents.end())
    return false;
  return true;
}

template<class Tstate>
//ACTION_NODELIST_TYPE ActionNode<Tstate>::get_parents() {
const std::set<ActionNode<Tstate>*>& ActionNode<Tstate>::get_parents() const {
  return parents;
}

template<class Tstate>
void ActionNode<Tstate>::add_child(ActionNode<Tstate>& p) {
  children.insert(&p);
}

template<class Tstate>
bool ActionNode<Tstate>::is_child(ActionNode<Tstate>& p) const {
  if (children.find(&p) == children.end())
    return false;
  return true;
}

template<class Tstate>
//ACTION_NODELIST_TYPE ActionNode<Tstate>::get_children() {
const std::set<ActionNode<Tstate>*>& ActionNode<Tstate>::get_children() const {
  return children;
}

template<class Tstate>
double ActionNode<Tstate>::get_onestep_lowerbound() const {
  return onestep_lowerbound;
}

template<class Tstate>
void ActionNode<Tstate>::set_onestep_lowerbound(double bound) {
  onestep_lowerbound = bound;
}

template<class Tstate>
double ActionNode<Tstate>::get_goalvalue_lowerbound() const {
  return goalvalue_lowerbound;
}

template<class Tstate>
void ActionNode<Tstate>::set_goalvalue_lowerbound(double bound) {
  goalvalue_lowerbound = bound;
}

template<class Tstate>
double ActionNode<Tstate>::get_onestep_upperbound() const {
  return onestep_upperbound;
}

template<class Tstate>
void ActionNode<Tstate>::set_onestep_upperbound(double bound) {
  onestep_upperbound = bound;
}

template<class Tstate>
double ActionNode<Tstate>::get_goalvalue_upperbound() const {
  return goalvalue_upperbound;
}

template<class Tstate>
void ActionNode<Tstate>::set_goalvalue_upperbound(double bound) {
  goalvalue_upperbound = bound;
}

template<class Tstate>
int ActionNode<Tstate>::get_goaldistance() const {
  return goal_distance;
}

template<class Tstate>
void ActionNode<Tstate>::set_goaldistance(int d) {
  goal_distance = d;
}

bool operator==(const ActionNode<GraphState>& a, const ActionNode<GraphState>& b) {
  return (a.get_action().name() == b.get_action().name());
}
bool operator==(const ActionNode<GraphStateWT<> >& a, const ActionNode<GraphStateWT<> >& b) {
  return (a.get_action().name() == b.get_action().name());
}
bool operator==(const ActionNode<GraphStateWA<> >& a, const ActionNode<GraphStateWA<> >& b) {
  return (a.get_action().name() == b.get_action().name());
}
bool operator==(const ActionNode<GraphStateWTA<> >& a, const ActionNode<GraphStateWTA<> >& b) {
  return (a.get_action().name() == b.get_action().name());
}

bool operator< (const ActionNode<GraphState>& a, const ActionNode<GraphState>& b) {
  return true;
}
bool operator< (const ActionNode<GraphStateWT<> >& a, const ActionNode<GraphStateWT<> >& b) {
  return true;
}
bool operator< (const ActionNode<GraphStateWA<> >& a, const ActionNode<GraphStateWA<> >& b) {
  return true;
}
bool operator< (const ActionNode<GraphStateWTA<> >& a, const ActionNode<GraphStateWTA<> >& b) {
  return true;
}





// Explicit template instantiations
INSTANTIATE_STATE_TEMPLATE(ActionNode)

// *****  ActionGraph ******
// Constructors and destructor
template<class Tstate>
ActionGraph<Tstate>::ActionGraph() {
}

template<class Tstate>
ActionGraph<Tstate>::ActionGraph(const ACTION_NODELIST_TYPE& nlist)
{
  for(typename ACTION_NODELIST_TYPE::const_iterator node=nlist.begin(); node != nlist.end(); ++node) {
    node_list.push_back(new ActionNode<Tstate>((*node)->get_action(), (*node)->get_onestep_lowerbound(), (*node)->get_onestep_upperbound()));
  }
}

template<class Tstate>
ActionGraph<Tstate>::~ActionGraph() {
//  for(typename ACTION_NODELIST_TYPE::iterator node=node_list.begin(); node != node_list.end(); ++node) {
//    delete (*node);
//  }
  node_list.clear();
}	

// Methods
template<class Tstate>
void ActionGraph<Tstate>::add_node(const Action& a) {
  ActionNode<Tstate>* newnode = new ActionNode<Tstate>(a);
  node_list.push_front(newnode);
}

template<class Tstate>
void ActionGraph<Tstate>::add_node(const Action& a, double lb, double ub) {
  ActionNode<Tstate>* newnode = new ActionNode<Tstate>(a, lb, ub);
  node_list.push_front(newnode);
}

template<class Tstate>
void ActionGraph<Tstate>::remove_node(const Action& a) {
  for(typename ACTION_NODELIST_TYPE::iterator node=node_list.begin(); node != node_list.end(); ++node) {
    ActionNode<Tstate>* n = *node;
    if (n->get_action().name() == a.name()) {
      delete (*node);
      node_list.erase(node);
      break;
    }
  }
}

template<class Tstate>
ActionNode<Tstate>* ActionGraph<Tstate>::get_node(const Action& a) {
  for(typename ACTION_NODELIST_TYPE::iterator node=node_list.begin(); 
      node != node_list.end(); ++node) {
    ActionNode<Tstate>* n = *node;
    if (n->get_action().name() == a.name()) {
      return n;
    }
  }
  //
//  std::cerr << "NULL" << std::endl;
  //
  return NULL;
}

template<class Tstate>
std::list<ActionNode<Tstate>*> ActionGraph<Tstate>::get_endnodes() {
  std::list<ActionNode<Tstate>*> newlist;

  for(typename ACTION_NODELIST_TYPE::iterator node=node_list.begin(); node != node_list.end(); ++node) {
    ActionNode<Tstate>* n = *node;
    if (n->get_children().empty())
  	newlist.push_back(n);
  }
  return newlist;
}
    

// Explicit template instantiations
INSTANTIATE_STATE_TEMPLATE(ActionGraph)
