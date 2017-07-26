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

#include "heuristics/symbolic_actiongraph.h"
#include "mdpsim/types.h"

// *****  Symbolic Action Node ******
// Constructors and destructor
symbolic_actionnode::symbolic_actionnode() {
  act = NULL;
  onestep_lowerbound = onestep_upperbound = 0.0;
  goalvalue_lowerbound = goalvalue_upperbound = 0.0;
  goal_distance = 0;
}
/*
symbolic_actionnode::symbolic_actionnode(Action* a) {
  act = *a;
  onestep_lowerbound = onestep_upperbound = 0.0;
  goalvalue_lowerbound = goalvalue_upperbound = 0.0;
  goal_distance = 0;
}

symbolic_actionnode::symbolic_actionnode(Action* a, double lb,
					 double ub) {
  act = *a;
  onestep_lowerbound = lb;
  onestep_upperbound = ub;
  goalvalue_lowerbound = goalvalue_upperbound = 0.0;
  goal_distance = 0;
}
*/
symbolic_actionnode::~symbolic_actionnode() {
  onestep_lowerbound = onestep_upperbound = 0.0;
  goalvalue_lowerbound = goalvalue_upperbound = 0.0;
  goal_distance = 0;
}

// Methods
void symbolic_actionnode::initialize(const Action& a, double lb, double ub) {
  act = &a;
  onestep_lowerbound = lb;
  onestep_upperbound = ub;
}
  
const Action& symbolic_actionnode::get_action() const {
  return *act;
}

/*
dd_node_ptr symbolic_actionnode::get_context() {
  return context;
}

void symbolic_actionnode::set_context(Trepresentation& ctxt) {
  context = ctxt;
}
*/

double symbolic_actionnode::get_onestep_lowerbound() const {
  return onestep_lowerbound;
}

void symbolic_actionnode::set_onestep_lowerbound(double bound) {
  onestep_lowerbound = bound;
}

double symbolic_actionnode::get_goalvalue_lowerbound() const {
  return goalvalue_lowerbound;
}

void symbolic_actionnode::set_goalvalue_lowerbound(double bound) {
  goalvalue_lowerbound = bound;
}

double symbolic_actionnode::get_onestep_upperbound() const {
  return onestep_upperbound;
}

void symbolic_actionnode::set_onestep_upperbound(double bound) {
  onestep_upperbound = bound;
}

double symbolic_actionnode::get_goalvalue_upperbound() const {
  return goalvalue_upperbound;
}

void symbolic_actionnode::set_goalvalue_upperbound(double bound) {
  goalvalue_upperbound = bound;
}

int symbolic_actionnode::get_goaldistance() const {
  return goal_distance;
}

void symbolic_actionnode::set_goaldistance(int d) {
  goal_distance = d;
}

void symbolic_actionnode::add_parent(symbolic_actionnode& p) {
  parents.insert(&p);
}

/*
bool symbolic_actionnode::is_parent(symbolic_actionnode& p) {
  if (parents.find(p) == NULL)
    return false;
  return true;
}
*/

//SYM_ACTION_NODELIST_TYPE symbolic_actionnode::get_parents() {
const std::set<symbolic_actionnode*>& symbolic_actionnode::get_parents() const {
  return parents;
}

void symbolic_actionnode::add_child(symbolic_actionnode& ch) {
  children.insert(&ch);
}

/*
bool symbolic_actionnode::is_child(symbolic_actionnode& ch) {
  for(std::list<symbolic_actionnode>::const_iterator node=children.begin(); 
      node != node_list.end(); ++node) {
    symbolic_actionnode xx = *node;
    Action a1 = xx.get_action();
    if (a1.name() == a.name()) 
      return false;
  return true;
}
*/

//SYM_ACTION_NODELIST_TYPE symbolic_actionnode::get_children() {
const std::set<symbolic_actionnode*>& symbolic_actionnode::get_children() const {
	return children;
}

// *****  Symbolic Action Graph ******
// Constructors and destructor
symbolic_actiongraph::symbolic_actiongraph() {
  node_list.clear(); 
}

symbolic_actiongraph::symbolic_actiongraph(const std::list<symbolic_actionnode>& nlist)
: node_list(nlist) {
}

symbolic_actiongraph::~symbolic_actiongraph() {
    node_list.clear(); 
}	

// Methods
void symbolic_actiongraph::add_node(const Action& a) {
  node_list.push_front(symbolic_actionnode());
  node_list.front().initialize(a,0,0);
}

void symbolic_actiongraph::add_node(const Action& a, double lb, double ub) {
  node_list.push_front(symbolic_actionnode());
  node_list.front().initialize(a,lb,ub);
}

void symbolic_actiongraph::remove_node(const Action& a) {
  for(std::list<symbolic_actionnode>::iterator node=node_list.begin(); 
      node != node_list.end(); ++node) {
    symbolic_actionnode& xx = *node;
    const Action& a1 = xx.get_action();
    if (a1.name() == a.name()) {
      node_list.erase(node);
      break;
    }
  } 	
}

symbolic_actionnode* symbolic_actiongraph::get_node(const Action& a) {
  for(std::list<symbolic_actionnode>::iterator node=node_list.begin(); 
      node != node_list.end(); ++node) {
    const symbolic_actionnode& xx = *node;
    const Action& a1 = xx.get_action();
    if (a1.name() == a.name()) {
      return (&(*node));
    }
  }
  return NULL;
}
  
std::list<symbolic_actionnode> symbolic_actiongraph::get_endnodes() const {
  std::list<symbolic_actionnode> end_nodes; 
  for (std::list<symbolic_actionnode>::const_iterator n = node_list.begin();
       n != node_list.end(); ++n) {
    const symbolic_actionnode& xx = (*n);
    const std::set<symbolic_actionnode*>& c_list = xx.get_children();
    if (c_list.empty())
      end_nodes.push_front(*n);
  }
  return end_nodes;
}

