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

#ifndef ACTIONGRAPH_
#define ACTIONGRAPH_

#include <set>
#include <mdpsim/states.h>
#include <mdpsim/actions.h>
#include "models/graph_mdp.h"
#include "models/base_mdp.h"
//#include "Equivalence_Class.h"
#include "commons/declarations.h"

template <class Tstate>
class ActionNode {
 public:
  typedef std::list<ActionNode<Tstate>*> ACTION_NODELIST_TYPE;
  typedef std::set<ActionNode<Tstate>*> ACTION_NODESET_TYPE;
  
 private:
  const Action* act;
  double onestep_lowerbound, onestep_upperbound;
  double goalvalue_lowerbound, goalvalue_upperbound;
  int goal_distance;
  ACTION_NODESET_TYPE parents; 
  ACTION_NODESET_TYPE children; 
		
 public:
  // Constructors and the destructor
  ActionNode();
  ActionNode(const Action& a);
  ActionNode(const Action& a, double lb, double ub);
  ~ActionNode();
  
  // Methods
  const Action& get_action() const;
/*
  EquivalenceClass<Tstate> get_context();
  void set_context(EquivalenceClass<Tstate> ctxt);
*/
     
  void add_parent (ActionNode<Tstate>& p);	
  bool is_parent (ActionNode<Tstate>& p) const;
  const ACTION_NODESET_TYPE& get_parents () const;
    	
  void add_child (ActionNode<Tstate>& p);	
  bool is_child (ActionNode<Tstate>& p) const;
  const ACTION_NODESET_TYPE& get_children () const;
    	
  double get_onestep_lowerbound() const;
  void set_onestep_lowerbound(double bound);
  double get_goalvalue_lowerbound() const;
  void set_goalvalue_lowerbound(double bound);

  double get_onestep_upperbound() const;
  void set_onestep_upperbound(double bound);
  double get_goalvalue_upperbound() const;
  void set_goalvalue_upperbound(double bound);

  int get_goaldistance() const;
  void set_goaldistance(int d);


};

bool operator==(const ActionNode<GraphState>& a, const ActionNode<GraphState>& b);
bool operator==(const ActionNode<GraphStateWT<> >& a, const ActionNode<GraphStateWT<> >& b);
bool operator==(const ActionNode<GraphStateWA<> >& a, const ActionNode<GraphStateWA<> >& b);
bool operator==(const ActionNode<GraphStateWTA<> >& a, const ActionNode<GraphStateWTA<> >& b);
bool operator< (const ActionNode<GraphState>& a, const ActionNode<GraphState>& b);
bool operator< (const ActionNode<GraphStateWT<> >& a, const ActionNode<GraphStateWT<> >& b);
bool operator< (const ActionNode<GraphStateWA<> >& a, const ActionNode<GraphStateWA<> >& b);
bool operator< (const ActionNode<GraphStateWTA<> >& a, const ActionNode<GraphStateWTA<> >& b);

template<class Tstate>
class ActionGraph {
 public:
  typedef std::list<ActionNode<Tstate>*> ACTION_NODELIST_TYPE;
  typedef typename std::set<Tstate*> states_set_t;
  
 private:
  ACTION_NODELIST_TYPE node_list;

 public:
  // Constructor & Destructor
  ActionGraph();
  ActionGraph(const ACTION_NODELIST_TYPE& nodelist);
  ~ActionGraph();
  
  // Methods
  void add_node(const Action& a);
  void add_node(const Action& a, double lb, double ub);
  void remove_node(const Action& a);
  ActionNode<Tstate>* get_node(const Action& a);
  std::list<ActionNode<Tstate>*> get_endnodes();
		
};

#endif /*ACTIONGRAPH_*/
