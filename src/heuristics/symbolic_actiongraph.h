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

#ifndef SYMBOLICACTIONGRAPH_
#define SYMBOLICACTIONGRAPH_

#include "commons/declarations.h"
#include "mdpsim/states.h"
#include "mdpsim/actions.h"
#include "models/dd_node_ptr.h"

class symbolic_actionnode {
		
 public:
  typedef std::list<symbolic_actionnode> SYM_ACTION_NODELIST_TYPE;

 private:
  const Action* act; // PPDDL action
  double onestep_lowerbound, onestep_upperbound;
  double goalvalue_lowerbound, goalvalue_upperbound;
  int goal_distance;
  std::set<symbolic_actionnode*> parents; 
  std::set<symbolic_actionnode*> children; 
		
 public:
  // Constructors and the destructor
  symbolic_actionnode();
  ~symbolic_actionnode();
    	
  // Methods
  void initialize (const Action& a, double lb, double ub);
  const Action& get_action() const;
     	
  void add_parent (symbolic_actionnode& p);	
//  bool is_parent (symbolic_actionnode& p);
  const std::set<symbolic_actionnode*>& get_parents () const;
    	
  void add_child (symbolic_actionnode& p);	
//  bool is_child (symbolic_actionnode& p);
  const std::set<symbolic_actionnode*>& get_children () const;
    	
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

class symbolic_actiongraph {	
 private:
//  symbolic_actionnode::SYM_ACTION_NODELIST_TYPE 
	std::list<symbolic_actionnode> node_list;

 public:
  // Constructor & Destructor
  symbolic_actiongraph();
  symbolic_actiongraph(const symbolic_actionnode::SYM_ACTION_NODELIST_TYPE& nodelist);
  ~symbolic_actiongraph();
  
  // Methods
  void add_node(const Action& a);
  void add_node(const Action& a, double lb, double ub);
  void remove_node(const Action& a);
  symbolic_actionnode* get_node(const Action& a);
  symbolic_actionnode::SYM_ACTION_NODELIST_TYPE get_endnodes() const;

};


#endif /*SYMBOLICACTIONGRAPH_*/
