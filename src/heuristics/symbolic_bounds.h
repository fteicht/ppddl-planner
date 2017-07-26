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

#include "commons/declarations.h"
#include "models/symbolic_mdp.h"
#include "models/dd_node_ptr.h"
#include "algorithms/symbolic_search.h"
#include "heuristics/base_heuristic.h"

class symbolic_bounds : public SymbolicHeuristic {
 public:
  typedef std::list<symbolic_actionnode> SYM_ACTION_NODELIST_TYPE;
  typedef std::set<symbolic_actionnode*> SYM_ACTION_NODESET_TYPE;

 private:
  symbolic_fwdsearch* builder;
  symbolic_actiongraph* agraph;
  
 public :
  /* Constructs the heuristic. */
  symbolic_bounds(SymbolicMDP& m, double discount_factor);
  
  /* Deletes this heuristic. */
  virtual ~symbolic_bounds();
  
  /* 
   * Methods 
   */
  
  void compute_single_state(const dd_node_ptr& state);

  // Returns the current action graph.
  const symbolic_actiongraph& get_agraph() const;
		
  // Compute the lower and upper bounds.
  void compute_bounds();

  // Retrival functions. Must be called after COMPUTE_BOUNDS.
  double get_lowerbound_value (const Action& a) const;
  double get_upperbound_value (const Action& a) const;

  // Compute the distance heuristic.
  void compute_distance();
  // Retrival function. Must be called after COMPUTE_DISTANCE.
  double get_distance_value (const Action& a) const;
  
  // These are the real interface functions
  virtual void compute (const dd_node_ptr& state);
  SYM_ACTION_NODELIST_TYPE prune(const dd_node_ptr& state);
};

