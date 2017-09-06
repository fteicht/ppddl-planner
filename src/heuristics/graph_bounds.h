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
#include <ppddl_planner_mdpsim/states.h>
#include <ppddl_planner_mdpsim/actions.h>
#include <ppddl_planner_mdpsim/problems.h>

#include "commons/declarations.h"
#include "algorithms/heuristic_algorithm.h"
#include "models/graph_mdp.h"
#include "Action_Graph.h"
#include "algorithms/graph_search.h"
#include "heuristics/base_heuristic.h"

template<class Tstate>
class graph_bounds : public GraphHeuristic<Tstate> {
public :
        typedef std::map<const PddlState*, Tstate*, PddlStateLess> states_map_t;
        typedef GraphTransition<Tstate> transition_t;
        typedef std::list<transition_t*> transition_list_t;
        typedef std::map<const Action*, transition_list_t> actions_map_t;
        typedef std::set<const Tstate*> states_set_t;
        typedef std::list<typename states_map_t::iterator> new_states_list_t;
	typedef std::list<ActionNode<Tstate>*> ACTION_NODELIST_TYPE;
	typedef std::set<ActionNode<Tstate>*> ACTION_NODESET_TYPE;
	typedef std::set<const Action*> ACTION_SET_TYPE;


 private :
  const Problem& prob;
  const ActionSet& all_actions;
  ActionGraph<Tstate>* agraph;
  graph_fwdsearch<Tstate>* builder;

  std::set<const Action*> get_actions_for (Tstate& state) const;
  bool action_in(const ACTION_NODELIST_TYPE& action_list, const ActionNode<Tstate>& act) const;
  double choose_best_value(const std::set<const Action*>& actions) const;

 public:
  /* Constructs the heuristic. */
  graph_bounds(GraphMDP<Tstate>& m, double discount_factor, const Problem& p, const ActionSet& all_acts);
  
  /* Deletes this heuristic. */
  virtual ~graph_bounds();
		
  /* Methods */
		
  // Returns the current action graph.
  ActionGraph<Tstate>* get_agraph();
  
  // Populates the action graph with the input MDP.
  void initialize(Tstate* current_state);
		
  // Compute the lower and upper bounds.
  void compute_bounds();
  
  // Retrival functions. Must be called after COMPUTE_BOUNDS.
  double get_lowerbound_value (const Action& a) const;
  double get_upperbound_value (const Action& a) const;

  // These are the real interface functions
  virtual void compute (Tstate& state);
  std::set<const Action*> prune(Tstate& state);
	
};

bool operator< (const Action& a, const Action& b);


