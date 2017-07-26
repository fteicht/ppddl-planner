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

#ifndef GRAPH_SEARCH_H_
#define GRAPH_SEARCH_H_

#include "commons/declarations.h"
#include "models/graph_mdp.h"
#include "heuristics/Action_Graph.h"

/* The Class Hierarchy below could be designed much better to be more general. 
 * I am implementing only what I need at the moment. [2008/04/04:ukuter]
 */
 
template<class Tstate>
class graph_fwdsearch {
public :
        typedef std::map<const PddlState*, Tstate*, PddlStateLess> states_map_t;
        typedef GraphTransition<Tstate> transition_t;
        typedef std::list<transition_t*> transition_list_t;
        typedef std::map<const Action*, transition_list_t> actions_map_t;
        typedef std::set<Tstate*> ss_states_set_t;
        typedef std::list<typename states_map_t::iterator> new_states_list_t;

 private:
  Tstate* init;
  const Problem* prob;
  GraphMDP<Tstate>* mdp;
  ActionGraph<Tstate>* agraph;
  int bound_type;
  
  inline static void _delete_transitions_(const actions_map_t* transitions);
  
 public:
  /* constructor(s) and destructor. */
  graph_fwdsearch(GraphMDP<Tstate>* m, const Problem* p);
  ~graph_fwdsearch();
  
  /* Methods */
  void set_boundtype(int btype);
  int get_boundtype();

  void initialize(ActionGraph<Tstate>* ag, Tstate* current_state);
  ActionGraph<Tstate>* generate_actiongraph();
  bool donep(const ss_states_set_t& fringe, const ss_states_set_t& prev_fringe); 
  double get_minreward(const transition_list_t& transs);
  double get_maxreward(const transition_list_t& transs);
};

/*
template<class Tstate>
class BwdSearch : PlanningSearch<Tstate> {
private:
ActionGraph<Tstate> agraph;
int bound_type;

bool initial_is_in(std::list<ActionNode<Tstate> > nodelist);

public:
// constructor(s) and destructor.
BwdSearch();
BwdSearch(ActionGraph<Tstate> ag);
~BwdSearch();

// Methods 
void set_boundtype(int btype);
int get_boundtype();

ActionGraph<Tstate> run();
};
*/


#endif /*GRAPH_SEARCH_H_*/
