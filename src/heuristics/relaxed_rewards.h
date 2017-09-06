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

#ifndef RELAXED_REWARDS_H_
#define RELAXED_REWARDS_H_

#include <ppddl_planner_mdpsim/problems.h>
#include "base_heuristic.h"

class RelaxedRewards : public virtual BaseHeuristic
{
protected :
	const Problem& problem_;
	AtomSet true_predicates_;
	AtomSet false_predicates_; // actually, its complement
	ValueMap values_;
	ValueMap::iterator reward_fluent_;
	
	bool first_reward_;
	double min_level_reward_;
	double max_level_reward_;
	bool first_level_;
	double min_reward_;
	double max_reward_;
	
	bool update_rewards();
	void update_goal_reward();
	
	bool relaxed_hold(const StateFormula& formula, bool negate = false) const;
	void relaxed_effect(const Effect& effect, AtomSet& true_prd, AtomSet& false_prd);
	
public :
	RelaxedRewards(double discount_factor, const Problem& pb); // throws
	virtual ~RelaxedRewards() {}
	
	virtual double value(const PddlState& st);
	
	inline const Problem& get_problem() const {return problem_;}
	bool is_true(const Atom& atom) const {return (true_predicates_.find(&atom) != true_predicates_.end());}
	bool is_false(const Atom& atom) const {return (false_predicates_.find(&atom) == false_predicates_.end());}
	
	inline double min_reward() const {return min_reward_;}
	inline double max_reward() const {return max_reward_;}
};


template <class Tstate>
class GraphRelaxedRewards : public GraphHeuristic<Tstate>, public RelaxedRewards
{
public :
	GraphRelaxedRewards(GraphMDP<Tstate>& mdp, double discount_factor, const Problem& pb);
	virtual ~GraphRelaxedRewards() {}
	
	virtual void compute(Tstate& st);
	virtual double value(const PddlState& st) {return RelaxedRewards::value(st);}
};


class SymbolicRelaxedRewards : public SymbolicHeuristic, public RelaxedRewards
{
public :
	SymbolicRelaxedRewards(SymbolicMDP& mdp, double discount_factor, const Problem& pb);
	virtual ~SymbolicRelaxedRewards() {}
	
	virtual void compute(const dd_node_ptr& states);
	virtual double value(const PddlState& st) {return RelaxedRewards::value(st);}
};

#endif /*RELAXED_REWARDS_H_*/
