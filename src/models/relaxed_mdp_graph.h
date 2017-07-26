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

#ifndef PLANNING_GRAPH_H_
#define PLANNING_GRAPH_H_

#include <set>
#include <list>
#include <mdpsim/formulas.h>
#include <mdpsim/actions.h>
#include <mdpsim/problems.h>
#include "models/transitions_blackbox.h"
#include "heuristics/base_heuristic.h"
#include "commons/declarations.h"

class RelaxedTransition
{
	private:
		std::set<const Atom*> triggers;
		const Action& act;
		std::set<const Atom*> outcomes;
		double reward;

	public:
		RelaxedTransition(const Action& a, double rew);
		~RelaxedTransition();

		double getReward();
		void setReward(double rew);
		const Action& getAction();
		bool isTriggeredBy(const Atom& a);
		std::set<const Atom*>& getTriggerCondition();
		std::set<const Atom*>& getOutcomes();
		void addTrigger (const Atom& at);
		void addOutcome (const Atom& at);
};

class TransitionLevel
{
	private:
		std::list<RelaxedTransition> transitions;
		int level_number;
		AtomSet atom_set;

	public:
		TransitionLevel(int level);
		~TransitionLevel();

		bool contains(const AtomSet& ats);
		int get_level_number() { return level_number; };
		void addTransition(const RelaxedTransition& tr);
		std::list<RelaxedTransition>& getTransitions();
		AtomSet& getAtoms() { return atom_set; };
		void setAtoms(const AtomSet& atoms);

};

class RelaxedMDPGraph
{
	private:
		const Problem& problem;
		std::list<TransitionLevel> levels;
		ValueMap values_;
		ValueMap::iterator reward_fluent_;
		double max_reward;
		std::set<double> transition_reward_set; // for internal use only
		double g_reward; // should be fixed throughout the problem
		double gamma; // should be fixed throughout the problem

		double getMinRewardOf(int level_num);
		double getMaxRewardOf(int level_num);
		bool relaxed_hold(const StateFormula& formula, const AtomSet& atoms);
		void extract_effect_information(const Effect& effect, const AtomSet& atoms, 
										AtomSet& adds, double probability);
		bool generate_triggers(const StateFormula& formula, AtomSet& triggerset);

	public:
		RelaxedMDPGraph(double discount_factor, const Problem& p);
		~RelaxedMDPGraph();

		// Returns the set of all trigger atoms in the transitions of a given LEVEL.
		AtomSet generate_atoms(int level);

		// Expands the planning graph with an additional pair of action and
		// proposition levels.
		AtomSet expand(const AtomSet& atoms);

		// Similar to EXPAND, but only adds the specified transition to the planning graph.
		void expandWithTransition(RelaxedTransition& a);

		// Returns the size of LEVELS.
		int getCurrentLevel();

		// Returns the graph level in which all of the atoms in ATS appear as TRIGGER
		// conditions of RelaxedTransitions.
		int find_level(const AtomSet& ats);

		// Computes the reward for each atom that appear in the planning graph, before the
		// specified level.
		void computeRewards(int level_num);

		// Returns the set of transitions at a specified LEVEL_NUM
		TransitionLevel& getTransitions(int level_num);

		// Returns the value of a STATE based in the rewards computed for each
		// atom that is true in that state.
		double getStateValue (const PddlState& state);

};

#endif /*PLANNING_GRAPH_H_*/
