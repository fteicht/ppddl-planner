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
#include <iostream>
#include <cstdio>
#include <typeinfo>

#include "relaxed_mdp_graph.h"

/*
 * Class: RelaxedTransition
*/
RelaxedTransition::RelaxedTransition(const Action &a, double rew) : act(a)
{
	triggers.clear();
	outcomes.clear();
	reward = rew;
}

RelaxedTransition::~RelaxedTransition()
{
	triggers.clear();
	outcomes.clear();
	reward = -1.0;
}

bool RelaxedTransition::isTriggeredBy(const Atom& a)
{
	std::set<const Atom*>::const_iterator it = triggers.find(&a);
	/*
	 * for(std::set<const Atom*>::const_iterator it = triggers.begin(); it != triggers.end(); ++it) {
		if ((**it).predicate() == a.predicate()) 
			return true;
	}
	*/
	bool ret = (it != triggers.end());
	return ret;
}

double RelaxedTransition::getReward()
{  return reward; }

void RelaxedTransition::setReward(double rew)
{
	reward = rew;
}

const Action& RelaxedTransition::getAction()
{ return act; }

std::set<const Atom*>& RelaxedTransition::getTriggerCondition()
{ return triggers; }

void RelaxedTransition::addTrigger (const Atom &at)
{
	triggers.insert(&at);
}
std::set<const Atom*>& RelaxedTransition::getOutcomes()
{ return outcomes; }

void RelaxedTransition::addOutcome (const Atom &at)
{
	outcomes.insert(&at);
}


/*
 * Class: TransitionLevel
 */
 TransitionLevel::TransitionLevel(int level)
 {
 	transitions.clear();
 	level_number = level;
 }

  TransitionLevel::~TransitionLevel()
 {
 	transitions.clear();
 	level_number = 0;
 }

 void TransitionLevel::addTransition(const RelaxedTransition& tr)
 {
 	transitions.push_front(tr);
 }

 std::list<RelaxedTransition>& TransitionLevel::getTransitions()
 {
 	return transitions;
 }

 bool TransitionLevel::contains(const AtomSet& ats)
 {
 	AtomSet temp_atoms;
	std::set_difference(ats.begin(), ats.end(),
					atom_set.begin(), atom_set.end(),
					std::inserter(temp_atoms, temp_atoms.begin()));
 	return (temp_atoms.empty());
 }
 /*
  bool TransitionLevel::contains(const AtomSet& ats)
 {
 	AtomSet all_trigger_atoms;
 	for(std::list<RelaxedTransition>::iterator itRT = transitions.begin(); itRT != transitions.end(); ++itRT) {
 		RelaxedTransition &trans = *itRT;
 		std::set<const Atom*>& trigs = trans.getTriggerCondition();
 		for (std::set<const Atom*>::const_iterator itA = trigs.begin(); itA != trigs.end(); ++itA)
 			all_trigger_atoms.insert(*itA); // pointer, but it should be unique
 	}		
/*
 	cout << "All Trigger Atoms Atoms: " << endl;
 	for (AtomSet::iterator itAt = all_trigger_atoms.begin(); itAt != all_trigger_atoms.end(); ++itAt)
 	   cout << "Atom: " << **itAt << endl;
*
 	AtomSet temp_atoms;
	std::set_difference(ats.begin(), ats.end(),
					all_trigger_atoms.begin(), all_trigger_atoms.end(),
					std::inserter(temp_atoms, temp_atoms.begin()));
 	return (temp_atoms.empty());
 }
 */
 void TransitionLevel::setAtoms(const AtomSet& atoms) {
 	for(AtomSet::const_iterator it = atoms.begin(); it != atoms.end(); ++it)
 		atom_set.insert(*it);
 }

/*
 * Class: Relaxed MDP Graph
 */

 RelaxedMDPGraph::RelaxedMDPGraph(double discount_factor, const Problem& p) : problem(p)
{
	levels.clear();
	gamma = discount_factor;
	const Function* reward_function = problem.domain().functions().find_function("reward");

	if (!reward_function)
		throw IncompatibilityException("domain contains no reward function");

	const Function* total_time_function = problem.domain().functions().find_function("total-time");

	if (total_time_function)
		values_.insert(std::make_pair(Fluent::fluent(*total_time_function), Rational(0)));

	const Function* goal_achieved_function = problem.domain().functions().find_function("goal-achieved");

	if (goal_achieved_function)
		values_.insert(std::make_pair(Fluent::fluent(*goal_achieved_function), Rational(0)));

	reward_fluent_= values_.insert(std::make_pair(Fluent::fluent(*reward_function), Rational(0))).first;
	problem.goal_reward()->affect(values_);
	g_reward = reward_fluent_->second.double_value();


}

 RelaxedMDPGraph::~RelaxedMDPGraph()
 {
 	levels.clear();
 }

// Adapted from Florent's function in RELAXED_REWARDS heuristic.
bool RelaxedMDPGraph::relaxed_hold(const StateFormula& formula, const AtomSet& atoms)
{
	if (formula.tautology())
		return true;

	if (formula.contradiction())
		return false;

	const Atom* atom_formula = dynamic_cast<const Atom*>(&formula);

	if (atom_formula)
		return (atoms.find(atom_formula) != atoms.end());

	const Negation* negation_formula = dynamic_cast<const Negation*>(&formula);

	if (negation_formula)
		return true; //(! (relaxed_hold(*negation_formula, atoms))); //atoms.find(atom_formula) == atoms.end());

	const Conjunction* conjunction_formula = dynamic_cast<const Conjunction*>(&formula);

	if (conjunction_formula)
	{
		for (FormulaList::const_iterator If = conjunction_formula->conjuncts().begin() ; If != conjunction_formula->conjuncts().end() ; ++If)
		{
			if (!relaxed_hold(**If, atoms))
				return false;
		}

		return true;
	}

	const Disjunction* disjunction_formula = dynamic_cast<const Disjunction*>(&formula);

	if (disjunction_formula)
	{
		for (FormulaList::const_iterator If = disjunction_formula->disjuncts().begin() ; If != disjunction_formula->disjuncts().end() ; ++If)
		{
			if (relaxed_hold(**If, atoms))
				return true;
		}

		return false;
	}

	// No other types of (instantiated) state formula exist
	throw BaseException(std::string("unexpected state formula ") + typeid(formula).name(), "RelaxedMDPGraph::relaxed_hold");
}

// Adapted from Florent's function in RELAXED_REWARDS heuristic.
void RelaxedMDPGraph::extract_effect_information(const Effect& effect, const AtomSet& atoms, AtomSet& adds,
												 double probability)
{
	if (effect.empty())
		return;

	const AddEffect* add_effect = dynamic_cast<const AddEffect*>(&effect);

	if (add_effect)
	{
		adds.insert(&(add_effect->atom()));
		(this->transition_reward_set).insert(probability);
		return;
	}

	const DeleteEffect* delete_effect = dynamic_cast<const DeleteEffect*>(&effect);

	if (delete_effect)
		return;

	const UpdateEffect* update_effect = dynamic_cast<const UpdateEffect*>(&effect);

	if (update_effect)
	{
		reward_fluent_->second = Rational(0);
		update_effect->update().affect(values_);
		double rew = reward_fluent_->second.double_value();
		(this->transition_reward_set).insert(probability * rew);
		//if (rew > this->max_reward)
		//	this->max_reward = rew;
		return;
	}

	const ConjunctiveEffect* conjunctive_effect = dynamic_cast<const ConjunctiveEffect*>(&effect);

	if (conjunctive_effect)
	{
		for (EffectList::const_iterator Ie = conjunctive_effect->conjuncts().begin() ;
			Ie != conjunctive_effect->conjuncts().end() ; ++Ie)
			extract_effect_information(**Ie, atoms, adds, probability);
		return;
	}

	const ConditionalEffect* conditional_effect = dynamic_cast<const ConditionalEffect*>(&effect);

	if (conditional_effect)
	{
		if (relaxed_hold(conditional_effect->condition(), atoms))
			extract_effect_information(conditional_effect->effect(), atoms, adds, probability);

		return;
	}

	const ProbabilisticEffect* probabilistic_effect = dynamic_cast<const ProbabilisticEffect*>(&effect);

	if (probabilistic_effect)
	{
		size_t nb_outcomes = probabilistic_effect->size();

		for (unsigned int cnt = 0 ; cnt < nb_outcomes ; cnt++) {
			Rational pr = probabilistic_effect->probability(cnt);
			extract_effect_information(probabilistic_effect->effect(cnt), atoms, adds, 
										probability * (pr.numerator() / (double) pr.denominator()));
		}

		return;
	}
	
	// No other types of (instantiated) effect exist
	throw BaseException(std::string("unexpected effect ") + typeid(effect).name(), "RelaxedRewards::relaxed_effect");
}

AtomSet RelaxedMDPGraph::expand(const AtomSet& atoms)
 {
	levels.push_back(TransitionLevel(levels.size()+1));
 	TransitionLevel& newlevel = levels.back();
 	AtomSet new_atoms;
    newlevel.setAtoms(atoms);
	   
 	  for (ActionSet::const_iterator it = problem.actions().begin(); it != problem.actions().end(); ++it)
 	  {
  
   		  const Action* a = (*it);
//		  cout << "Action: " << *a << endl;
//		  cout << "Checking Preconditions... " << endl;
 		  if (!relaxed_hold(a->precondition(), atoms)){
// 		  	cout << "Preconditions of the action " << *a << " does not hold ..." << endl;
 			continue;
 		  }
//		cout << "Applicable Action: " << *a << endl;
/*
	cout << "Starting to expand the following atom set with action: " << *a << endl;
 	for (AtomSet::iterator itAt = atoms.begin(); itAt != atoms.end(); ++itAt)
 	   cout << "Atom: " << **itAt << endl;
    getchar();
*/
	  	const Effect& eff = a->effect();
  	   	AtomSet addlist;
   	  		
   	  	this->max_reward = -1 * BASE_INFTY;
	  	extract_effect_information(eff, atoms, addlist, 1.0);
	  	for(std::set<double>::iterator itR = (this->transition_reward_set).begin();
	  		itR != (this->transition_reward_set).end(); ++itR) 
	  		{
	  			if (*itR > (this->max_reward))
	  				this->max_reward = *itR;
	  		}
	  		
	 	for (AtomSet::iterator itAt = atoms.begin(); itAt != atoms.end(); ++itAt)
 	  		addlist.insert(*itAt);
	  	
	  	//cout << "Transition reward: " << this->max_reward << endl;
	  	newlevel.addTransition(RelaxedTransition(*a, this->max_reward));
	  	RelaxedTransition& tr = newlevel.getTransitions().back();

 	  	// ignore delete list and the updates since we are working with a  relaxed planning graph.
 	  	for (AtomSet::iterator itAt = addlist.begin(); itAt != addlist.end(); ++itAt) {
 	  		//cout << "Outcome: " << **itAt << endl;
 	  		tr.addOutcome(**itAt);
 	  		new_atoms.insert(*itAt);
 	  	}
		
		//getchar();
		
		AtomSet triggerlist;
		bool flag = generate_triggers(a->precondition(), triggerlist);
		if (flag) {
 			for(AtomSet::iterator itF = triggerlist.begin(); itF != triggerlist.end(); ++itF){
 				tr.addTrigger(**itF);
 			}
		}
		else {
			std::cout << "Could not generate triggers" << std::endl;
			newlevel.getTransitions().pop_back();
		}
 	  }
 	  	//cout << "Finishing expand ..." << endl;
 	  	return new_atoms;
 }

// Based on Florent's RELAXED_HOLD function.
bool RelaxedMDPGraph::generate_triggers (const StateFormula& formula, AtomSet& triggerset)
{
	if (formula.tautology())
		return true;

	if (formula.contradiction())
		return false;

	const Atom* atom_formula = dynamic_cast<const Atom*>(&formula);
	if (atom_formula)
	{
		triggerset.insert(atom_formula);
		return true;
	}
	const Negation* negation_formula = dynamic_cast<const Negation*>(&formula);

	if (negation_formula)
		return true; //(! (relaxed_hold(*negation_formula, atoms))); //atoms.find(atom_formula) == atoms.end());

	const Conjunction* conjunction_formula = dynamic_cast<const Conjunction*>(&formula);
	if (conjunction_formula) {
		for (FormulaList::const_iterator If = conjunction_formula->conjuncts().begin() ; If != conjunction_formula->conjuncts().end() ; ++If)
			if (!(generate_triggers (**If, triggerset)))
				return false;
		return true;
	}

	const Disjunction* disjunction_formula = dynamic_cast<const Disjunction*>(&formula);
	if (disjunction_formula) {
		for (FormulaList::const_iterator If = disjunction_formula->disjuncts().begin() ; If != disjunction_formula->disjuncts().end() ; ++If)
			if (!(generate_triggers(**If, triggerset)))
				return false;
		return true;
	}
	// No other types of (instantiated) state formula exist
	throw BaseException(std::string("unexp8ected state formula ") + typeid(formula).name(), "RelaxeMDPGraph::generate_triggers");
}

void RelaxedMDPGraph::expandWithTransition(RelaxedTransition& tr)
{
		levels.push_back(TransitionLevel(levels.size()+1));
		TransitionLevel& newlevel = levels.back();
		newlevel.addTransition(tr);
}

int RelaxedMDPGraph::getCurrentLevel()
{
		return levels.size();
}

TransitionLevel& RelaxedMDPGraph::getTransitions(int level_num)
{
	int i =1;
	for (std::list<TransitionLevel>::iterator itL = levels.begin(); itL != levels.end(); ++itL)
	{
		if (i == level_num) 
		{ 
			return (*itL);
		}
		++i;
	}
	throw BaseException("Input level number is not defined in the Relaxed MDP Graph...", "RelaxedMDPGraph::getTransitions");
}

void RelaxedMDPGraph::computeRewards(int level_num)
{

	TransitionLevel& last_level = levels.back();
	AtomSet& final_atoms = last_level.getAtoms();
	if(!relaxed_hold(problem.goal(),final_atoms))
		this->g_reward = 0.0;
	
	
 for (; level_num > 0; --level_num) 
 {	
 	//cout << " Level num: " << level_num << endl; getchar();
	TransitionLevel& transitions = this->getTransitions(level_num);
	std::list<RelaxedTransition>& trs = transitions.getTransitions();
	for(std::list<RelaxedTransition>::iterator itT = trs.begin(); itT != trs.end(); ++ itT)
	{
		RelaxedTransition& t = *itT;
		if (level_num == (int) levels.size())
		{
			t.setReward(t.getReward() + (gamma * g_reward));
		}
		else
		{
			double reward = -1 * BASE_INFTY;
			std::set<const Atom*>& outcomes = t.getOutcomes();
			//cout<<"Action " << t.getAction() << "'s outcomes: " << outcomes.empty() << endl;
			for(std::set<const Atom*>::iterator itA = outcomes.begin(); itA != outcomes.end(); ++ itA)
			{

				TransitionLevel& nexttransitions = this->getTransitions(level_num + 1);
				std::list<RelaxedTransition>& trs2 = nexttransitions.getTransitions();
				for(std::list<RelaxedTransition>::iterator itT2 = trs2.begin(); itT2 != trs2.end(); ++ itT2)
				{
					RelaxedTransition& t2 = *itT2;
					//cout << "Triggered? : " << t2.isTriggeredBy(**itA) <<endl;
					//cout << "T2 Reward: " << t2.getReward() <<endl;
					if (t2.isTriggeredBy(**itA) && (reward < t2.getReward()))
						reward = t2.getReward();
				}
			}
			t.setReward(t.getReward() + ((this->gamma) * reward));	
		}
		//cout << " Transition reward: " << t.getReward() << endl; getchar();
	}
 }
}

int RelaxedMDPGraph::find_level(const AtomSet& ats){
	int i = 1;
	for(std::list<TransitionLevel>::iterator itTL = levels.begin(); itTL != levels.end(); ++itTL) {
		if(itTL->contains(ats))
			return i; //-1 * 
		++i; //(itTL->get_level_number());
	}
 	return -1;
 }

double RelaxedMDPGraph::getStateValue (const PddlState& state)
{
	const AtomSet& ats = state.atoms();
/*
	cout << "State Atoms: " << endl;
 	for (AtomSet::iterator itAt = ats.begin(); itAt != ats.end(); ++itAt)
 	   cout << "Atom: " << **itAt << endl;
*/ 	   
	int level = this->find_level(ats);
	//cout << "Level: " << level << endl; //getchar();
	
	TransitionLevel& next_transitions = this->getTransitions(level);
	std::list<RelaxedTransition>& trs = next_transitions.getTransitions();
	double reward = -1 * BASE_INFTY;
	//cout << "Initial reward: " << reward << endl;
	for(AtomSet::iterator itAS = ats.begin(); itAS != ats.end(); ++itAS)
	{
		for(std::list<RelaxedTransition>::iterator itT = trs.begin(); itT != trs.end(); ++ itT)
		{
			RelaxedTransition& t = *itT;			
			if (t.isTriggeredBy(**itAS) && (reward < t.getReward())) {
				reward = t.getReward();
			}
		}
	}
/*
 	reward_fluent_->second = Rational(0);
	problem.goal_reward()->affect(values_);
	double rwd = reward_fluent_->second.double_value();
	//cout << "Final reward: " << reward << endl;
	return (rwd + reward);
	*/
	return reward;
}


/*
 * PRIVATE FUNCTIONS
 */

/*
AtomSet RelaxedMDPGraph::generate_atoms(int level)
{
	AtomSet ats;
	TransitionLevel& transitions = this->getTransitions(level);
	std::list<RelaxedTransition>& trs = transitions.getTransitions();
	for(std::list<RelaxedTransition>::iterator itT = trs.begin(); itT != trs.end(); ++ itT)
	{
		RelaxedTransition& t = *itT;
		std::set<const Atom*>& triggers = t.getTriggerCondition();
		for(std::set<const Atom*>::iterator itA = triggers.begin(); itA != triggers.end(); ++itA)
			ats.insert(*itA);
	}
	return ats;
}
*/
AtomSet RelaxedMDPGraph::generate_atoms(int level)
{
	AtomSet ats;
	TransitionLevel& transitions = this->getTransitions(level);
	std::list<RelaxedTransition>& trs = transitions.getTransitions();
	for(std::list<RelaxedTransition>::iterator itT = trs.begin(); itT != trs.end(); ++ itT)
	{
		RelaxedTransition& t = *itT;
		std::set<const Atom*>& outcomes = t.getOutcomes();
		for(std::set<const Atom*>::iterator itA = outcomes.begin(); itA != outcomes.end(); ++itA)
			ats.insert(*itA);
	}
	return ats;
}
double RelaxedMDPGraph::getMinRewardOf(int level_num)
{
	double reward = BASE_INFTY;
	TransitionLevel& transitions = this->getTransitions(level_num);
	std::list<RelaxedTransition>& trs = transitions.getTransitions();
	for(std::list<RelaxedTransition>::iterator itT = trs.begin(); itT != trs.end(); ++ itT)
	{
		RelaxedTransition& t = *itT;
		if (level_num == (int) levels.size())
			return 0.0;
		else if (reward > t.getReward())
			reward = t.getReward();
	}
	return reward;
}

double RelaxedMDPGraph::getMaxRewardOf(int level_num)
{
	double reward = BASE_MINUS_INFTY;
	TransitionLevel& transitions = this->getTransitions(level_num);
	std::list<RelaxedTransition>& trs = transitions.getTransitions();
	for(std::list<RelaxedTransition>::iterator itT = trs.begin(); itT != trs.end(); ++ itT)
	{
		RelaxedTransition& t = *itT;
		if (level_num == (int) levels.size())
			return 0.0;
		else if (reward < t.getReward())
			reward = t.getReward();
	}
	return reward;
}

/*

*/
