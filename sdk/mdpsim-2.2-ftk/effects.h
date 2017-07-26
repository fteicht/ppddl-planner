/* -*-C++-*- */
/*
 * Effects.
 *
 * Copyright 2003-2005 Carnegie Mellon University and Rutgers University
 * Copyright 2007 Håkan Younes
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef EFFECTS_H
#define EFFECTS_H

#include <config.h>
#include "formulas.h"
#include "expressions.h"
#include "refcount.h"
#include "terms.h"
#include "rational.h"
#include <iostream>
#include <utility>
#include <vector>
#include <list>


/* ====================================================================== */
/* Update */

/*
 * An update.
 */
struct Update {
  /* Deletes this update. */
  virtual ~Update();

  /* Returns the fluent of this update. */
  const Fluent& fluent() const { return *fluent_; }

  /* Returns the expression of this update. */
  const Expression& expression() const { return *expr_; }

  /* Changes the given state according to this update. */
  virtual void affect(ValueMap& values) const = 0;

  /* Returns an instantiaion of this update. */
  virtual const Update& instantiation(const SubstitutionMap& subst,
                                      const ValueMap& values) const = 0;

  /* Returns the reward update or 0 if not a reward update or if state-dependent update */
  virtual double reward(const ValueMap& static_fluents_values) const = 0;

 protected:
  /* Constructs an update. */
  Update(const Fluent& fluent, const Expression& expr);

  /* Prints this object on the given stream. */
  virtual void print(std::ostream& os) const = 0;

 private:
  /* Fluent affected by this update. */
  const Fluent* fluent_;
  /* Expression. */
  const Expression* expr_;

  friend std::ostream& operator<<(std::ostream& os, const Update& u);
};

/* Output operator for updates. */
std::ostream& operator<<(std::ostream& os, const Update& u);


/* ====================================================================== */
/* UpdateList */

/*
 * List of updates.
 */
struct UpdateList : public std::vector<const Update*> {
};


/* ====================================================================== */
/* Assign */

/*
 * An assign update.
 */
struct Assign : public Update {
  /* Constructs an assign update. */
  Assign(const Fluent& fluent, const Expression& expr)
    : Update(fluent, expr) {}

  /* Changes the given state according to this update. */
  virtual void affect(ValueMap& values) const;

  /* Returns an instantiaion of this update. */
  virtual const Update& instantiation(const SubstitutionMap& subst,
                                      const ValueMap& values) const;

  /* Returns the reward update or 0 if not a reward update or if state-dependent update */
  virtual double reward(const ValueMap& static_fluents_values) const;

 protected:
  /* Prints this object on the given stream. */
  virtual void print(std::ostream& os) const;
};


/* ====================================================================== */
/* ScaleUp */

/*
 * An scale-up update.
 */
struct ScaleUp : public Update {
  /* Constructs an scale-up update. */
  ScaleUp(const Fluent& fluent, const Expression& expr)
    : Update(fluent, expr) {}

  /* Changes the given state according to this update. */
  virtual void affect(ValueMap& values) const;

  /* Returns an instantiaion of this update. */
  virtual const Update& instantiation(const SubstitutionMap& subst,
                                      const ValueMap& values) const;

  /* Returns the reward update or 0 if not a reward update or if state-dependent update */
  virtual double reward(const ValueMap& static_fluents_values) const;

 protected:
  /* Prints this object on the given stream. */
  virtual void print(std::ostream& os) const;
};


/* ====================================================================== */
/* ScaleDown */

/*
 * An scale-down update.
 */
struct ScaleDown : public Update {
  /* Constructs an scale-down update. */
  ScaleDown(const Fluent& fluent, const Expression& expr)
    : Update(fluent, expr) {}

  /* Changes the given state according to this update. */
  virtual void affect(ValueMap& values) const;

  /* Returns an instantiaion of this update. */
  virtual const Update& instantiation(const SubstitutionMap& subst,
                                      const ValueMap& values) const;

  /* Returns the reward update or 0 if not a reward update or if state-dependent update */
  virtual double reward(const ValueMap& static_fluents_values) const;

 protected:
  /* Prints this object on the given stream. */
  virtual void print(std::ostream& os) const;
};


/* ====================================================================== */
/* Increase */

/*
 * An increase update.
 */
struct Increase : public Update {
  /* Constructs an increase update. */
  Increase(const Fluent& fluent, const Expression& expr)
    : Update(fluent, expr) {}

  /* Changes the given state according to this update. */
  virtual void affect(ValueMap& values) const;

  /* Returns an instantiaion of this update. */
  virtual const Update& instantiation(const SubstitutionMap& subst,
                                      const ValueMap& values) const;

  /* Returns the reward update or 0 if not a reward update or if state-dependent update */
  virtual double reward(const ValueMap& static_fluents_values) const;

 protected:
  /* Prints this object on the given stream. */
  virtual void print(std::ostream& os) const;
};


/* ====================================================================== */
/* Decrease */

/*
 * An decrease update.
 */
struct Decrease : public Update {
  /* Constructs an decrease update. */
  Decrease(const Fluent& fluent, const Expression& expr)
    : Update(fluent, expr) {}

  /* Changes the given state according to this update. */
  virtual void affect(ValueMap& values) const;

  /* Returns an instantiaion of this update. */
  virtual const Update& instantiation(const SubstitutionMap& subst,
                                      const ValueMap& values) const;

  /* Returns the reward update or 0 if not a reward update or if state-dependent update */
  virtual double reward(const ValueMap& static_fluents_values) const;

 protected:
  /* Prints this object on the given stream. */
  virtual void print(std::ostream& os) const;
};


/* ====================================================================== */
/* UpdateSet */

/*
 * Set of updates.
 */
struct UpdateSet : public std::set<const Update*> {
};


/* ====================================================================== */
/* EffectList */

/*
 * List of effects.
 */
struct Effect;
struct EffectList : public std::vector<const Effect*> {
};


/* ====================================================================== */
/* Effect */

/*
 * An effect.
 */
struct Effect : public RCObject {
  /* The empty effect. */
  static const Effect& EMPTY;

  /* Tests if this is the empty effect. */
  bool empty() const { return this == &EMPTY; }

  struct FlatTransition
  {
	double probability_;
	AtomSet adds_;
	AtomSet deletes_;
	UpdateSet updates_;
	FlatTransition() {}
	FlatTransition(double probability, AtomSet adds, AtomSet deletes, UpdateSet updates)
		: probability_(probability), adds_(adds), deletes_(deletes), updates_(updates) {}
  };
  typedef std::list<FlatTransition> transition_list_t;

  /* Generates the list of next states (flat transitions)
     from the given state by applying this effect */
  virtual void transitions(const TermTable& terms,
						   const AtomSet& atoms, const ValueMap& values,
						   transition_list_t& flat_transitions) const = 0;

  /* Fills the provided lists with a sampled state change for this
     effect in the given state. */
  virtual void state_change(AtomList& adds, AtomList& deletes,
                            UpdateList& updates,
                            const TermTable& terms,
                            const AtomSet& atoms,
                            const ValueMap& values) const = 0;

  /* Inserts the atoms become true, and
     erases the atoms becoming false */
  virtual void relaxation(const AtomSet& true_atoms_pre,
						  const AtomSet& false_atoms_pre,
						  AtomSet& true_atoms_post,
						  AtomSet& false_atoms_post) const = 0;

  /* Sets the weights of atoms becoming true or false to minimum of
     their current weight and the given weight, and
     returns true if at least one predicate has been modified */
  virtual bool hadd_relaxation(std::vector<unsigned int>& true_atoms_weights,
							   std::vector<unsigned int>& false_atoms_weights,
							   unsigned int weight) const = 0;

  /* Sets the weights of atoms becoming true or false to the given weight, and
     returns true if at least one predicate has been modified */
  virtual bool hmax_relaxation(std::vector<unsigned int>& true_atoms_weights,
							   std::vector<unsigned int>& false_atoms_weights,
							   unsigned int weight) const = 0;

  /* Returns all the deterministic effects of this effect */
  virtual void ao_determinization(EffectList& deterministic_effects) const = 0;

  /* Returns all the deterministic effects of this effect with fluents included and pairs of probability/reward returned */
  virtual void ao_fdeterminization(EffectList& deterministic_effects, std::vector<std::pair<double, double> >& pr, const ValueMap& static_fluents_values) const = 0;

  /* Returns the most probable outcome effect of this effect */
  virtual const Effect* mpo_determinization() const = 0;

  /* Returns the most probable outcome effect of this effect with fluents included and pairs of probability/reward returned */
  virtual const Effect* mpo_fdeterminization(double& probability, double& reward, const ValueMap& static_fluents_values) const = 0;

  /* Returns an instantiation of this effect. */
  virtual const Effect& instantiation(const SubstitutionMap& subst,
                                      const TermTable& terms,
                                      const AtomSet& atoms,
                                      const ValueMap& values) const = 0;

 protected:
  /* Prints this object on the given stream. */
  virtual void print(std::ostream& os) const = 0;

  friend std::ostream& operator<<(std::ostream& os, const Effect& e);
};

/* Conjunction operator for effects. */
const Effect& operator&&(const Effect& e1, const Effect& e2);

/* Output operator for effects. */
std::ostream& operator<<(std::ostream& os, const Effect& e);


/* ====================================================================== */
/* SimpleEffect */

/*
 * A simple effect.
 */
struct SimpleEffect : public Effect {
  /* Deletes this simple effect. */
  virtual ~SimpleEffect();

  /* Returns the atom associated with this simple effect. */
  const Atom& atom() const { return *atom_; }

 protected:
  /* Constructs a simple effect. */
  explicit SimpleEffect(const Atom& atom);

 private:
  /* Atom added by this effect. */
  const Atom* atom_;
};


/* ====================================================================== */
/* AddEffect */

/*
 * An add effect.
 */
struct AddEffect : public SimpleEffect {
  /* Constructs an add effect. */
  explicit AddEffect(const Atom& atom) : SimpleEffect(atom) {}

  /* Generates the list of next states (flat transitions)
     from the given state by applying this effect */
  virtual void transitions(const TermTable& terms,
						   const AtomSet& atoms, const ValueMap& values,
						   transition_list_t& flat_transitions) const;

  /* Fills the provided lists with a sampled state change for this
     effect in the given state. */
  virtual void state_change(AtomList& adds, AtomList& deletes,
                            UpdateList& updates,
                            const TermTable& terms,
                            const AtomSet& atoms,
                            const ValueMap& values) const;

  /* Inserts the atoms become true, and
     erases the atoms becoming false */
  virtual void relaxation(const AtomSet& true_atoms_pre,
						  const AtomSet& false_atoms_pre,
						  AtomSet& true_atoms_post,
						  AtomSet& false_atoms_post) const;

  /* Sets the weights of atoms becoming true or false to minimum of
     their current weight and the given weight, and
     returns true if at least one predicate has been modified */
  virtual bool hadd_relaxation(std::vector<unsigned int>& true_atoms_weights,
							   std::vector<unsigned int>& false_atoms_weights,
							   unsigned int weight) const;

  /* Sets the weights of atoms becoming true or false to the given weight, and
     returns true if at least one predicate has been modified */
  virtual bool hmax_relaxation(std::vector<unsigned int>& true_atoms_weights,
							   std::vector<unsigned int>& false_atoms_weights,
							   unsigned int weight) const;

  /* Returns all the deterministic effects of this effect */
  virtual void ao_determinization(EffectList& deterministic_effects) const;

  /* Returns all the deterministic effects of this effect with fluents included and pairs of probability/reward returned */
  virtual void ao_fdeterminization(EffectList& deterministic_effects, std::vector<std::pair<double, double> >& pr, const ValueMap& static_fluents_values) const;

  /* Returns the most probable outcome effect of this effect */
  virtual const Effect* mpo_determinization() const;

  /* Returns the most probable outcome effect of this effect with fluents included and pairs of probability/reward returned */
  virtual const Effect* mpo_fdeterminization(double& probability, double& reward, const ValueMap& static_fluents_values) const;

  /* Returns an instantiation of this effect. */
  virtual const Effect& instantiation(const SubstitutionMap& subst,
                                      const TermTable& terms,
                                      const AtomSet& atoms,
                                      const ValueMap& values) const;

 protected:
  /* Prints this object on the given stream. */
  virtual void print(std::ostream& os) const;
};


/* ====================================================================== */
/* DeleteEffect */

/*
 * A delete effect.
 */
struct DeleteEffect : public SimpleEffect {
  /* Constructs a delete effect. */
  explicit DeleteEffect(const Atom& atom) : SimpleEffect(atom) {}

  /* Generates the list of next states (flat transitions)
     from the given state by applying this effect */
  virtual void transitions(const TermTable& terms,
						   const AtomSet& atoms, const ValueMap& values,
						   transition_list_t& flat_transitions) const;

  /* Fills the provided lists with a sampled state change for this
     effect in the given state. */
  virtual void state_change(AtomList& adds, AtomList& deletes,
                            UpdateList& updates,
                            const TermTable& terms,
                            const AtomSet& atoms,
                            const ValueMap& values) const;

  /* Inserts the atoms become true, and
     erases the atoms becoming false */
  virtual void relaxation(const AtomSet& true_atoms_pre,
						  const AtomSet& false_atoms_pre,
						  AtomSet& true_atoms_post,
						  AtomSet& false_atoms_post) const;

  /* Sets the weights of atoms becoming true or false to minimum of
     their current weight and the given weight, and
     returns true if at least one predicate has been modified */
  virtual bool hadd_relaxation(std::vector<unsigned int>& true_atoms_weights,
							   std::vector<unsigned int>& false_atoms_weights,
							   unsigned int weight) const;

  /* Sets the weights of atoms becoming true or false to the given weight, and
     returns true if at least one predicate has been modified */
  virtual bool hmax_relaxation(std::vector<unsigned int>& true_atoms_weights,
							   std::vector<unsigned int>& false_atoms_weights,
							   unsigned int weight) const;

  /* Returns all the deterministic effects of this effect */
  virtual void ao_determinization(EffectList& deterministic_effects) const;

  /* Returns all the deterministic effects of this effect with fluents included and pairs of probability/reward returned */
  virtual void ao_fdeterminization(EffectList& deterministic_effects, std::vector<std::pair<double, double> >& pr, const ValueMap& static_fluents_values) const;

  /* Returns the most probable outcome effect of this effect */
  virtual const Effect* mpo_determinization() const;

  /* Returns the most probable outcome effect of this effect with fluents included and pairs of probability/reward returned */
  virtual const Effect* mpo_fdeterminization(double& probability, double& reward, const ValueMap& static_fluents_values) const;

  /* Returns an instantiation of this effect. */
  virtual const Effect& instantiation(const SubstitutionMap& subst,
                                      const TermTable& terms,
                                      const AtomSet& atoms,
                                      const ValueMap& values) const;

 protected:
  /* Prints this object on the given stream. */
  virtual void print(std::ostream& os) const;
};


/* ====================================================================== */
/* UpdateEffect */

/*
 * An update effect.
 */
struct UpdateEffect : public Effect {
  /* Returns an effect for the given update. */
  static const Effect& make(const Update& update);

  /* Deletes this update effect. */
  virtual ~UpdateEffect();

  /* Returns the update performed by this effect. */
  const Update& update() const { return *update_; }

  /* Generates the list of next states (flat transitions)
     from the given state by applying this effect */
  virtual void transitions(const TermTable& terms,
						   const AtomSet& atoms, const ValueMap& values,
						   transition_list_t& flat_transitions) const;

  /* Fills the provided lists with a sampled state change for this
     effect in the given state. */
  virtual void state_change(AtomList& adds, AtomList& deletes,
                            UpdateList& updates,
                            const TermTable& terms,
                            const AtomSet& atoms,
                            const ValueMap& values) const;

  /* Inserts the atoms become true, and
     erases the atoms becoming false */
  virtual void relaxation(const AtomSet& true_atoms_pre,
						  const AtomSet& false_atoms_pre,
						  AtomSet& true_atoms_post,
						  AtomSet& false_atoms_post) const;

  /* Sets the weights of atoms becoming true or false to minimum of
     their current weight and the given weight, and
     returns true if at least one predicate has been modified */
  virtual bool hadd_relaxation(std::vector<unsigned int>& true_atoms_weights,
							   std::vector<unsigned int>& false_atoms_weights,
							   unsigned int weight) const;

  /* Sets the weights of atoms becoming true or false to the given weight, and
     returns true if at least one predicate has been modified */
  virtual bool hmax_relaxation(std::vector<unsigned int>& true_atoms_weights,
							   std::vector<unsigned int>& false_atoms_weights,
							   unsigned int weight) const;

  /* Returns all the deterministic effects of this effect */
  virtual void ao_determinization(EffectList& deterministic_effects) const;

  /* Returns all the deterministic effects of this effect with fluents included and pairs of probability/reward returned */
  virtual void ao_fdeterminization(EffectList& deterministic_effects, std::vector<std::pair<double, double> >& pr, const ValueMap& static_fluents_values) const;

  /* Returns the most probable outcome effect of this effect */
  virtual const Effect* mpo_determinization() const;

  /* Returns the most probable outcome effect of this effect with fluents included and pairs of probability/reward returned */
  virtual const Effect* mpo_fdeterminization(double& probability, double& reward, const ValueMap& static_fluents_values) const;

  /* Returns an instantiation of this effect. */
  virtual const Effect& instantiation(const SubstitutionMap& subst,
                                      const TermTable& terms,
                                      const AtomSet& atoms,
                                      const ValueMap& values) const;

 protected:
  /* Prints this object on the given stream. */
  virtual void print(std::ostream& os) const;

 private:
  /* Update performed by this effect. */
  const Update* update_;

  /* Constructs an update effect. */
  explicit UpdateEffect(const Update& update) : update_(&update) {}
};


/* ====================================================================== */
/* ConjunctiveEffect */

/*
 * A conjunctive effect.
 */
struct ConjunctiveEffect : public Effect {
  /* Deletes this conjunctive effect. */
  virtual ~ConjunctiveEffect();

  /* Returns the conjuncts of this conjunctive effect. */
  const EffectList& conjuncts() const { return conjuncts_; }

  /* Generates the list of next states (flat transitions)
     from the given state by applying this effect */
  virtual void transitions(const TermTable& terms,
						   const AtomSet& atoms, const ValueMap& values,
						   transition_list_t& flat_transitions) const;

  /* Fills the provided lists with a sampled state change for this
     effect in the given state. */
  virtual void state_change(AtomList& adds, AtomList& deletes,
                            UpdateList& updates,
                            const TermTable& terms,
                            const AtomSet& atoms,
                            const ValueMap& values) const;

  /* Inserts the atoms become true, and
     erases the atoms becoming false */
  virtual void relaxation(const AtomSet& true_atoms_pre,
						  const AtomSet& false_atoms_pre,
						  AtomSet& true_atoms_post,
						  AtomSet& false_atoms_post) const;

  /* Sets the weights of atoms becoming true or false to minimum of
     their current weight and the given weight, and
     returns true if at least one predicate has been modified */
  virtual bool hadd_relaxation(std::vector<unsigned int>& true_atoms_weights,
							   std::vector<unsigned int>& false_atoms_weights,
							   unsigned int weight) const;

  /* Sets the weights of atoms becoming true or false to the given weight, and
     returns true if at least one predicate has been modified */
  virtual bool hmax_relaxation(std::vector<unsigned int>& true_atoms_weights,
							   std::vector<unsigned int>& false_atoms_weights,
							   unsigned int weight) const;

  /* Returns all the deterministic effects of this effect */
  virtual void ao_determinization(EffectList& deterministic_effects) const;

  /* Returns all the deterministic effects of this effect with fluents included and pairs of probability/reward returned */
  virtual void ao_fdeterminization(EffectList& deterministic_effects, std::vector<std::pair<double, double> >& pr, const ValueMap& static_fluents_values) const;

  /* Returns the most probable outcome effect of this effect */
  virtual const Effect* mpo_determinization() const;

  /* Returns the most probable outcome effect of this effect with fluents included and pairs of probability/reward returned */
  virtual const Effect* mpo_fdeterminization(double& probability, double& reward, const ValueMap& static_fluents_values) const;

  /* Returns an instantiation of this effect. */
  virtual const Effect& instantiation(const SubstitutionMap& subst,
                                      const TermTable& terms,
                                      const AtomSet& atoms,
                                      const ValueMap& values) const;

 protected:
  /* Prints this object on the given stream. */
  virtual void print(std::ostream& os) const;

 private:
  /* The conjuncts. */
  EffectList conjuncts_;

  /* Constructs a conjunctive effect. */
  ConjunctiveEffect() {}

  /* Adds a conjunct to this conjunctive effect. */
  void add_conjunct(const Effect& conjunct);

  friend const Effect& operator&&(const Effect& e1, const Effect& e2);
};


/* ====================================================================== */
/* ConditionalEffect */

/*
 * A conditional effect.
 */
struct ConditionalEffect : public Effect {
  /* Returns a conditional effect. */
  static const Effect& make(const StateFormula& condition,
                            const Effect& effect);

  /* Deletes this conditional effect. */
  virtual ~ConditionalEffect();

  /* Returns the condition of this effect. */
  const StateFormula& condition() const { return *condition_; }

  /* Returns the conditional effect of this effect. */
  const Effect& effect() const { return *effect_; }

  /* Generates the list of next states (flat transitions)
     from the given state by applying this effect */
  virtual void transitions(const TermTable& terms,
						   const AtomSet& atoms, const ValueMap& values,
						   transition_list_t& flat_transitions) const;

  /* Fills the provided lists with a sampled state change for this
     effect in the given state. */
  virtual void state_change(AtomList& adds, AtomList& deletes,
                            UpdateList& updates,
                            const TermTable& terms,
                            const AtomSet& atoms,
                            const ValueMap& values) const;

  /* Inserts the atoms become true, and
     erases the atoms becoming false */
  virtual void relaxation(const AtomSet& true_atoms_pre,
						  const AtomSet& false_atoms_pre,
						  AtomSet& true_atoms_post,
						  AtomSet& false_atoms_post) const;

  /* Sets the weights of atoms becoming true or false to minimum of
     their current weight and the given weight, and
     returns true if at least one predicate has been modified */
  virtual bool hadd_relaxation(std::vector<unsigned int>& true_atoms_weights,
							   std::vector<unsigned int>& false_atoms_weights,
							   unsigned int weight) const;

  /* Sets the weights of atoms becoming true or false to the given weight, and
     returns true if at least one predicate has been modified */
  virtual bool hmax_relaxation(std::vector<unsigned int>& true_atoms_weights,
							   std::vector<unsigned int>& false_atoms_weights,
							   unsigned int weight) const;

  /* Returns all the deterministic effects of this effect */
  virtual void ao_determinization(EffectList& deterministic_effects) const;

  /* Returns all the deterministic effects of this effect with fluents included and pairs of probability/reward returned */
  virtual void ao_fdeterminization(EffectList& deterministic_effects, std::vector<std::pair<double, double> >& pr, const ValueMap& static_fluents_values) const;

  /* Returns the most probable outcome effect of this effect */
  virtual const Effect* mpo_determinization() const;

  /* Returns the most probable outcome effect of this effect with fluents included and pairs of probability/reward returned */
  virtual const Effect* mpo_fdeterminization(double& probability, double& reward, const ValueMap& static_fluents_values) const;

  /* Returns an instantiation of this effect. */
  virtual const Effect& instantiation(const SubstitutionMap& subst,
                                      const TermTable& terms,
                                      const AtomSet& atoms,
                                      const ValueMap& values) const;

 protected:
  /* Prints this object on the given stream. */
  virtual void print(std::ostream& os) const;

 private:
  /* Effect condition. */
  const StateFormula* condition_;
  /* Effect. */
  const Effect* effect_;

  /* Constructs a conditional effect. */
  ConditionalEffect(const StateFormula& condition, const Effect& effect);
};


/* ====================================================================== */
/* ProbabilisticEffect */

/*
 * A probabilistic effect.
 */
struct ProbabilisticEffect : public Effect {
  /* Returns a probabilistic effect. */
  static const Effect&
  make(const std::vector<std::pair<Rational, const Effect*> >& os);

  /* Deletes this probabilistic effect. */
  virtual ~ProbabilisticEffect();

  /* Returns the number of outcomes of this probabilistic effect. */
  size_t size() const { return weights_.size(); }

  /* Returns the ith outcome's probability. */
  Rational probability(size_t i) const {
    return Rational(weights_[i], weight_sum_);
  }

  /* Returns the ith outcome's effect. */
  const Effect& effect(size_t i) const { return *effects_[i]; }

  /* Generates the list of next states (flat transitions)
     from the given state by applying this effect */
  virtual void transitions(const TermTable& terms,
						   const AtomSet& atoms, const ValueMap& values,
						   transition_list_t& flat_transitions) const;

  /* Fills the provided lists with a sampled state change for this
     effect in the given state. */
  virtual void state_change(AtomList& adds, AtomList& deletes,
                            UpdateList& updates,
                            const TermTable& terms,
                            const AtomSet& atoms,
                            const ValueMap& values) const;

  /* Inserts the atoms become true, and
     erases the atoms becoming false */
  virtual void relaxation(const AtomSet& true_atoms_pre,
						  const AtomSet& false_atoms_pre,
						  AtomSet& true_atoms_post,
						  AtomSet& false_atoms_post) const;

  /* Sets the weights of atoms becoming true or false to minimum of
     their current weight and the given weight, and
     returns true if at least one predicate has been modified */
  virtual bool hadd_relaxation(std::vector<unsigned int>& true_atoms_weights,
							   std::vector<unsigned int>& false_atoms_weights,
							   unsigned int weight) const;

  /* Sets the weights of atoms becoming true or false to the given weight, and
     returns true if at least one predicate has been modified */
  virtual bool hmax_relaxation(std::vector<unsigned int>& true_atoms_weights,
							   std::vector<unsigned int>& false_atoms_weights,
							   unsigned int weight) const;

  /* Returns all the deterministic effects of this effect */
  virtual void ao_determinization(EffectList& deterministic_effects) const;

  /* Returns all the deterministic effects of this effect with fluents included and pairs of probability/reward returned */
  virtual void ao_fdeterminization(EffectList& deterministic_effects, std::vector<std::pair<double, double> >& pr, const ValueMap& static_fluents_values) const;

  /* Returns the most probable outcome effect of this effect */
  virtual const Effect* mpo_determinization() const;

  /* Returns the most probable outcome effect of this effect with fluents included and pairs of probability/reward returned */
  virtual const Effect* mpo_fdeterminization(double& probability, double& reward, const ValueMap& static_fluents_values) const;

  /* Returns an instantiation of this effect. */
  virtual const Effect& instantiation(const SubstitutionMap& subst,
                                      const TermTable& terms,
                                      const AtomSet& atoms,
                                      const ValueMap& values) const;

 protected:
  /* Prints this object on the given stream. */
  virtual void print(std::ostream& os) const;

 private:
  /* Weights associated with outcomes. */
  std::vector<int> weights_;
  /* The sum of weights. */
  int weight_sum_;
  /* Outcome effects. */
  EffectList effects_;

  /* Constructs an empty probabilistic effect. */
  ProbabilisticEffect() : weight_sum_(0) {}

  /* Adds an outcome to this probabilistic effect. */
  bool add_outcome(const Rational& p, const Effect& effect);
};


/* ====================================================================== */
/* QuantifiedEffect */

/*
 * A universally quantified effect.
 */
struct QuantifiedEffect : public Effect {
  /* Returns a universally quantified effect. */
  static const Effect& make(const VariableList& parameters,
                            const Effect& body);

  /* Deletes this universally quantifed effect. */
  virtual ~QuantifiedEffect();

  /* Returns the parameters of this universally quantified effect. */
  const VariableList& parameters() const { return parameters_; }

  /* Returns the quantified effect. */
  const Effect& effect() const { return *effect_; }

  /* Generates the list of next states (flat transitions)
     from the given state by applying this effect */
  virtual void transitions(const TermTable& terms,
						   const AtomSet& atoms, const ValueMap& values,
						   transition_list_t& flat_transitions) const;

  /* Fills the provided lists with a sampled state change for this
     effect in the given state. */
  virtual void state_change(AtomList& adds, AtomList& deletes,
                            UpdateList& updates,
                            const TermTable& terms,
                            const AtomSet& atoms,
                            const ValueMap& values) const;

  /* Inserts the atoms become true, and
     erases the atoms becoming false */
  virtual void relaxation(const AtomSet& true_atoms_pre,
						  const AtomSet& false_atoms_pre,
						  AtomSet& true_atoms_post,
						  AtomSet& false_atoms_post) const;

  /* Sets the weights of atoms becoming true or false to minimum of
     their current weight and the given weight, and
     returns true if at least one predicate has been modified */
  virtual bool hadd_relaxation(std::vector<unsigned int>& true_atoms_weights,
							   std::vector<unsigned int>& false_atoms_weights,
							   unsigned int weight) const;

  /* Sets the weights of atoms becoming true or false to the given weight, and
     returns true if at least one predicate has been modified */
  virtual bool hmax_relaxation(std::vector<unsigned int>& true_atoms_weights,
							   std::vector<unsigned int>& false_atoms_weights,
							   unsigned int weight) const;

  /* Returns all the deterministic effects of this effect */
  virtual void ao_determinization(EffectList& deterministic_effects) const;

  /* Returns all the deterministic effects of this effect with fluents included and pairs of probability/reward returned */
  virtual void ao_fdeterminization(EffectList& deterministic_effects, std::vector<std::pair<double, double> >& pr, const ValueMap& static_fluents_values) const;

  /* Returns the most probable outcome effect of this effect */
  virtual const Effect* mpo_determinization() const;

  /* Returns the most probable outcome effect of this effect with fluents included and pairs of probability/reward returned */
  virtual const Effect* mpo_fdeterminization(double& probability, double& reward, const ValueMap& static_fluents_values) const;

  /* Returns an instantiation of this effect. */
  virtual const Effect& instantiation(const SubstitutionMap& subst,
                                      const TermTable& terms,
                                      const AtomSet& atoms,
                                      const ValueMap& values) const;

 protected:
  /* Prints this object on the given stream. */
  virtual void print(std::ostream& os) const;

 private:
  /* Quantified variables. */
  VariableList parameters_;
  /* The quantified effect. */
  const Effect* effect_;

  /* Constructs a universally quantified effect. */
  explicit QuantifiedEffect(const VariableList& parameters,
                            const Effect& effect);
};


#endif /* EFFECTS_H */
