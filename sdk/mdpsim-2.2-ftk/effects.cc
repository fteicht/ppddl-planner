/*
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
#include "effects.h"
#include <stack>
#include <limits>
#include <algorithm>
#include <stdexcept>
#include <typeinfo>
#include <cstdlib>


/* ====================================================================== */
/* Update */

/* Constructs an update. */
Update::Update(const Fluent& fluent, const Expression& expr)
  : fluent_(&fluent), expr_(&expr) {
  RCObject::ref(fluent_);
  RCObject::ref(expr_);
}


/* Deletes this update. */
Update::~Update() {
  RCObject::destructive_deref(fluent_);
  RCObject::destructive_deref(expr_);
}


/* Output operator for updates. */
std::ostream& operator<<(std::ostream& os, const Update& u) {
  u.print(os);
  return os;
}


/* ====================================================================== */
/* Assign */

/* Changes the given state according to this update. */
void Assign::affect(ValueMap& values) const {
  values[&fluent()] = expression().value(values);
}


/* Returns an instantiaion of this update. */
const Update& Assign::instantiation(const SubstitutionMap& subst,
                                    const ValueMap& values) const {
  return *new Assign(fluent().substitution(subst),
                     expression().instantiation(subst, values));
}


double Assign::reward(const ValueMap& static_fluents_values) const {
    if (FunctionTable::name(fluent().function()) == "reward") {
        std::cerr << "Assigning reward interpreted as increasing it!" << std::endl;
        try {
            return expression().value(static_fluents_values).double_value();
        } catch (const std::logic_error& e) {
            std::cerr << "State-dependent reward cast to 0!" << std::endl;
            return 0.0;
        }
    } else {
        return 0.0;
    }
}


/* Prints this object on the given stream. */
void Assign::print(std::ostream& os) const {
  os << "(assign " << fluent() << ' ' << expression() << ")";
}


/* ====================================================================== */
/* ScaleUp */

/* Changes the given state according to this update. */
void ScaleUp::affect(ValueMap& values) const {
  ValueMap::const_iterator vi = values.find(&fluent());
  if (vi == values.end()) {
    throw std::logic_error("changing undefined value");
  } else {
    values[&fluent()] = (*vi).second * expression().value(values);
  }
}


/* Returns an instantiaion of this update. */
const Update& ScaleUp::instantiation(const SubstitutionMap& subst,
                                     const ValueMap& values) const {
  return *new ScaleUp(fluent().substitution(subst),
                      expression().instantiation(subst, values));
}


double ScaleUp::reward(const ValueMap& static_fluents_values) const {
    if (FunctionTable::name(fluent().function()) == "reward") {
        std::cerr << "Scaling-up reward interpreted as increasing it!" << std::endl;
        try {
            return expression().value(static_fluents_values).double_value();
        } catch (const std::logic_error& e) {
            std::cerr << "State-dependent reward cast to 0!" << std::endl;
            return 0.0;
        }
    } else {
        return 0.0;
    }
}


/* Prints this object on the given stream. */
void ScaleUp::print(std::ostream& os) const {
  os << "(scale-up " << fluent() << ' ' << expression() << ")";
}


/* ====================================================================== */
/* ScaleDown */

/* Changes the given state according to this update. */
void ScaleDown::affect(ValueMap& values) const {
  ValueMap::const_iterator vi = values.find(&fluent());
  if (vi == values.end()) {
    throw std::logic_error("changing undefined value");
  } else {
    values[&fluent()] = (*vi).second / expression().value(values);
  }
}


/* Returns an instantiaion of this update. */
const Update& ScaleDown::instantiation(const SubstitutionMap& subst,
                                       const ValueMap& values) const {
  return *new ScaleDown(fluent().substitution(subst),
                        expression().instantiation(subst, values));
}


double ScaleDown::reward(const ValueMap& static_fluents_values) const {
    if (FunctionTable::name(fluent().function()) == "reward") {
        std::cerr << "Scaling-down reward interpreted as decreasing it!" << std::endl;
        try {
            return -(expression().value(static_fluents_values).double_value());
        } catch (const std::logic_error& e) {
            std::cerr << "State-dependent reward cast to 0!" << std::endl;
            return 0.0;
        }
    } else {
        return 0.0;
    }
}


/* Prints this object on the given stream. */
void ScaleDown::print(std::ostream& os) const {
  os << "(scale-down " << fluent() << ' ' << expression() << ")";
}


/* ====================================================================== */
/* Increase */

/* Changes the given state according to this update. */
void Increase::affect(ValueMap& values) const {
  ValueMap::const_iterator vi = values.find(&fluent());
  if (vi == values.end()) {
    throw std::logic_error("changing undefined value");
  } else {
    values[&fluent()] = (*vi).second + expression().value(values);
  }
}


/* Returns an instantiaion of this update. */
const Update& Increase::instantiation(const SubstitutionMap& subst,
                                      const ValueMap& values) const {
  return *new Increase(fluent().substitution(subst),
                       expression().instantiation(subst, values));
}


double Increase::reward(const ValueMap& static_fluents_values) const {
    if (FunctionTable::name(fluent().function()) == "reward") {
        try {
            return expression().value(static_fluents_values).double_value();
        } catch (const std::logic_error& e) {
            std::cerr << "State-dependent reward cast to 0!" << std::endl;
            return 0.0;
        }
    } else {
        return 0.0;
    }
}


/* Prints this object on the given stream. */
void Increase::print(std::ostream& os) const {
  os << "(increase " << fluent() << ' ' << expression() << ")";
}


/* ====================================================================== */
/* Decrease */

/* Changes the given state according to this update. */
void Decrease::affect(ValueMap& values) const {
  ValueMap::const_iterator vi = values.find(&fluent());
  if (vi == values.end()) {
    throw std::logic_error("changing undefined value");
  } else {
    values[&fluent()] = (*vi).second - expression().value(values);
  }
}


/* Returns an instantiaion of this update. */
const Update& Decrease::instantiation(const SubstitutionMap& subst,
                                      const ValueMap& values) const {
  return *new Decrease(fluent().substitution(subst),
                       expression().instantiation(subst, values));
}


double Decrease::reward(const ValueMap& static_fluents_values) const {
    if (FunctionTable::name(fluent().function()) == "reward") {
        try {
            return -(expression().value(static_fluents_values).double_value());
        } catch (const std::logic_error& e) {
            std::cerr << "State-dependent reward cast to 0!" << std::endl;
            return 0.0;
        }
    } else {
        return 0.0;
    }
}


/* Prints this object on the given stream. */
void Decrease::print(std::ostream& os) const {
  os << "(decrease " << fluent() << ' ' << expression() << ")";
}


/* ====================================================================== */
/* EmptyEffect */

/*
 * An empty effect.
 */
struct EmptyEffect : public Effect {
  /* Generates the list of next states (flat transitions)
     from the given state by applying this effect */
  virtual void transitions(const TermTable& terms,
						   const AtomSet& atoms, const ValueMap& values,
						   transition_list_t& flat_transitions) const {
	flat_transitions.push_back(FlatTransition(1.0, AtomSet(), AtomSet(), UpdateSet()));
  }

  /* Fills the provided lists with a sampled state change for this
     effect in the given state. */
  virtual void state_change(AtomList& adds, AtomList& deletes,
                            UpdateList& updates,
                            const TermTable& terms,
                            const AtomSet& atoms,
                            const ValueMap& values) const {}

  /* Returns an instantiation of this effect. */
  virtual const Effect& instantiation(const SubstitutionMap& subst,
                                      const TermTable& terms,
                                      const AtomSet& atoms,
                                      const ValueMap& values) const {
    return *this;
  }

  /* Inserts the atoms become true, and
     erases the atoms becoming false */
  virtual void relaxation(const AtomSet& true_atoms_pre,
						  const AtomSet& false_atoms_pre,
						  AtomSet& true_atoms_post,
						  AtomSet& false_atoms_post) const {}

  /* Sets the weights of atoms becoming true or false to minimum of
     their current weight and the given weight, and
     returns true if at least one predicate has been modified */
  virtual bool hadd_relaxation(std::vector<unsigned int>& true_atoms_weights,
							   std::vector<unsigned int>& false_atoms_weights,
							   unsigned int weight) const {
	return false;
  }

  /* Sets the weights of atoms becoming true or false to the given weight, and
     returns true if at least one predicate has been modified */
  virtual bool hmax_relaxation(std::vector<unsigned int>& true_atoms_weights,
							   std::vector<unsigned int>& false_atoms_weights,
							   unsigned int weight) const {
	return false;
  }

  /* Returns all the deterministic effects of this effect */
  virtual void ao_determinization(EffectList& deterministic_effects) const {
	deterministic_effects.push_back(&Effect::EMPTY);
	RCObject::ref(&Effect::EMPTY);
  }

  /* Returns all the deterministic effects of this effect */
  virtual void ao_fdeterminization(EffectList& deterministic_effects, std::vector<std::pair<double, double> >& pr, const ValueMap& static_fluents_values) const {
    pr.push_back(std::make_pair(1.0, 0.0));
	deterministic_effects.push_back(&Effect::EMPTY);
	RCObject::ref(&Effect::EMPTY);
  }

  /* Returns the most probable outcome effect of this effect */
  virtual const Effect* mpo_determinization() const {
	RCObject::ref(&Effect::EMPTY);
	return &Effect::EMPTY;
  }

  /* Returns the most probable outcome effect of this effect */
  virtual const Effect* mpo_fdeterminization(double& probability, double& reward, const ValueMap& static_fluents_values) const {
    probability = 1.0;
    reward = 0.0;
	RCObject::ref(&Effect::EMPTY);
	return &Effect::EMPTY;
  }

 protected:
  /* Prints this object on the given stream. */
  virtual void print(std::ostream& os) const { os << "(and)"; }

 private:
  /* Constant representing the empty effect. */
  static const EmptyEffect EMPTY_;

  /* Constructs an empty effect. */
  EmptyEffect() { ref(this); }

  friend struct Effect;
};

/* Constant representing the empty effect. */
const EmptyEffect EmptyEffect::EMPTY_;


/* ====================================================================== */
/* Effect */

/* The empty effect. */
const Effect& Effect::EMPTY = EmptyEffect::EMPTY_;


/* Conjunction operator for effects. */
const Effect& operator&&(const Effect& e1, const Effect& e2) {
  if (e1.empty()) {
    return e2;
  } else if (e2.empty()) {
    return e1;
  } else {
    ConjunctiveEffect& conjunction = *new ConjunctiveEffect();
    const ConjunctiveEffect* c1 = dynamic_cast<const ConjunctiveEffect*>(&e1);
    if (c1 != 0) {
      for (EffectList::const_iterator ei = c1->conjuncts().begin();
           ei != c1->conjuncts().end(); ei++) {
        conjunction.add_conjunct(**ei);
      }
      RCObject::ref(c1);
      RCObject::destructive_deref(c1);
    } else {
      conjunction.add_conjunct(e1);
    }
    const ConjunctiveEffect* c2 = dynamic_cast<const ConjunctiveEffect*>(&e2);
    if (c2 != 0) {
      for (EffectList::const_iterator ei = c2->conjuncts().begin();
           ei != c2->conjuncts().end(); ei++) {
        conjunction.add_conjunct(**ei);
      }
      RCObject::ref(c2);
      RCObject::destructive_deref(c2);
    } else {
      conjunction.add_conjunct(e2);
    }
    return conjunction;
  }
}


/* Output operator for effects. */
std::ostream& operator<<(std::ostream& os, const Effect& e) {
  e.print(os);
  return os;
}


/* ====================================================================== */
/* SimpleEffect */

/* Constructs a simple effect. */
SimpleEffect::SimpleEffect(const Atom& atom)
  : atom_(&atom) {
  ref(atom_);
}


/* Deletes this simple effect. */
SimpleEffect::~SimpleEffect() {
  destructive_deref(atom_);
}


/* ====================================================================== */
/* AddEffect */

/* Generates the list of next states (flat transitions)
   from the given state by applying this effect */
void AddEffect::transitions(const TermTable& terms,
							const AtomSet& atoms, const ValueMap& values,
							transition_list_t& flat_transitions) const {
	flat_transitions.push_back(FlatTransition());
	flat_transitions.back().probability_ = 1.0;
	flat_transitions.back().adds_.insert(&(this->atom()));
}


/* Fills the provided lists with a sampled state change for this
   effect in the given state. */
void AddEffect::state_change(AtomList& adds, AtomList& deletes,
                             UpdateList& updates,
                             const TermTable& terms,
                             const AtomSet& atoms,
                             const ValueMap& values) const {
  adds.push_back(&atom());
}


/* Inserts the atoms become true, and
 erases the atoms becoming false */
void AddEffect::relaxation(const AtomSet& true_atoms_pre,
						   const AtomSet& false_atoms_pre,
						   AtomSet& true_atoms_post,
						   AtomSet& false_atoms_post) const {
	true_atoms_post.insert(&(this->atom()));
}


/* Sets the weights of atoms becoming true or false to minimum of
 their current weight and the given weight, and
 returns true if at least one predicate has been modified */
bool AddEffect::hadd_relaxation(std::vector<unsigned int>& true_atoms_weights,
								std::vector<unsigned int>& false_atoms_weights,
								unsigned int weight) const {
	if (true_atoms_weights[this->atom().index()] > weight)
	{
		true_atoms_weights[this->atom().index()] = weight;
		return true;
	}
	else
		return false;
}


/* Sets the weights of atoms becoming true or false to the given weight, and
 returns true if at least one predicate has been modified */
bool AddEffect::hmax_relaxation(std::vector<unsigned int>& true_atoms_weights,
								std::vector<unsigned int>& false_atoms_weights,
								unsigned int weight) const {
	if (true_atoms_weights[this->atom().index()] == std::numeric_limits<unsigned int>::max())
	{
		true_atoms_weights[this->atom().index()] = weight;
		return true;
	}
	else
		return false;
}


/* Returns all the deterministic effects of this effect */
void AddEffect::ao_determinization(EffectList& deterministic_effects) const {
	// Copy this effect
	deterministic_effects.push_back(this);
	RCObject::ref(this);
}


/* Returns all the deterministic effects of this effect */
void AddEffect::ao_fdeterminization(EffectList& deterministic_effects, std::vector<std::pair<double, double> >& pr, const ValueMap& static_fluents_values) const {
	// Copy this effect
	pr.push_back(std::make_pair(1.0, 0.0));
	deterministic_effects.push_back(this);
	RCObject::ref(this);
}


/* Returns the most probable outcome effect of this effect */
const Effect* AddEffect::mpo_determinization() const {
	// Copy this effect
	RCObject::ref(this);
	return this;
}


/* Returns the most probable outcome effect of this effect */
const Effect* AddEffect::mpo_fdeterminization(double& probability, double& reward, const ValueMap& static_fluents_values) const {
	// Copy this effect
	probability = 1.0;
	reward = 0.0;
	RCObject::ref(this);
	return this;
}


/* Returns an instantiation of this effect. */
const Effect& AddEffect::instantiation(const SubstitutionMap& subst,
                                       const TermTable& terms,
                                       const AtomSet& atoms,
                                       const ValueMap& values) const {
  const Atom* inst_atom = &atom().substitution(subst);
  if (inst_atom == &atom()) {
    return *this;
  } else {
    return *new AddEffect(*inst_atom);
  }
}


/* Prints this object on the given stream. */
void AddEffect::print(std::ostream& os) const {
  os << atom();
}


/* ====================================================================== */
/* DeleteEffect */

/* Generates the list of next states (flat transitions)
   from the given state by applying this effect */
void DeleteEffect::transitions(const TermTable& terms,
							   const AtomSet& atoms, const ValueMap& values,
							   transition_list_t& flat_transitions) const {
	flat_transitions.push_back(FlatTransition());
	flat_transitions.back().probability_ = 1.0;
	flat_transitions.back().deletes_.insert(&(this->atom()));
}


/* Fills the provided lists with a sampled state change for this
   effect in the given state. */
void DeleteEffect::state_change(AtomList& adds, AtomList& deletes,
                                UpdateList& updates,
                                const TermTable& terms,
                                const AtomSet& atoms,
                                const ValueMap& values) const {
  deletes.push_back(&atom());
}


/* Inserts the atoms become true, and
 erases the atoms becoming false */
void DeleteEffect::relaxation(const AtomSet& true_atoms_pre,
							  const AtomSet& false_atoms_pre,
							  AtomSet& true_atoms_post,
							  AtomSet& false_atoms_post) const {
	false_atoms_post.erase(&(this->atom()));
}


/* Sets the weights of atoms becoming true or false to minimum of
 their current weight and the given weight, and
 returns true if at least one predicate has been modified */
bool DeleteEffect::hadd_relaxation(std::vector<unsigned int>& true_atoms_weights,
								   std::vector<unsigned int>& false_atoms_weights,
								   unsigned int weight) const {
	if (false_atoms_weights[this->atom().index()] > weight)
	{
		false_atoms_weights[this->atom().index()] = weight;
		return true;
	}
	else
		return false;
}


/* Sets the weights of atoms becoming true or false to the given weight, and
 returns true if at least one predicate has been modified */
bool DeleteEffect::hmax_relaxation(std::vector<unsigned int>& true_atoms_weights,
								   std::vector<unsigned int>& false_atoms_weights,
								   unsigned int weight) const {
	if (false_atoms_weights[this->atom().index()] == std::numeric_limits<unsigned int>::max())
	{
		false_atoms_weights[this->atom().index()] = weight;
		return true;
	}
	else
		return false;
}


/* Returns all the deterministic effects of this effect */
void DeleteEffect::ao_determinization(EffectList& deterministic_effects) const {
	// Copy this effect
	deterministic_effects.push_back(this);
	RCObject::ref(this);
}


/* Returns all the deterministic effects of this effect */
void DeleteEffect::ao_fdeterminization(EffectList& deterministic_effects, std::vector<std::pair<double, double> >& pr, const ValueMap& static_fluents_values) const {
	// Copy this effect
	pr.push_back(std::make_pair(1.0, 0.0));
	deterministic_effects.push_back(this);
	RCObject::ref(this);
}


/* Returns the most probable outcome effect of this effect */
const Effect* DeleteEffect::mpo_determinization() const {
	// Copy this effect
	RCObject::ref(this);
	return this;
}


/* Returns the most probable outcome effect of this effect */
const Effect* DeleteEffect::mpo_fdeterminization(double& probability, double& reward, const ValueMap& static_fluents_values) const {
	// Copy this effect
	probability = 1.0;
	reward = 0.0;
	RCObject::ref(this);
	return this;
}


/* Returns an instantiation of this effect. */
const Effect& DeleteEffect::instantiation(const SubstitutionMap& subst,
                                          const TermTable& terms,
                                          const AtomSet& atoms,
                                          const ValueMap& values) const {
  const Atom* inst_atom = &atom().substitution(subst);
  if (inst_atom == &atom()) {
    return *this;
  } else {
    return *new DeleteEffect(*inst_atom);
  }
}


/* Prints this object on the given stream. */
void DeleteEffect::print(std::ostream& os) const {
  os << "(not " << atom() << ")";
}


/* ====================================================================== */
/* UpdateEffect */

/* Returns an effect for the given update. */
const Effect& UpdateEffect::make(const Update& update) {
  if (typeid(update) == typeid(ScaleUp)
      || typeid(update) == typeid(ScaleDown)) {
    const Value* v = dynamic_cast<const Value*>(&update.expression());
    if (v != 0 && v->value() == 1) {
      return EMPTY;
    }
  } else if (typeid(update) == typeid(Increase)
             || typeid(update) == typeid(Decrease)) {
    const Value* v = dynamic_cast<const Value*>(&update.expression());
    if (v != 0 && v->value() == 0) {
      return EMPTY;
    }
  }
  return *new UpdateEffect(update);
}


/* Deletes this update effect. */
UpdateEffect::~UpdateEffect() {
  delete update_;
}


/* Generates the list of next states (flat transitions)
   from the given state by applying this effect */
void UpdateEffect::transitions(const TermTable& terms,
							   const AtomSet& atoms, const ValueMap& values,
							   transition_list_t& flat_transitions) const {
	flat_transitions.push_back(FlatTransition());
	flat_transitions.back().probability_ = 1.0;
	flat_transitions.back().updates_.insert(update_);
}


/* Fills the provided lists with a sampled state change for this
   effect in the given state. */
void UpdateEffect::state_change(AtomList& adds, AtomList& deletes,
                                UpdateList& updates,
                                const TermTable& terms,
                                const AtomSet& atoms,
                                const ValueMap& values) const {
  updates.push_back(update_);
}


/* Inserts the atoms become true, and
 erases the atoms becoming false */
void UpdateEffect::relaxation(const AtomSet& true_atoms_pre,
							  const AtomSet& false_atoms_pre,
							  AtomSet& true_atoms_post,
							  AtomSet& false_atoms_post) const {
	//throw std::logic_error("UpdateEffect::relaxation not implemented");
	return; // takes into account reward updates!
}


/* Sets the weights of atoms becoming true or false to minimum of
 their current weight and the given weight, and
 returns true if at least one predicate has been modified */
bool UpdateEffect::hadd_relaxation(std::vector<unsigned int>& true_atoms_weights,
								   std::vector<unsigned int>& false_atoms_weights,
								   unsigned int weight) const {
	//throw std::logic_error("UpdateEffect::hadd_relaxation not implemented");
	return false; // takes into account reward updates!
}


/* Sets the weights of atoms becoming true or false to the given weight, and
 returns true if at least one predicate has been modified */
bool UpdateEffect::hmax_relaxation(std::vector<unsigned int>& true_atoms_weights,
								   std::vector<unsigned int>& false_atoms_weights,
								   unsigned int weight) const {
	//throw std::logic_error("UpdateEffect::hmax_relaxation not implemented");
	return false; // takes into account reward updates!
}


/* Returns all the deterministic effects of this effect */
void UpdateEffect::ao_determinization(EffectList& deterministic_effects) const {
	//throw std::logic_error("UpdateEffect::ao_determinization not implemented");
	deterministic_effects.push_back(&Effect::EMPTY); // takes into account reward updates!
	RCObject::ref(&Effect::EMPTY);
}


/* Returns all the deterministic effects of this effect */
void UpdateEffect::ao_fdeterminization(EffectList& deterministic_effects, std::vector<std::pair<double, double> >& pr, const ValueMap& static_fluents_values) const {
	// Copy this effect
	pr.push_back(std::make_pair(1.0, update_->reward(static_fluents_values)));
	deterministic_effects.push_back(this);
	RCObject::ref(this);
}


/* Returns the most probable outcome effect of this effect */
const Effect* UpdateEffect::mpo_determinization() const {
	//throw std::logic_error("UpdateEffect::mpo_determinization not implemented");
	RCObject::ref(&Effect::EMPTY); // takes into account reward updates!
	return &Effect::EMPTY;
}


/* Returns the most probable outcome effect of this effect */
const Effect* UpdateEffect::mpo_fdeterminization(double& probability, double& reward, const ValueMap& static_fluents_values) const {
	// Copy this effect
	probability = 1.0;
	reward = update_->reward(static_fluents_values);
	RCObject::ref(this);
	return this;
}


/* Returns an instantiation of this effect. */
const Effect& UpdateEffect::instantiation(const SubstitutionMap& subst,
                                          const TermTable& terms,
                                          const AtomSet& atoms,
                                          const ValueMap& values) const {
  return *new UpdateEffect(update().instantiation(subst, values));
}


/* Prints this object on the given stream. */
void UpdateEffect::print(std::ostream& os) const {
  os << update();
}


/* ====================================================================== */
/* ConjunctiveEffect */

/* Deletes this conjunctive effect. */
ConjunctiveEffect::~ConjunctiveEffect() {
  for (EffectList::const_iterator ei = conjuncts().begin();
       ei != conjuncts().end(); ei++) {
    destructive_deref(*ei);
  }
}


/* Adds a conjunct to this conjunctive effect. */
void ConjunctiveEffect::add_conjunct(const Effect& conjunct) {
  const ConjunctiveEffect* conj_effect =
    dynamic_cast<const ConjunctiveEffect*>(&conjunct);
  if (conj_effect != 0) {
    for (EffectList::const_iterator ei = conj_effect->conjuncts().begin();
         ei != conj_effect->conjuncts().end(); ei++) {
      conjuncts_.push_back(*ei);
      ref(*ei);
    }
    ref(&conjunct);
    destructive_deref(&conjunct);
  } else {
    conjuncts_.push_back(&conjunct);
    ref(&conjunct);
  }
}


/* Generates the list of next states (flat transitions)
   from the given state by applying this effect */
void ConjunctiveEffect::transitions(const TermTable& terms,
									const AtomSet& atoms, const ValueMap& values,
									transition_list_t& flat_transitions) const {
	flat_transitions.push_back(FlatTransition(1.0, AtomSet(), AtomSet(), UpdateSet()));

	for (EffectList::const_iterator Ieff = conjuncts_.begin() ; Ieff != conjuncts_.end() ; ++Ieff)
	{
		transition_list_t transitions_temp;
		(*Ieff)->transitions(terms, atoms, values, transitions_temp);
		transition_list_t transitions_temp_bis;

		for (transition_list_t::const_iterator Itr1 = flat_transitions.begin() ; Itr1 != flat_transitions.end() ; ++Itr1)
		{
			for (transition_list_t::const_iterator Itr2 = transitions_temp.begin() ; Itr2 != transitions_temp.end() ; ++Itr2)
			{
				transitions_temp_bis.push_back(FlatTransition());
				transitions_temp_bis.back().probability_ = (Itr1->probability_) * (Itr2->probability_);

				std::set_union(Itr1->adds_.begin(), Itr1->adds_.end(),
						Itr2->adds_.begin(), Itr2->adds_.end(),
						std::inserter(transitions_temp_bis.back().adds_, transitions_temp_bis.back().adds_.begin()));

				std::set_union(Itr1->deletes_.begin(), Itr1->deletes_.end(),
						Itr2->deletes_.begin(), Itr2->deletes_.end(),
						std::inserter(transitions_temp_bis.back().deletes_, transitions_temp_bis.back().deletes_.begin()));

				std::set_union(Itr1->updates_.begin(), Itr1->updates_.end(),
						Itr2->updates_.begin(), Itr2->updates_.end(),
						std::inserter(transitions_temp_bis.back().updates_, transitions_temp_bis.back().updates_.begin()));
			}
		}

		flat_transitions = transitions_temp_bis;
	}
}


/* Fills the provided lists with a sampled state change for this
   effect in the given state. */
void ConjunctiveEffect::state_change(AtomList& adds, AtomList& deletes,
                                     UpdateList& updates,
                                     const TermTable& terms,
                                     const AtomSet& atoms,
                                     const ValueMap& values) const {
  for (EffectList::const_iterator ei = conjuncts().begin();
       ei != conjuncts().end(); ei++) {
    (*ei)->state_change(adds, deletes, updates, terms, atoms, values);
  }
}


/* Inserts the atoms become true, and
 erases the atoms becoming false */
void ConjunctiveEffect::relaxation(const AtomSet& true_atoms_pre,
								   const AtomSet& false_atoms_pre,
								   AtomSet& true_atoms_post,
								   AtomSet& false_atoms_post) const {
	for (EffectList::const_iterator Ie = conjuncts_.begin() ; Ie != conjuncts_.end() ; ++Ie)
		(*Ie)->relaxation(true_atoms_pre, false_atoms_pre, true_atoms_post, false_atoms_post);
}


/* Sets the weights of atoms becoming true or false to minimum of
 their current weight and the given weight, and
 returns true if at least one predicate has been modified */
bool ConjunctiveEffect::hadd_relaxation(std::vector<unsigned int>& true_atoms_weights,
										std::vector<unsigned int>& false_atoms_weights,
										unsigned int weight) const {
	bool flag = false;

	for (EffectList::const_iterator Ie = conjuncts_.begin() ; Ie != conjuncts_.end() ; ++Ie)
		flag = ((*Ie)->hadd_relaxation(true_atoms_weights, false_atoms_weights, weight)) || flag;

	return flag;
}


/* Sets the weights of atoms becoming true or false to the given weight, and
 returns true if at least one predicate has been modified */
bool ConjunctiveEffect::hmax_relaxation(std::vector<unsigned int>& true_atoms_weights,
										std::vector<unsigned int>& false_atoms_weights,
										unsigned int weight) const {
	bool flag = false;

	for (EffectList::const_iterator Ie = conjuncts_.begin() ; Ie != conjuncts_.end() ; ++Ie)
		flag = ((*Ie)->hmax_relaxation(true_atoms_weights, false_atoms_weights, weight)) || flag;

	return flag;
}


/* Returns all the deterministic effects of this effect */
void ConjunctiveEffect::ao_determinization(EffectList& deterministic_effects) const {
	deterministic_effects.push_back(&Effect::EMPTY);
	RCObject::ref(&Effect::EMPTY);

	for (EffectList::const_iterator Ieff = conjuncts_.begin() ; Ieff != conjuncts_.end() ; ++Ieff)
	{
		EffectList effect_temp;
		(*Ieff)->ao_determinization(effect_temp);
		EffectList efftmp = deterministic_effects;
		deterministic_effects.clear();

		for (EffectList::const_iterator Ieffbis = effect_temp.begin() ; Ieffbis != effect_temp.end() ; ++Ieffbis)
		{
			for (EffectList::const_iterator Iefftmp = efftmp.begin() ; Iefftmp != efftmp.end() ; ++Iefftmp)
			{
				const Effect* eff = &((**Iefftmp) && (**Ieffbis));
				RCObject::ref(eff);
				deterministic_effects.push_back(eff);
			}

			RCObject::destructive_deref(*Ieffbis);
		}

		for (EffectList::const_iterator Iefftmp = efftmp.begin() ; Iefftmp != efftmp.end() ; ++Iefftmp)
			RCObject::destructive_deref(*Iefftmp);
	}
}


/* Returns all the deterministic effects of this effect */
void ConjunctiveEffect::ao_fdeterminization(EffectList& deterministic_effects, std::vector<std::pair<double, double> >& pr, const ValueMap& static_fluents_values) const {
	deterministic_effects.push_back(&Effect::EMPTY);
	RCObject::ref(&Effect::EMPTY);
	pr.push_back(std::make_pair(1.0, 0.0));

	for (EffectList::const_iterator Ieff = conjuncts_.begin() ; Ieff != conjuncts_.end() ; ++Ieff)
	{
		EffectList effect_temp;
		std::vector<std::pair<double, double> > pr_temp;
		(*Ieff)->ao_fdeterminization(effect_temp, pr_temp, static_fluents_values);
		EffectList efftmp = deterministic_effects;
		deterministic_effects.clear();
		std::vector<std::pair<double, double> > prtmp = pr;
		pr.clear();

        std::vector<std::pair<double, double> >::const_iterator Iprbis = pr_temp.begin();
		for (EffectList::const_iterator Ieffbis = effect_temp.begin() ; Ieffbis != effect_temp.end() ; ++Ieffbis)
		{
		    std::vector<std::pair<double, double> >::const_iterator Iprtmp = prtmp.begin();
			for (EffectList::const_iterator Iefftmp = efftmp.begin() ; Iefftmp != efftmp.end() ; ++Iefftmp)
			{
				const Effect* eff = &((**Iefftmp) && (**Ieffbis));
				RCObject::ref(eff);
				deterministic_effects.push_back(eff);
				pr.push_back(std::make_pair((Iprtmp->first) * (Iprbis-> first), (Iprtmp->second) + (Iprbis-> second)));
				++Iprtmp;
			}

			RCObject::destructive_deref(*Ieffbis);
			++Iprbis;
		}

		for (EffectList::const_iterator Iefftmp = efftmp.begin() ; Iefftmp != efftmp.end() ; ++Iefftmp)
			RCObject::destructive_deref(*Iefftmp);
	}
}


/* Returns the most probable outcome effect of this effect */
const Effect* ConjunctiveEffect::mpo_determinization() const {
	const Effect* eff = &Effect::EMPTY;
	RCObject::ref(eff);

	for (EffectList::const_iterator Ieff = conjuncts_.begin() ; Ieff != conjuncts_.end() ; ++Ieff)
	{
		const Effect* effect_temp = (*Ieff)->mpo_determinization();
		const Effect* efftmp = eff;
		eff = &(*efftmp && (*effect_temp));
		RCObject::ref(eff);
		RCObject::destructive_deref(efftmp);
		RCObject::destructive_deref(effect_temp);
	}

	return eff;
}


/* Returns the most probable outcome effect of this effect */
const Effect* ConjunctiveEffect::mpo_fdeterminization(double& probability, double& reward, const ValueMap& static_fluents_values) const {
	const Effect* eff = &Effect::EMPTY;
	RCObject::ref(eff);
	probability = 1.0;
	reward = 0.0;

	for (EffectList::const_iterator Ieff = conjuncts_.begin() ; Ieff != conjuncts_.end() ; ++Ieff)
	{
	    double pr, rw;
		const Effect* effect_temp = (*Ieff)->mpo_fdeterminization(pr, rw, static_fluents_values);
		const Effect* efftmp = eff;
		eff = &(*efftmp && (*effect_temp));
		RCObject::ref(eff);
		RCObject::destructive_deref(efftmp);
		RCObject::destructive_deref(effect_temp);
		probability *= pr;
		reward += rw;
	}

	return eff;
}


/* Returns an instantiation of this effect. */
const Effect& ConjunctiveEffect::instantiation(const SubstitutionMap& subst,
                                               const TermTable& terms,
                                               const AtomSet& atoms,
                                               const ValueMap& values) const {
  ConjunctiveEffect& inst_effect = *new ConjunctiveEffect();
  for (EffectList::const_iterator ei = conjuncts().begin();
       ei != conjuncts().end(); ei++) {
    inst_effect.add_conjunct((*ei)->instantiation(subst, terms,
                                                  atoms, values));
  }
  return inst_effect;
}


/* Prints this object on the given stream. */
void ConjunctiveEffect::print(std::ostream& os) const {
  os << "(and";
  for (EffectList::const_iterator ei = conjuncts().begin();
       ei != conjuncts().end(); ei++) {
    os << ' ' << **ei;
  }
  os << ")";
}


/* ====================================================================== */
/* ConditionalEffect */

/* Returns a conditional effect. */
const Effect& ConditionalEffect::make(const StateFormula& condition,
                                      const Effect& effect) {
  if (condition.tautology()) {
    return effect;
  } else if (condition.contradiction() || effect.empty()) {
    return EMPTY;
  } else {
    return *new ConditionalEffect(condition, effect);
  }
}


/* Constructs a conditional effect. */
ConditionalEffect::ConditionalEffect(const StateFormula& condition,
                                     const Effect& effect)
  : condition_(&condition), effect_(&effect) {
  ref(condition_);
  ref(effect_);
}


/* Deletes this conditional effect. */
ConditionalEffect::~ConditionalEffect() {
  destructive_deref(condition_);
  destructive_deref(effect_);
}


/* Generates the list of next states (flat transitions)
   from the given state by applying this effect */
void ConditionalEffect::transitions(const TermTable& terms,
									const AtomSet& atoms, const ValueMap& values,
									transition_list_t& flat_transitions) const {
	if (condition_->holds(terms, atoms, values))
		effect_->transitions(terms, atoms, values, flat_transitions);
	else
		flat_transitions.push_back(FlatTransition(1.0, AtomSet(), AtomSet(), UpdateSet()));
}


/* Fills the provided lists with a sampled state change for this
   effect in the given state. */
void ConditionalEffect::state_change(AtomList& adds, AtomList& deletes,
                                     UpdateList& updates,
                                     const TermTable& terms,
                                     const AtomSet& atoms,
                                     const ValueMap& values) const {
  if (condition().holds(terms, atoms, values)) {
    /* Effect condition holds. */
    effect().state_change(adds, deletes, updates, terms, atoms, values);
  }
}


/* Inserts the atoms become true, and
 erases the atoms becoming false */
void ConditionalEffect::relaxation(const AtomSet& true_atoms_pre,
								   const AtomSet& false_atoms_pre,
								   AtomSet& true_atoms_post,
								   AtomSet& false_atoms_post) const {
	if (condition_->holds(true_atoms_pre, false_atoms_pre))
		effect_->relaxation(true_atoms_pre, false_atoms_pre, true_atoms_post, false_atoms_post);
}


/* Sets the weights of atoms becoming true or false to minimum of
 their current weight and the given weight, and
 returns true if at least one predicate has been modified */
bool ConditionalEffect::hadd_relaxation(std::vector<unsigned int>& true_atoms_weights,
										std::vector<unsigned int>& false_atoms_weights,
										unsigned int weight) const {
	bool flag = false;
	unsigned int conditional_weight = condition_->hadd_holds(true_atoms_weights, false_atoms_weights);

	if (conditional_weight != std::numeric_limits<unsigned int>::max())
		flag = (effect_->hadd_relaxation(true_atoms_weights, false_atoms_weights, conditional_weight + 1)) || flag;

	return flag;
}


/* Sets the weights of atoms becoming true or false to the given weight, and
 returns true if at least one predicate has been modified */
bool ConditionalEffect::hmax_relaxation(std::vector<unsigned int>& true_atoms_weights,
										std::vector<unsigned int>& false_atoms_weights,
										unsigned int weight) const {
	bool flag = false;
	unsigned int conditional_weight = condition_->hmax_holds(true_atoms_weights, false_atoms_weights);

	if (conditional_weight != std::numeric_limits<unsigned int>::max())
		flag = (effect_->hmax_relaxation(true_atoms_weights, false_atoms_weights, conditional_weight + 1)) || flag;

	return flag;
}


/* Returns all the deterministic effects of this effect */
void ConditionalEffect::ao_determinization(EffectList& deterministic_effects) const {
	effect_->ao_determinization(deterministic_effects);

	for (EffectList::iterator Ieff = deterministic_effects.begin() ; Ieff != deterministic_effects.end() ; ++Ieff)
	{
		const Effect* efftmp = *Ieff;
		*Ieff = &ConditionalEffect::make(*condition_, *efftmp);
		RCObject::ref(*Ieff);
		RCObject::destructive_deref(efftmp);
	}
}


/* Returns all the deterministic effects of this effect */
void ConditionalEffect::ao_fdeterminization(EffectList& deterministic_effects, std::vector<std::pair<double, double> >& pr, const ValueMap& static_fluents_values) const {
	effect_->ao_fdeterminization(deterministic_effects, pr, static_fluents_values);

	for (EffectList::iterator Ieff = deterministic_effects.begin() ; Ieff != deterministic_effects.end() ; ++Ieff)
	{
		const Effect* efftmp = *Ieff;
		*Ieff = &ConditionalEffect::make(*condition_, *efftmp);
		RCObject::ref(*Ieff);
		RCObject::destructive_deref(efftmp);
	}
}


/* Returns the most probable outcome effect of this effect */
const Effect* ConditionalEffect::mpo_determinization() const {
	const Effect* efftmp = effect_->mpo_determinization();
	const Effect* deterministic_effect = &ConditionalEffect::make(*condition_, *efftmp);
	RCObject::ref(deterministic_effect);
	RCObject::destructive_deref(efftmp);
	return deterministic_effect;
}


/* Returns the most probable outcome effect of this effect */
const Effect* ConditionalEffect::mpo_fdeterminization(double& probability, double& reward, const ValueMap& static_fluents_values) const {
	const Effect* efftmp = effect_->mpo_fdeterminization(probability, reward, static_fluents_values);
	const Effect* deterministic_effect = &ConditionalEffect::make(*condition_, *efftmp);
	RCObject::ref(deterministic_effect);
	RCObject::destructive_deref(efftmp);
	return deterministic_effect;
}


/* Returns an instantiation of this effect. */
const Effect& ConditionalEffect::instantiation(const SubstitutionMap& subst,
                                               const TermTable& terms,
                                               const AtomSet& atoms,
                                               const ValueMap& values) const {
  return make(condition().instantiation(subst, terms, atoms, values, false),
              effect().instantiation(subst, terms, atoms, values));
}


/* Prints this object on the given stream. */
void ConditionalEffect::print(std::ostream& os) const {
  os << "(when " << condition() << ' ' << effect() << ")";
}


/* ====================================================================== */
/* ProbabilisticEffect */

/* Returns a probabilistic effect. */
const Effect&
ProbabilisticEffect::make(const std::vector<std::pair<Rational,
                          const Effect*> >& os) {
  for (size_t i = 0; i < os.size(); i++) {
    if (os[i].first > 0 && !os[i].second->empty()) {
      ProbabilisticEffect& peff = *new ProbabilisticEffect();
      for (; i < os.size(); i++) {
        peff.add_outcome(os[i].first, *os[i].second);
      }
      return peff;
    }
  }
  return EMPTY;
}


/* Deletes this probabilistic effect. */
ProbabilisticEffect::~ProbabilisticEffect() {
  for (EffectList::const_iterator ei = effects_.begin();
       ei != effects_.end(); ei++) {
    destructive_deref(*ei);
  }
}


/* Adds an outcome to this probabilistic effect. */
bool ProbabilisticEffect::add_outcome(const Rational& p,
                                      const Effect& effect) {
  const ProbabilisticEffect* prob_effect =
    dynamic_cast<const ProbabilisticEffect*>(&effect);
  if (prob_effect != 0) {
    size_t n = prob_effect->size();
    for (size_t i = 0; i < n; i++) {
      if (!add_outcome(p*prob_effect->probability(i),
                       prob_effect->effect(i))) {
        return false;
      }
    }
    ref(&effect);
    destructive_deref(&effect);
  } else if (p != 0) {
    effects_.push_back(&effect);
    ref(&effect);
    if (weight_sum_ == 0) {
      weights_.push_back(p.numerator());
      weight_sum_ = p.denominator();
      return true;
    } else {
      std::pair<int, int> m =
        Rational::multipliers(weight_sum_, p.denominator());
      int sum = 0;
      size_t n = size();
      for (size_t i = 0; i < n; i++) {
        sum += weights_[i] *= m.first;
      }
      weights_.push_back(p.numerator()*m.second);
      sum += p.numerator()*m.second;
      weight_sum_ *= m.first;
      return sum <= weight_sum_;
    }
  }
  return true;
}


/* Generates the list of next states (flat transitions)
   from the given state by applying this effect */
void ProbabilisticEffect::transitions(const TermTable& terms,
									  const AtomSet& atoms, const ValueMap& values,
									  transition_list_t& flat_transitions) const {
	double prob_sum = 0.0;

	for (size_t eff = 0 ; eff < (this->size()) ; eff++)
	{
		transition_list_t transitions_temp;
		this->effect(eff).transitions(terms, atoms, values, transitions_temp);
		double prob = this->probability(eff).double_value();
		prob_sum += prob;

		for (transition_list_t::const_iterator Itr = transitions_temp.begin() ; Itr != transitions_temp.end() ; ++Itr)
			flat_transitions.push_back(FlatTransition(prob * (Itr->probability_), Itr->adds_, Itr->deletes_, Itr->updates_));
	}

	if (prob_sum < 1.0)
		flat_transitions.push_back(FlatTransition(1.0 - prob_sum, AtomSet(), AtomSet(), UpdateSet()));
}


/* Fills the provided lists with a sampled state change for this
   effect in the given state. */
void ProbabilisticEffect::state_change(AtomList& adds, AtomList& deletes,
                                       UpdateList& updates,
                                       const TermTable& terms,
                                       const AtomSet& atoms,
                                       const ValueMap& values) const {
  if (size() != 0) {
    int w = int(rand()/(RAND_MAX + 1.0)*weight_sum_);
    int wtot = 0;
    size_t n = size();
    for (size_t i = 0; i < n; i++) {
      wtot += weights_[i];
      if (w < wtot) {
        effect(i).state_change(adds, deletes, updates, terms, atoms, values);
        return;
      }
    }
  }
}


/* Inserts the atoms become true, and
 erases the atoms becoming false */
void ProbabilisticEffect::relaxation(const AtomSet& true_atoms_pre,
									 const AtomSet& false_atoms_pre,
									 AtomSet& true_atoms_post,
									 AtomSet& false_atoms_post) const {
	for (unsigned int cnt = 0 ; cnt < (this->size()) ; cnt++)
		this->effect(cnt).relaxation(true_atoms_pre, false_atoms_pre, true_atoms_post, false_atoms_post);
}


/* Sets the weights of atoms becoming true or false to minimum of
 their current weight and the given weight, and
 returns true if at least one predicate has been modified */
bool ProbabilisticEffect::hadd_relaxation(std::vector<unsigned int>& true_atoms_weights,
										  std::vector<unsigned int>& false_atoms_weights,
										  unsigned int weight) const {
	bool flag = false;

	for (unsigned int cnt = 0 ; cnt < (this->size()) ; cnt++)
		flag = (this->effect(cnt).hadd_relaxation(true_atoms_weights, false_atoms_weights, weight)) || flag;

	return flag;
}


/* Sets the weights of atoms becoming true or false to the given weight, and
 returns true if at least one predicate has been modified */
bool ProbabilisticEffect::hmax_relaxation(std::vector<unsigned int>& true_atoms_weights,
										  std::vector<unsigned int>& false_atoms_weights,
										  unsigned int weight) const {
	bool flag = false;

	for (unsigned int cnt = 0 ; cnt < (this->size()) ; cnt++)
		flag = (this->effect(cnt).hmax_relaxation(true_atoms_weights, false_atoms_weights, weight)) || flag;

	return flag;
}


/* Returns all the deterministic effects of this effect */
void ProbabilisticEffect::ao_determinization(EffectList& deterministic_effects) const {
	double prob = 0.0;

	for (unsigned int cnt = 0 ; cnt < (this->size()) ; cnt++)
	{
		EffectList effect_temp;
		this->effect(cnt).ao_determinization(effect_temp);
		deterministic_effects.insert(deterministic_effects.end(), effect_temp.begin(), effect_temp.end());
		prob += this->probability(cnt).double_value();
	}

	if (prob < 1.0)
	{
		deterministic_effects.push_back(&Effect::EMPTY);
		RCObject::ref(&Effect::EMPTY);
	}
}


/* Returns all the deterministic effects of this effect */
void ProbabilisticEffect::ao_fdeterminization(EffectList& deterministic_effects, std::vector<std::pair<double, double> >& pr, const ValueMap& static_fluents_values) const {
	double prob = 0.0;

	for (unsigned int cnt = 0 ; cnt < (this->size()) ; cnt++)
	{
		EffectList effect_temp;
		std::vector<std::pair<double, double> > pr_temp;
		this->effect(cnt).ao_fdeterminization(effect_temp, pr_temp, static_fluents_values);
		deterministic_effects.insert(deterministic_effects.end(), effect_temp.begin(), effect_temp.end());
		prob += this->probability(cnt).double_value();

		for (std::vector<std::pair<double, double> >::const_iterator Ipr = pr_temp.begin() ; Ipr != pr_temp.end() ; ++Ipr)
        {
            pr.push_back(std::make_pair((this->probability(cnt).double_value()) * (Ipr->first), Ipr->second));
        }
	}

	if (prob < 1.0)
	{
		deterministic_effects.push_back(&Effect::EMPTY);
		RCObject::ref(&Effect::EMPTY);
		pr.push_back(std::make_pair(1.0 - prob, 0.0));
	}
}


/* Returns the most probable outcome effect of this effect */
const Effect* ProbabilisticEffect::mpo_determinization() const {
	double max_prob = 0.0;
	unsigned int max_outcome = 0;
	double prob_sum = 0.0;

	for (unsigned int cnt = 0 ; cnt < (this->size()) ; cnt++)
	{
		double prob = this->probability(cnt).double_value();
		prob_sum += prob;

		if (prob > max_prob)
		{
			max_prob = prob;
			max_outcome = cnt;
		}
	}

	if ((1.0 - prob_sum) > max_prob)
	{
		RCObject::ref(&Effect::EMPTY);
		return &Effect::EMPTY;
	}
	else
		return this->effect(max_outcome).mpo_determinization();
}


/* Returns the most probable outcome effect of this effect */
const Effect* ProbabilisticEffect::mpo_fdeterminization(double& probability, double& reward, const ValueMap& static_fluents_values) const {
	double max_prob = 0.0;
	unsigned int max_outcome = 0;
	double prob_sum = 0.0;

	for (unsigned int cnt = 0 ; cnt < (this->size()) ; cnt++)
	{
		double prob = this->probability(cnt).double_value();
		prob_sum += prob;

		if (prob > max_prob)
		{
			max_prob = prob;
			max_outcome = cnt;
		}
	}

	if ((1.0 - prob_sum) > max_prob)
	{
	    probability = 1.0 - prob_sum;
	    reward = 0.0;
		RCObject::ref(&Effect::EMPTY);
		return &Effect::EMPTY;
	}
	else
    {
        double pr;
        const Effect* eff = this->effect(max_outcome).mpo_fdeterminization(pr, reward, static_fluents_values);
        probability = (this->probability(max_outcome).double_value()) * pr;
        return eff;
    }
}


/* Returns an instantiation of this effect. */
const Effect&
ProbabilisticEffect::instantiation(const SubstitutionMap& subst,
                                   const TermTable& terms,
                                   const AtomSet& atoms,
                                   const ValueMap& values) const {
  ProbabilisticEffect& inst_effect = *new ProbabilisticEffect();
  size_t n = size();
  for (size_t i = 0; i < n; i++) {
    inst_effect.add_outcome(probability(i),
                            effect(i).instantiation(subst, terms,
                                                    atoms, values));
  }
  return inst_effect;
}


/* Prints this object on the given stream. */
void ProbabilisticEffect::print(std::ostream& os) const {
  if (weight_sum_ == 0) {
    os << "(and)";
  } else if (weight_sum_ == weights_.back()) {
    os << effect(0);
  } else {
    os << "(probabilistic";
    size_t n = size();
    for (size_t i = 0; i < n; i++) {
      os << ' ' << probability(i) << ' ' << effect(i);
    }
    os << ")";
  }
}


/* ====================================================================== */
/* QuantifiedEffect */

/* Returns a universally quantified effect. */
const Effect& QuantifiedEffect::make(const VariableList& parameters,
                                     const Effect& effect) {
  if (parameters.empty() || effect.empty()) {
    return effect;
  } else {
    return *new QuantifiedEffect(parameters, effect);
  }
}


/* Constructs a universally quantified effect. */
QuantifiedEffect::QuantifiedEffect(const VariableList& parameters,
                                   const Effect& effect)
  : parameters_(parameters), effect_(&effect) {
  ref(effect_);
}


/* Deletes this universally quantifed effect. */
QuantifiedEffect::~QuantifiedEffect() {
  destructive_deref(effect_);
}


/* Generates the list of next states (flat transitions)
   from the given state by applying this effect */
void QuantifiedEffect::transitions(const TermTable& terms,
								   const AtomSet& atoms, const ValueMap& values,
								   transition_list_t& flat_transitions) const {
	// Never reached since the instantiation of a quantified effect transforms it into a conjunctive effect
	throw std::logic_error("QuantifiedEffect::transitions: quantified effects should be transformed into conjunctive effects after instantiation");
}


/* Fills the provided lists with a sampled state change for this
   effect in the given state. */
void QuantifiedEffect::state_change(AtomList& adds, AtomList& deletes,
                                    UpdateList& updates,
                                    const TermTable& terms,
                                    const AtomSet& atoms,
                                    const ValueMap& values) const {
  throw std::logic_error("Quantified::state_change not implemented");
}


/* Inserts the atoms become true, and
 erases the atoms becoming false */
void QuantifiedEffect::relaxation(const AtomSet& true_atoms_pre,
								  const AtomSet& false_atoms_pre,
								  AtomSet& true_atoms_post,
								  AtomSet& false_atoms_post) const {
	throw std::logic_error("Quantified::relaxation not implemented");
}


/* Sets the weights of atoms becoming true or false to minimum of
 their current weight and the given weight, and
 returns true if at least one predicate has been modified */
bool QuantifiedEffect::hadd_relaxation(std::vector<unsigned int>& true_atoms_weights,
									   std::vector<unsigned int>& false_atoms_weights,
									   unsigned int weight) const {
	throw std::logic_error("Quantified::hadd_relaxation not implemented");
}


/* Sets the weights of atoms becoming true or false to the given weight, and
 returns true if at least one predicate has been modified */
bool QuantifiedEffect::hmax_relaxation(std::vector<unsigned int>& true_atoms_weights,
									   std::vector<unsigned int>& false_atoms_weights,
									   unsigned int weight) const {
	throw std::logic_error("Quantified::hmax_relaxation not implemented");
}


/* Returns all the deterministic effects of this effect */
void QuantifiedEffect::ao_determinization(EffectList& deterministic_effects) const {
	effect_->ao_determinization(deterministic_effects);

	for (EffectList::iterator Ieff = deterministic_effects.begin() ; Ieff != deterministic_effects.end() ; ++Ieff)
	{
		const Effect* efftmp = *Ieff;
		*Ieff = &QuantifiedEffect::make(parameters_, *efftmp);
		RCObject::ref(*Ieff);
		RCObject::destructive_deref(efftmp);
	}
}


/* Returns all the deterministic effects of this effect */
void QuantifiedEffect::ao_fdeterminization(EffectList& deterministic_effects, std::vector<std::pair<double, double> >& pr, const ValueMap& static_fluents_values) const {
	effect_->ao_fdeterminization(deterministic_effects, pr, static_fluents_values);

	for (EffectList::iterator Ieff = deterministic_effects.begin() ; Ieff != deterministic_effects.end() ; ++Ieff)
	{
		const Effect* efftmp = *Ieff;
		*Ieff = &QuantifiedEffect::make(parameters_, *efftmp);
		RCObject::ref(*Ieff);
		RCObject::destructive_deref(efftmp);
	}
}


/* Returns the most probable outcome effect of this effect */
const Effect* QuantifiedEffect::mpo_determinization() const {
	const Effect* efftmp = effect_->mpo_determinization();
	const Effect* deterministic_effect = &QuantifiedEffect::make(parameters_, *efftmp);
	RCObject::ref(deterministic_effect);
	RCObject::destructive_deref(efftmp);
	return deterministic_effect;
}


/* Returns the most probable outcome effect of this effect */
const Effect* QuantifiedEffect::mpo_fdeterminization(double& probability, double& reward, const ValueMap& static_fluents_values) const {
	const Effect* efftmp = effect_->mpo_fdeterminization(probability, reward, static_fluents_values);
	const Effect* deterministic_effect = &QuantifiedEffect::make(parameters_, *efftmp);
	RCObject::ref(deterministic_effect);
	RCObject::destructive_deref(efftmp);
	return deterministic_effect;
}


/* Returns an instantiation of this effect. */
const Effect& QuantifiedEffect::instantiation(const SubstitutionMap& subst,
                                              const TermTable& terms,
                                              const AtomSet& atoms,
                                              const ValueMap& values) const {
  int n = parameters().size();
  if (n == 0) {
    return effect().instantiation(subst, terms, atoms, values);
  } else {
    SubstitutionMap args(subst);
    std::vector<const ObjectList*> arguments(n);
    std::vector<ObjectList::const_iterator> next_arg;
    for (int i = 0; i < n; i++) {
      Type t = TermTable::type(parameters()[i]);
      arguments[i] = &terms.compatible_objects(t);
      if (arguments[i]->empty()) {
        return EMPTY;
      }
      next_arg.push_back(arguments[i]->begin());
    }
    const Effect* conj = &EMPTY;
    std::stack<const Effect*> conjuncts;
    conjuncts.push(&effect().instantiation(args, terms, atoms, values));
    ref(conjuncts.top());
    for (int i = 0; i < n; ) {
      SubstitutionMap pargs;
      pargs.insert(std::make_pair(parameters()[i], *next_arg[i]));
      const Effect& conjunct =
        conjuncts.top()->instantiation(pargs, terms, atoms, values);
      conjuncts.push(&conjunct);
      if (i + 1 == n) {
        conj = &(*conj && conjunct);
        for (int j = i; j >= 0; j--) {
          if (j < i) {
            destructive_deref(conjuncts.top());
          }
          conjuncts.pop();
          next_arg[j]++;
          if (next_arg[j] == arguments[j]->end()) {
            if (j == 0) {
              i = n;
              break;
            } else {
              next_arg[j] = arguments[j]->begin();
            }
          } else {
            i = j;
            break;
          }
        }
      } else {
        ref(conjuncts.top());
        i++;
      }
    }
    while (!conjuncts.empty()) {
      destructive_deref(conjuncts.top());
      conjuncts.pop();
    }
    return *conj;
  }
}


/* Prints this object on the given stream. */
void QuantifiedEffect::print(std::ostream& os) const {
  if (parameters().empty()) {
    os << effect();
  } else {
    VariableList::const_iterator vi = parameters().begin();
    os << "(forall (";// << *vi;
    Term(*vi).print_with_type(os);
    for (vi++; vi != parameters().end(); vi++) {
      os << ' ';// << *vi;
      Term(*vi).print_with_type(os);
    }
    os << ") " << effect() << ")";
  }
}
