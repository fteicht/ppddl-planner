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

#include <list>
#include "Equivalence_Class.h"
#include "models/graph_mdp.h"

// Constructor.
template<class Tstate>
EquivalenceClass<Tstate>::EquivalenceClass() {
	states.clear(); 
}

// Destructor.
template<class Tstate>
EquivalenceClass<Tstate>::~EquivalenceClass() {
	states.clear();
}

// Methods.
template<class Tstate>
const typename EquivalenceClass<Tstate>::states_set_t& EquivalenceClass<Tstate>::get_states() const {
	return states;
}

template<class Tstate>
void EquivalenceClass<Tstate>::add_state(Tstate& state) {
	states.insert(&state);
}

template<class Tstate>
void EquivalenceClass<Tstate>::remove_state(Tstate& state) {
	states.erase(&state);
}

template<class Tstate>
bool EquivalenceClass<Tstate>::contains (const EquivalenceClass<Tstate>& target) const {
	const states_set_t& target_states = target.get_states();
	for(typename states_set_t::const_iterator s=target_states.begin(); s != target_states.end(); ++s) {
		if (states.find(*s) == states.end())
			return false;
	}
	return true;
}

// Explicit template instantiations
INSTANTIATE_STATE_TEMPLATE(EquivalenceClass)
