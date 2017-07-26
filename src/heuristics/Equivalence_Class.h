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
#include <set>

template<class Tstate>
class EquivalenceClass {
public:
  typedef typename std::set<Tstate*> states_set_t;


    private:
     	states_set_t states;
     	
    public:
    	EquivalenceClass();
    	~EquivalenceClass();

    	void add_state(Tstate& state);
    	void remove_state(Tstate& state);
    	inline const states_set_t& get_states() const;
    	
    	bool contains(const EquivalenceClass<Tstate>& target) const;
    	
};
