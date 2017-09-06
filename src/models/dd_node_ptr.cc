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


#include <sstream>
#include <ppddl_planner_mdpsim/actions.h>

#include "dd_node_ptr.h"
#include "models/symbolic_mdp.h"
#include "commons/base_exceptions.h"


DdManager * dd_node_ptr::dd_manager_ = NULL;
Cudd_ReorderingType dd_node_ptr::reordering_method_ = CUDD_REORDER_NONE;


void dd_node_ptr::initialize_cudd_manager(unsigned int number_of_variables)
{
	if (dd_manager_ != NULL)
		throw BaseException("Cudd manager being used or not released", "dd_node_ptr::initialize_cudd_manager");

	dd_manager_ = Cudd_Init(number_of_variables, 0, CUDD_UNIQUE_SLOTS, CUDD_CACHE_SLOTS, 0);

	if (dd_manager_ == NULL)
		throw BaseException("Cudd manager can not be initialized", "dd_node_ptr::initialize_cudd_manager");

	if (reordering_method_ != CUDD_REORDER_NONE)
		Cudd_AutodynEnable(dd_manager_, reordering_method_);
}


void dd_node_ptr::shutdown_cudd_manager()
{
	int dead_dd_nodes_number = Cudd_CheckZeroRef(dd_manager_);
	Cudd_Quit(dd_manager_);
	dd_manager_ = NULL;
	reordering_method_ = CUDD_REORDER_NONE;

	if (dead_dd_nodes_number != 0)
	{
		std::stringstream dead_dd_nodes_number_str;
		dead_dd_nodes_number_str << dead_dd_nodes_number;
		throw BaseException(dead_dd_nodes_number_str.str() + " dead cudd nodes", "dd_node_ptr::shutdown_cudd_manager");
	}
}


DdManager * dd_node_ptr::get_cudd_manager()
{
	return dd_manager_;
}


void dd_node_ptr::set_cudd_reordering_method(Cudd_ReorderingType reordering_method)
{
	if (dd_manager_ != NULL)
	{
		if (reordering_method_ != CUDD_REORDER_NONE)
			Cudd_AutodynDisable(dd_manager_);

		if (reordering_method != CUDD_REORDER_NONE)
			Cudd_AutodynEnable(dd_manager_, reordering_method);
	}

	reordering_method_ = reordering_method;
}


Cudd_ReorderingType dd_node_ptr::get_cudd_reordering_method()
{
	return reordering_method_;
}


dd_node_ptr::dd_node_ptr()
: dd_node_(NULL), reference_(false)
{
}


dd_node_ptr::dd_node_ptr(DdNode * dd_node, bool ref)
try : dd_node_(dd_node), reference_(ref)
{
	if (dd_node == NULL)
		throw CuddException("dd_node_ is NULL", "dd_node_ptr::dd_node_ptr");

	if (ref)
		Cudd_Ref(dd_node);
}
catch(...)
{
	reference_ = false;
	throw;
}


dd_node_ptr::dd_node_ptr(dd_node_ptr const & dd_node)
{
	dd_node_ = dd_node.get();

	if (dd_node.reference_)
	{
		dd_node.reference_ = false;
		reference_ = true;
	}
	else
		reference_ = false;
}


dd_node_ptr::~dd_node_ptr()
{
	if (reference_)
		Cudd_RecursiveDeref(dd_manager_, dd_node_);
}


void dd_node_ptr::clear()
{
	if (reference_)
	{
		Cudd_RecursiveDeref(dd_manager_, dd_node_);
		reference_ = false;
		dd_node_ = NULL;
	}
}


dd_node_ptr & dd_node_ptr::operator=(dd_node_ptr const & dd_node)
{
	if (reference_)
		Cudd_RecursiveDeref(dd_manager_, dd_node_);

	dd_node_ = dd_node.get();

	if (dd_node.reference_)
	{
		dd_node.reference_ = false;
		reference_ = true;
	}
	else
		reference_ = false;

	return (*this);
}


void dd_node_ptr::copy(dd_node_ptr const & dd_node)
{
	if (reference_)
		Cudd_RecursiveDeref(dd_manager_, dd_node_);

	dd_node_ = dd_node.get();

	if (dd_node.reference_)
	{
		Cudd_Ref(dd_node_);
		reference_ = true;
	}
	else
		reference_ = false;
}


dd_node_ptr & dd_node_ptr::operator=(DdNode * dd_node)
{
	throw CuddException("unauthorized operator because of ambiguous CUDD node referencing", "dd_node_ptr::operator=");
	return (*this);
}


DdNode * dd_node_ptr::get() const
{
	return dd_node_;
}


DdNode & dd_node_ptr::operator*() const
{
	return (*dd_node_);
}


DdNode * dd_node_ptr::operator->() const
{
	return dd_node_;
}


dd_node_ptr::operator bool () const
{
	return (dd_node_ != NULL);
}


bool dd_node_ptr::operator!() const
{
	return (dd_node_ == NULL);
}


bool operator==(dd_node_ptr const & a, dd_node_ptr const & b)
{
	return (a.get() == b.get());
}


bool operator!=(dd_node_ptr const & a, dd_node_ptr const & b)
{
	return (a.get() != b.get());
}


bool operator<(dd_node_ptr const & a, dd_node_ptr const & b)
{
	return (a.get() < b.get());
}


bool operator==(dd_node_ptr const & a, DdNode * b)
{
	return (a.get() == b);
}


bool operator!=(dd_node_ptr const & a, DdNode * b)
{
	return (a.get() != b);
}


bool operator<(dd_node_ptr const & a, DdNode * b)
{
	return (a.get() < b);
}


bool operator==(DdNode * a, dd_node_ptr const & b)
{
	return (a == b.get());
}


bool operator!=(DdNode * a, dd_node_ptr const & b)
{
	return (a != b.get());
}


bool operator<(DdNode * a, dd_node_ptr const & b)
{
	return (a < b.get());
}


// CLASS dd_node_ptr_container


template <class Tdd_node_ptr_container>
void dd_node_ptr_container<Tdd_node_ptr_container>::copy(Tdd_node_ptr_container const & other)
{
	resize(other.size());
	typename Tdd_node_ptr_container::iterator Iself = this->begin();

	for (typename Tdd_node_ptr_container::const_iterator Iother = other.begin() ; Iother != other.end() ; ++Iother)
	{
		Iself->copy(*Iother);
		++Iself;
	}
}


template <class Tdd_node_ptr_container>
dd_node_ptr_container<Tdd_node_ptr_container>::operator bool () const
{
	if (this->empty())
		return false;

	for (typename Tdd_node_ptr_container::const_iterator It = this->begin() ; It != this->end() ; ++It)
	{
		if (!(*It))
			return false;
	}

	return true;
}


template <class Tdd_node_ptr_container>
bool dd_node_ptr_container<Tdd_node_ptr_container>::operator!() const
{
	if (this->empty())
		return true;

	for (typename Tdd_node_ptr_container::const_iterator It = this->begin() ; It != this->end() ; ++It)
	{
		if (*It)
			return true;
	}

	return false;
}

// Explicit template instantiations
// no need for the moment




// CLASS dd_node_ptr_associative_container


template <class Tdd_node_ptr_associative_container>
void dd_node_ptr_associative_container<Tdd_node_ptr_associative_container>::copy(Tdd_node_ptr_associative_container const & other)
{
	this->clear();

	for (typename Tdd_node_ptr_associative_container::const_iterator Iother = other.begin() ; Iother != other.end() ; ++Iother)
	{
		typename Tdd_node_ptr_associative_container::mapped_type new_element;
		new_element.copy(Iother->second);
		this->insert(std::make_pair(Iother->first, new_element));
	}
}


template <class Tdd_node_ptr_associative_container>
dd_node_ptr_associative_container<Tdd_node_ptr_associative_container>::operator bool () const
{
	if (this->empty())
		return false;

	for (typename Tdd_node_ptr_associative_container::const_iterator It = this->begin() ; It != this->end() ; ++It)
	{
		if (!(It->second))
			return false;
	}

	return true;
}


template <class Tdd_node_ptr_associative_container>
bool dd_node_ptr_associative_container<Tdd_node_ptr_associative_container>::operator!() const
{
	if (this->empty())
		return true;

	for (typename Tdd_node_ptr_associative_container::const_iterator It = this->begin() ; It != this->end() ; ++It)
	{
		if (It->second)
			return true;
	}

	return false;
}

// Explicit template instantiations
template class dd_node_ptr_associative_container<std::map<const Action*, SymbolicTransition> >;




// CLASS dd_node_ptr_pair


void dd_node_ptr_pair::copy(dd_node_ptr_pair const & other)
{
	this->first.copy(other.first);
	this->second.copy(other.second);
}


dd_node_ptr_pair::operator bool () const
{
	return ((this->first) && (this->second));
}


bool dd_node_ptr_pair::operator!() const
{
	return ((!(this->first)) || (!(this->second)));
}
