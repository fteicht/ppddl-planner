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


#ifndef DD_NODE_PTR_H_
#define DD_NODE_PTR_H_

#include <cudd.h> // redefines SIZEOF_VOID_P

class dd_node_ptr
{
private :
	DdNode * dd_node_;
	mutable bool reference_;

	static DdManager * dd_manager_;
	static Cudd_ReorderingType reordering_method_;

	dd_node_ptr & operator=(DdNode * dd_node);

public :
	dd_node_ptr();
	dd_node_ptr(DdNode * dd_node, bool ref = true); // throws
	dd_node_ptr(dd_node_ptr const & dd_node);
	virtual ~dd_node_ptr();

	void clear();

	dd_node_ptr & operator=(dd_node_ptr const & dd_node); // propagate reference (dd_node must be reinitialized)
	void copy(dd_node_ptr const & dd_node); // new reference (dd_node is still valid)
	DdNode * get() const;
	DdNode & operator*() const;
	DdNode * operator->() const;
	operator bool () const;
	bool operator!() const;

	static void initialize_cudd_manager(unsigned int number_of_variables = 0); // throws
	static void shutdown_cudd_manager(); // throws
	static DdManager * get_cudd_manager();

	static void set_cudd_reordering_method(Cudd_ReorderingType reordering_method);
	static Cudd_ReorderingType get_cudd_reordering_method();
};


bool operator==(dd_node_ptr const & a, dd_node_ptr const & b);
bool operator!=(dd_node_ptr const & a, dd_node_ptr const & b);
bool operator<(dd_node_ptr const & a, dd_node_ptr const & b);
bool operator==(dd_node_ptr const & a, DdNode * b);
bool operator!=(dd_node_ptr const & a, DdNode * b);
bool operator<(dd_node_ptr const & a, DdNode * b);
bool operator==(DdNode * a, dd_node_ptr const & b);
bool operator!=(DdNode * a, dd_node_ptr const & b);
bool operator<(DdNode * a, dd_node_ptr const & b);


template <class Tdd_node_ptr_stl>
class dd_node_ptr_stl : public Tdd_node_ptr_stl
{
public :
	virtual ~dd_node_ptr_stl() {}
	virtual void copy(Tdd_node_ptr_stl const & other) =0;
	virtual operator bool () const =0;
	virtual bool operator!() const =0;
};


template <class Tdd_node_ptr_container>
class dd_node_ptr_container : public dd_node_ptr_stl<Tdd_node_ptr_container>
{
public :
	virtual ~dd_node_ptr_container() {}
	virtual void copy(Tdd_node_ptr_container const & other);
	virtual operator bool () const;
	virtual bool operator!() const;
};


template <class Tdd_node_ptr_associative_container>
class dd_node_ptr_associative_container : public dd_node_ptr_stl<Tdd_node_ptr_associative_container>
{
public :
	virtual ~dd_node_ptr_associative_container() {}
	virtual void copy(Tdd_node_ptr_associative_container const & other);
	virtual operator bool () const;
	virtual bool operator!() const;
};


class dd_node_ptr_pair : public std::pair<dd_node_ptr, dd_node_ptr>
{
public :
	virtual ~dd_node_ptr_pair() {}
	virtual void copy(dd_node_ptr_pair const & other);
	virtual operator bool () const;
	virtual bool operator!() const;
};

#endif /*DD_NODE_PTR_H_*/
