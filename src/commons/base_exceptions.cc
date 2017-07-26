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

#include "base_exceptions.h"
#include "models/dd_node_ptr.h"

// Class BaseException

BaseException::BaseException(std::string const & error_message, std::string const & throwing_function)
: error_message_(error_message)
{
	if (!throwing_function.empty())
		function_backtrace_ = std::string("    from '") + throwing_function + "':\n";
}


BaseException::~BaseException() throw()
{
}


const char * BaseException::what() const throw()
{
	what_message_ = std::string("plannerclient exception:\n") + function_backtrace_ + "        " + error_message_;
	return what_message_.c_str();
}


void BaseException::push_function_backtrace(std::string const & throwing_function)
{
	function_backtrace_.insert(0, std::string("    from '") + throwing_function + "':\n");
}


void BaseException::clear_function_backtrace()
{
	function_backtrace_.clear();
}




// CLASS CuddException

CuddException::CuddException(std::string const & error_message, std::string const & throwing_function)
: BaseException("CUDD exception: ", throwing_function)
{
	std::string cudd_error_string;

	switch (Cudd_ReadErrorCode(dd_node_ptr::get_cudd_manager()))
	{
		case CUDD_NO_ERROR :
			error_message_ += "'NO ERROR'";
			break;

		case CUDD_MEMORY_OUT :
			error_message_ += "'MEMORY OUT'";
			break;

		case CUDD_TOO_MANY_NODES :
			error_message_ += "'TOO MANY NODES'";
			break;

		case CUDD_MAX_MEM_EXCEEDED :
			error_message_ += "'MAX MEM EXCEEDED'";
			break;

		case CUDD_INVALID_ARG :
			error_message_ += "'INVALID ARG'";
			break;

		case CUDD_INTERNAL_ERROR :
			error_message_ += "'INTERNAL ERROR'";
			break;

		default :
			error_message_ += "";
	}

	if (error_message != std::string(""))
		error_message_ += ": " + error_message;

	Cudd_ClearErrorCode(dd_node_ptr::get_cudd_manager());
}


CuddException::~CuddException() throw()
{
}




// CLASS TimeoutException

TimeoutException::TimeoutException(std::string const & error_message, std::string const & throwing_function)
: BaseException("Timeout exception: ", throwing_function)
{
	error_message_ += error_message;
}


TimeoutException::~TimeoutException() throw()
{
}




// CLASS IncompatibilityException

IncompatibilityException::IncompatibilityException(std::string const & error_message, std::string const & throwing_function)
: BaseException("Incompatibility exception: ", throwing_function)
{
	error_message_ += error_message;
}


IncompatibilityException::~IncompatibilityException() throw()
{
}
