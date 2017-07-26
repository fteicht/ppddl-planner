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

#ifndef BASE_EXCEPTIONS_H_
#define BASE_EXCEPTIONS_H_

#include <exception>
#include <string>


class BaseException : public std::exception
{
protected :
	std::string error_message_;
	mutable std::string what_message_;
	std::string function_backtrace_;

public :
	BaseException(std::string const & error_message, std::string const & throwing_function = "");
	virtual ~BaseException() throw();

	virtual const char * what() const throw();
	virtual void push_function_backtrace(std::string const & throwing_function);
	virtual void clear_function_backtrace();
};


class CuddException : public BaseException
{
public :
	CuddException(std::string const & error_message = "", std::string const & throwing_function = "");
	virtual ~CuddException() throw();
};


class TimeoutException : public BaseException
{
public :
	TimeoutException(std::string const & error_message, std::string const & throwing_function = "");
	virtual ~TimeoutException() throw();
};


class IncompatibilityException : public BaseException
{
public :
	IncompatibilityException(std::string const & error_message, std::string const & throwing_function = "");
	virtual ~IncompatibilityException() throw();
};

#endif /*BASE_EXCEPTIONS_H_*/
