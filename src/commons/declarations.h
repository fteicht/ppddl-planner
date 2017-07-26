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

#ifndef DECLARATIONS_H_
#define DECLARATIONS_H_

#include <limits>
#include <list>
#include <map>
#include <set>
#include "mdpsim/actions.h"
#include "heuristics/symbolic_actiongraph.h"
#include "heuristics/Action_Graph.h"

#define BASE_LOWER 0
#define BASE_UPPER 1

#define BASE_INFTY (std::numeric_limits<double>::max())
#define BASE_MINUS_INFTY (std::numeric_limits<double>::min())

#endif /*DECLARATIONS_H_*/
