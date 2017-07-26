/* 
   ppddl-planner - client for IPPC'08

   Copyright (C) 2012 Florent Teichteil-Koenigsbuch and Guillaume Infantes and Ugur Kuter

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

#include "graph_gce.h"
#include <cmath>
#include <limits>

template <class Tstate>
GraphGCE<Tstate>::GraphGCE(const Problem& pb, double epsilon, GraphAlgorithm<Tstate>& algorithm)
try : BaseAlgorithm(pb, epsilon, 1.0), GoalStatisticsAlgorithm(pb, epsilon),
      GraphAlgorithm<Tstate>(pb, epsilon, 1.0), algorithm_(algorithm)
{
    this->tb_->set_metric(TransitionsBlackbox::METRIC_REWARD);
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphGCE::GraphGCE");
	throw;
}


template <class Tstate>
double GraphGCE<Tstate>::goal_probability(const PddlState& st) const
{
    typename GraphMDP<Tstate>::states_map_t::const_iterator Ist = algorithm_.mdp().get_states().find(&st);
    
    if (Ist != algorithm_.mdp().get_states().end())
    {
        typename MyValuesMap::const_iterator Ival = values_map_.find(Ist->second);
        
        if (Ival != values_map_.end())
            return Ival->second.current_goal_probability_value_;
        else
            return -1.0;
    }
    else
        return -1.0;
}


template <class Tstate>
double GraphGCE<Tstate>::goal_cost(const PddlState& st) const
{
    typename GraphMDP<Tstate>::states_map_t::const_iterator Ist = algorithm_.mdp().get_states().find(&st);
    
    if (Ist != algorithm_.mdp().get_states().end())
    {
        typename MyValuesMap::const_iterator Ival = values_map_.find(Ist->second);
        
        if (Ival != values_map_.end())
            return Ival->second.current_goal_cost_value_;
        else
            return std::numeric_limits<double>::max();
    }
    else
        return std::numeric_limits<double>::max();
}


template <class Tstate>
bool GraphGCE<Tstate>::policy_defined(const PddlState& st)
{
    typename GraphMDP<Tstate>::states_map_t::const_iterator Ist = algorithm_.mdp().get_states().find(&st);
    
    if (Ist != algorithm_.mdp().get_states().end())
        return Ist->second->policy_defined();
    else
        return false;
}


template <class Tstate>
void GraphGCE<Tstate>::solve_initialize(const PddlState& st)
{
    values_map_.clear();
    
    for (typename GraphMDP<Tstate>::states_map_t::const_iterator Ist = algorithm_.mdp().get_states().begin() ; Ist != algorithm_.mdp().get_states().end() ; ++Ist)
    {
        MyValues& sval = values_map_.insert(std::make_pair(Ist->second, MyValues())).first->second;
        
        if (algorithm_.mdp().goal(*(Ist->second)))
            sval.current_goal_probability_value_ = 1.0;
    }
}


template <class Tstate>
void GraphGCE<Tstate>::solve_progress()
{
	// record values
	
	for (typename MyValuesMap::iterator Ival = values_map_.begin() ; Ival != values_map_.end() ; ++Ival)
	{
        Ival->second.previous_goal_probability_value_ = Ival->second.current_goal_probability_value_;
        Ival->second.previous_goal_cost_value_ = Ival->second.current_goal_cost_value_;
	}
	
	// update values
	this->bellman_error_ = 0.0;
    
	for (typename GraphMDP<Tstate>::states_map_t::const_iterator Ist = algorithm_.mdp().get_states().begin() ; Ist != algorithm_.mdp().get_states().end() ; ++Ist)
	{
        MyValues& sval = values_map_.find(Ist->second)->second;
        
        if (algorithm_.mdp().goal(*(Ist->second)))
        {
            sval.current_goal_probability_value_ = 1.0;
            sval.current_goal_cost_value_ = 0.0;
        }
        else
        {
            if (!(Ist->second->policy_defined()))
            {
                sval.current_goal_probability_value_ = 0.0;
                sval.current_goal_cost_value_ = 0.0;
            }
            else
            {
                const Action& action = Ist->second->get_policy();
                const typename GraphMDP<Tstate>::actions_map_t* tr = _transitions_(*(Ist->second));
                typename GraphMDP<Tstate>::actions_map_t::const_iterator Iac = tr->find(&action);
                sval.current_goal_probability_value_ = 0.0;
                sval.current_goal_cost_value_ = 0.0;
                
                for (typename GraphMDP<Tstate>::transition_list_t::const_iterator Ieff = Iac->second.begin() ; Ieff != Iac->second.end() ; ++Ieff)
                {
                    MyValues& nsval = values_map_.find(&((*Ieff)->get_outcome()))->second;
                    sval.current_goal_probability_value_ += ((*Ieff)->get_probability()) * nsval.previous_goal_probability_value_;
                    sval.current_goal_cost_value_ += ((*Ieff)->get_probability()) * nsval.previous_goal_probability_value_ * (((*Ieff)->get_reward()) + nsval.previous_goal_cost_value_);
                }
                
                if (sval.current_goal_probability_value_ > 0.0)
                    sval.current_goal_cost_value_ /= sval.current_goal_probability_value_;
                else
                    sval.current_goal_cost_value_ = 0.0;
                
                _delete_transitions_(tr);
            }
        }
        
        double mval = std::max(std::fabs(sval.current_goal_probability_value_ - sval.previous_goal_probability_value_),
                               std::fabs(sval.current_goal_cost_value_ - sval.previous_goal_cost_value_));
		this->bellman_error_ = std::max(this->bellman_error_, mval);
	}
}


template <class Tstate>
bool GraphGCE<Tstate>::has_converged()
{
	if ((this->bellman_error_) < (this->epsilon_))
		return true;
	else
		return false;
}


#define INSTANTIATE_GRAPH_GCE_INTERNAL_NT(StateT) \
template <> \
const GraphMDP<StateT>::actions_map_t* GraphGCE<StateT>::_transitions_(StateT& st) \
{ \
	return algorithm_._mdp_().transitions(st); \
} \
template <>  \
void GraphGCE<StateT>::_delete_transitions_(const GraphMDP<StateT>::actions_map_t* tr) \
{ \
	delete tr; \
}

INSTANTIATE_GRAPH_GCE_INTERNAL_NT(GraphState)
INSTANTIATE_GRAPH_GCE_INTERNAL_NT(GraphStateProbability)
INSTANTIATE_GRAPH_GCE_INTERNAL_NT(GraphStateVariance)
INSTANTIATE_GRAPH_GCE_INTERNAL_NT(GraphStateSolveBit)
INSTANTIATE_GRAPH_GCE_INTERNAL_NT(GraphStateConnection)
INSTANTIATE_GRAPH_GCE_INTERNAL_NT(GraphStateSolveBitConnection)
INSTANTIATE_GRAPH_GCE_INTERNAL_NT(GraphStateWA<GraphState>)
INSTANTIATE_GRAPH_GCE_INTERNAL_NT(GraphStateWA<GraphStateProbability>)
INSTANTIATE_GRAPH_GCE_INTERNAL_NT(GraphStateWA<GraphStateVariance>)
INSTANTIATE_GRAPH_GCE_INTERNAL_NT(GraphStateWA<GraphStateSolveBit>)
INSTANTIATE_GRAPH_GCE_INTERNAL_NT(GraphStateWA<GraphStateConnection>)
INSTANTIATE_GRAPH_GCE_INTERNAL_NT(GraphStateWA<GraphStateSolveBitConnection>)


#define INSTANTIATE_GRAPH_GCE_INTERNAL_WT(StateT) \
template <> \
const GraphMDP<StateT>::actions_map_t* GraphGCE<StateT>::_transitions_(StateT& st) \
{ \
	return &(st.get_transitions()); \
} \
template <>  \
void GraphGCE<StateT>::_delete_transitions_(const GraphMDP<StateT>::actions_map_t* tr) \
{ \
}

INSTANTIATE_GRAPH_GCE_INTERNAL_WT(GraphStateWT<GraphState>)
INSTANTIATE_GRAPH_GCE_INTERNAL_WT(GraphStateWT<GraphStateProbability>)
INSTANTIATE_GRAPH_GCE_INTERNAL_WT(GraphStateWT<GraphStateVariance>)
INSTANTIATE_GRAPH_GCE_INTERNAL_WT(GraphStateWT<GraphStateSolveBit>)
INSTANTIATE_GRAPH_GCE_INTERNAL_WT(GraphStateWT<GraphStateConnection>)
INSTANTIATE_GRAPH_GCE_INTERNAL_WT(GraphStateWT<GraphStateSolveBitConnection>)
INSTANTIATE_GRAPH_GCE_INTERNAL_WT(GraphStateWTA<GraphState>)
INSTANTIATE_GRAPH_GCE_INTERNAL_WT(GraphStateWTA<GraphStateProbability>)
INSTANTIATE_GRAPH_GCE_INTERNAL_WT(GraphStateWTA<GraphStateVariance>)
INSTANTIATE_GRAPH_GCE_INTERNAL_WT(GraphStateWTA<GraphStateSolveBit>)
INSTANTIATE_GRAPH_GCE_INTERNAL_WT(GraphStateWTA<GraphStateConnection>)
INSTANTIATE_GRAPH_GCE_INTERNAL_WT(GraphStateWTA<GraphStateSolveBitConnection>)


// Explicit instantiations
INSTANTIATE_STATE_TEMPLATE(GraphGCE)
