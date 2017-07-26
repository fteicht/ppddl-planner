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

#include "particle_algorithm.h"

template <class Tstate>
ParticleAlgorithm<Tstate>::ParticleAlgorithm(const Problem& pb, double epsilon, double discount_factor,
		double probabilistic_threshold, unsigned int nb_of_particles, unsigned int trajectories_max_depth)
try : BaseAlgorithm(pb, epsilon, discount_factor),	trajectories_max_depth_(trajectories_max_depth)
{
	set_probabilistic_threshold(probabilistic_threshold);
	set_nb_of_particles(nb_of_particles);
}
catch (BaseException& error)
{
	error.push_function_backtrace("ParticleAlgorithm::ParticleAlgorithm");
	throw;
}


template <class Tstate>
void ParticleAlgorithm<Tstate>::set_probabilistic_threshold(double probabilistic_threshold)
{
	if ((probabilistic_threshold < 0.0) || (probabilistic_threshold > 1.0))
		throw BaseException("probabilistic threshold must be in range [0;1]", "ParticleAlgorithm::set_probabilistic_threshold");
	
	probabilistic_threshold_ = probabilistic_threshold;
}


#define INSTANTIATE_PARTICLE_ALGORITHM_METHODS(StateT) \
	template <> ParticleAlgorithm<StateT*>::Particle::Particle() :current_state_(NULL) {} \
	template <> void ParticleAlgorithm<StateT*>::Particle::set_state(StateT*& st) {current_state_ = st;} \
	template <> template <> void ParticleAlgorithm<StateT*>::Particle::change_state(GraphMDP<StateT>& mdp) {current_state_ = mdp.random_transition(*current_state_, current_state_->get_policy()).first;}

INSTANTIATE_STATE_TEMPLATE_MACRO(INSTANTIATE_PARTICLE_ALGORITHM_METHODS)

template <> ParticleAlgorithm<dd_node_ptr>::Particle::Particle() {current_state_ = dd_node_ptr(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));}
template <> void ParticleAlgorithm<dd_node_ptr>::Particle::set_state(dd_node_ptr& st) {current_state_.copy(st);}
template <> template <> void ParticleAlgorithm<dd_node_ptr>::Particle::change_state(SymbolicMDP& mdp) {current_state_ = mdp.random_transition(current_state_, mdp.get_policy(current_state_)).first;}

INSTANTIATE_STATE_TEMPLATE_PTR(ParticleAlgorithm)
template class ParticleAlgorithm<dd_node_ptr>;
