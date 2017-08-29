AUTOMAKE_OPTIONS = subdir-objects

#AM_CXXFLAGS =\
#	 -Wall\
#	 -g -ggdb -O3

bin_PROGRAMS = plannerclient plannertest

plannerclient_LDADD = -lmdpsim -lm ${CUDD_LDFLAGS}
plannerclient_CPPFLAGS = -Isrc

plannertest_LDADD = -lmdpsim -lm ${CUDD_LDFLAGS}
plannertest_CPPFLAGS = -Isrc

lib_LTLIBRARIES = libppddl_planner.la

if BOOSTPYTHON
libppddl_planner_la_LDFLAGS = -lmdpsim -lm ${CUDD_LDFLAGS} ${BOOST_LDFLAGS} -l${BOOST_PYTHON_LIB}
libppddl_planner_la_CPPFLAGS = -Isrc ${BOOST_CPPFLAGS} ${PYTHON_CPPFLAGS}
else
libppddl_planner_la_LDFLAGS = -lmdpsim -lm ${CUDD_LDFLAGS}
libppddl_planner_la_CPPFLAGS = -Isrc
endif

COMMON_SOURCES = \
	src/commons/base_exceptions.cc\
	src/client/ippc_client.cc\
	src/client/string_utils.cc\
	src/client/system_utils.cc\
	src/planners/base_planner.cc\
	src/planners/deterministic_planner.cc\
	src/planners/ff_planner.cc\
	src/models/transitions_blackbox.cc\
	src/models/graph_mdp.cc\
	src/models/symbolic_mdp.cc\
	src/models/dd_node_ptr.cc\
	src/models/base_determinization.cc\
	src/models/mpo_determinization.cc\
	src/models/ao_determinization.cc\
	src/models/relaxed_mdp_graph.cc\
	src/heuristics/base_heuristic.cc\
	src/heuristics/determinization_heuristic.cc\
	src/heuristics/step_distance.cc\
	src/heuristics/relaxed_step_distance.cc\
	src/heuristics/relaxed_rewards.cc\
	src/heuristics/relaxed_planning_graph.cc\
	src/heuristics/Action_Graph.cc\
	src/heuristics/Equivalence_Class.cc\
	src/heuristics/graph_bounds.cc\
	src/heuristics/symbolic_actiongraph.cc\
	src/heuristics/symbolic_bounds.cc\
	src/heuristics/hadd.cc\
	src/heuristics/hmax.cc\
	src/heuristics/hdet.cc\
	src/algorithms/base_algorithm.cc\
	src/algorithms/graph_vi.cc\
	src/algorithms/symbolic_vi.cc\
	src/algorithms/heuristic_algorithm.cc\
	src/algorithms/particle_algorithm.cc\
	src/algorithms/determinization_algorithm.cc\
	src/algorithms/graph_fsp.cc\
	src/algorithms/graph_fsp_star.cc\
	src/algorithms/graph_bfdt.cc\
	src/algorithms/graph_trfsp_star.cc\
	src/algorithms/graph_psp_star.cc\
	src/algorithms/graph_lao.cc\
	src/algorithms/graph_improved_lao.cc\
	src/algorithms/graph_rtdp.cc\
	src/algorithms/graph_lrtdp.cc\
	src/algorithms/graph_ldfs.cc\
	src/algorithms/graph_fdp.cc\
	src/algorithms/graph_robust_determinization_reward.cc\
	src/algorithms/graph_robust_determinization_goal.cc\
	src/algorithms/graph_optimal_determinization_goal.cc\
	src/algorithms/graph_gce.cc\
	src/algorithms/graph_gco.cc\
	src/algorithms/graph_mvpf.cc\
	src/algorithms/graph_mvho.cc\
	src/algorithms/graph_mvpi.cc\
	src/algorithms/graph_mvas.cc\
	src/algorithms/graph_mvssp.cc\
	src/algorithms/symbolic_fsp.cc\
	src/algorithms/symbolic_fsp_star.cc\
	src/algorithms/symbolic_lao.cc\
	src/algorithms/symbolic_rtdp.cc\
	src/algorithms/symbolic_gco.cc\
	src/algorithms/graph_search.cc\
	src/algorithms/symbolic_search.cc

COMMON_INCLUDES = \
	config.h\
	src/commons/declarations.h\
	src/commons/base_exceptions.h\
	src/client/ippc_client.h\
	src/client/string_utils.h\
	src/client/system_utils.h\
	src/planners/base_planner.h\
	src/planners/deterministic_planner.h\
	src/planners/ff_planner.h\
	src/models/transitions_blackbox.h\
	src/models/base_mdp.h\
	src/models/graph_mdp.h\
	src/models/symbolic_mdp.h\
	src/models/dd_node_ptr.h\
	src/models/base_determinization.h\
	src/models/mpo_determinization.h\
	src/models/ao_determinization.h\
	src/models/relaxed_mdp_graph.h\
	src/heuristics/base_heuristic.h\
	src/heuristics/determinization_heuristic.h\
	src/heuristics/zero_heuristic.h\
	src/heuristics/step_distance.h\
	src/heuristics/relaxed_step_distance.h\
	src/heuristics/relaxed_rewards.h\
	src/heuristics/relaxed_planning_graph.h\
	src/heuristics/Action_Graph.h\
	src/heuristics/Equivalence_Class.h\
	src/heuristics/graph_bounds.h\
	src/heuristics/symbolic_actiongraph.h\
	src/heuristics/symbolic_bounds.h\
	src/heuristics/hadd.h\
	src/heuristics/hmax.h\
	src/heuristics/hdet.h\
	src/algorithms/base_algorithm.h\
	src/algorithms/graph_vi.h\
	src/algorithms/symbolic_vi.h\
	src/algorithms/heuristic_algorithm.h\
	src/algorithms/particle_algorithm.h\
	src/algorithms/determinization_algorithm.h\
	src/algorithms/graph_fsp.h\
	src/algorithms/graph_fsp_star.h\
	src/algorithms/graph_bfdt.h\
	src/algorithms/graph_trfsp_star.h\
	src/algorithms/graph_psp_star.h\
	src/algorithms/graph_lao.h\
	src/algorithms/graph_improved_lao.h\
	src/algorithms/graph_rtdp.h\
	src/algorithms/graph_lrtdp.h\
	src/algorithms/graph_ldfs.h\
	src/algorithms/graph_fdp.h\
	src/algorithms/graph_robust_determinization_reward.h\
	src/algorithms/graph_robust_determinization_goal.h\
	src/algorithms/graph_optimal_determinization_goal.h\
	src/algorithms/graph_gce.h\
	src/algorithms/graph_gco.h\
	src/algorithms/graph_mvpf.h\
	src/algorithms/graph_mvho.h\
	src/algorithms/graph_mvpi.h\
	src/algorithms/graph_mvas.h\
	src/algorithms/graph_mvssp.h\
	src/algorithms/symbolic_fsp.h\
	src/algorithms/symbolic_fsp_star.h\
	src/algorithms/symbolic_lao.h\
	src/algorithms/symbolic_rtdp.h\
	src/algorithms/symbolic_gco.h\
	src/algorithms/graph_search.h\
	src/algorithms/symbolic_search.h


plannerclient_SOURCES = \
	$(COMMON_SOURCES)\
    $(COMMON_INCLUDES)\
	src/planners/complete_planner.h\
	src/planners/complete_planner.cc\
	src/planners/rddl_planner.h\
	src/planners/rddl_planner.cc\
	src/planners/anytime_planner.h\
	src/planners/anytime_planner.cc\
	src/main_plannerclient.cc

plannertest_SOURCES = \
	$(COMMON_SOURCES)\
    $(COMMON_INCLUDES)\
	src/planners/test_planner.h\
	src/planners/test_planner.cc\
	src/main_plannertest.cc

LIBRARY_INCLUDES = src/planners/raw_planner.h
libppddl_planner_la_SOURCES = \
    $(COMMON_SOURCES)\
    $(COMMON_INCLUDES)\
    $(LIBRARY_INCLUDES)\
    src/planners/raw_planner.cc

include_HEADERS = $(COMMON_INCLUDES) $(LIBRARY_INCLUDES)

dist_bin_SCRIPTS = \
	planner_check_mem.sh\
	fsp-rbh\
	fsp-rdh\
	rff-bg\
	rff-pg\
	team-1-script.sh

EXTRA_DIST = autogen.sh
