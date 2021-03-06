dnl Process this file with autoconf to produce a configure script.

dnl Every other copy of the package version number gets its value from here
AC_INIT([ppddl-planner], [1.0.0])

AC_CONFIG_SRCDIR([configure.ac])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

AM_INIT_AUTOMAKE

AC_SUBST(VERSION)

ISODATE=`date +%Y-%m-%d`
AC_SUBST(ISODATE)

AC_CANONICAL_HOST

AC_DEFINE(PACKAGE_BUGREPORT, "florent.teichteil@gmail.com", [bug report e-mail])

LT_INIT

#CXXFLAGS=""
#AC_SUBST(CXXFLAGS)

dnl Checks for programs.
AC_PROG_INSTALL
AC_PROG_CC
AC_PROG_CXX

# Set the language.
AC_LANG(C++)

# Checks for libraries.
AC_SEARCH_LIBS(gettext, intl)
AC_SEARCH_LIBS(accept, socket)
AC_SEARCH_LIBS(gethostbyname, nsl)
AC_CHECK_LIB(pthread, pthread_create, PTHREADLIB=-lpthread)
AC_SUBST(PTHREADLIB)

# Checks for header files.
AC_HEADER_STDC
AC_CHECK_HEADERS([arpa/inet.h libintl.h netdb.h netinet/in.h stddef.h stdlib.h string.h strings.h sys/socket.h unistd.h sys/time.h sstream])

# Checks for typedefs, structures, and compiler characteristics.
AC_HEADER_STDBOOL
AC_C_CONST
AC_C_INLINE
AC_TYPE_SIZE_T
AC_CHECK_TYPES(socklen_t, , , [#include <sys/socket.h>])

# Checks for library functions.
AC_CHECK_FUNCS([gethostbyname memset socket strerror])
AC_CHECK_FUNCS(getopt_long, , [AC_LIBOBJ(getopt) AC_LIBOBJ(getopt1)])

##################################################
# Checks for boost
##################################################
#dnl Check for boost.

if test "$PYTHON_VERSION" != "none"; then
    AX_BOOST_BASE
    AX_PYTHON_DEVEL
    AX_BOOST_PYTHON
    AC_SUBST(BOOST_CPPFLAGS)
    AC_SUBST(BOOST_LDFLAGS)
    AC_SUBST(BOOST_PYTHON_LIB)
    if test "$ac_cv_boost_python" != "yes"; then
        AC_MSG_ERROR("Python or Boost.Python not found on your system!")
    fi
else
    AC_MSG_WARN("Python bindings won't be compiled.")
fi
AM_CONDITIONAL([BOOSTPYTHON], [test "$ac_cv_boost_python" = "yes"])


##################################################
# Checks for cudd
##################################################
dnl Check for cudd.

AC_ARG_WITH(cudd-prefix,
                        [  --with-cudd-prefix=DIR		cudd is installed in DIR],
			[with_cudd_prefix=$withval], )

CUDD_CPPFLAGS="-I$with_cudd_prefix/include"
CPPFLAGS="$CPPFLAGS $CUDD_CPPFLAGS"

CUDD_LDFLAGS="$with_cudd_prefix/cudd/libppddl_planner_cudd.la $with_cudd_prefix/util/libppddl_planner_util.la $with_cudd_prefix/mtr/libppddl_planner_mtr.la $with_cudd_prefix/st/libppddl_planner_st.la $with_cudd_prefix/dddmp/libppddl_planner_dddmp.la $with_cudd_prefix/epd/libppddl_planner_epd.la"

AC_CHECK_HEADER([cudd.h], , [AC_MSG_ERROR([Cannot find cudd headers: install CUDD from ftp://vlsi.colorado.edu/pub/cudd-2.4.1.tar.gz])])
AC_CHECK_FILE($with_cudd_prefix/cudd/libppddl_planner_cudd.la, , [AC_MSG_ERROR([Cannot find cudd library: compile CUDD before running configure])])

AC_SUBST(CUDD_CPPFLAGS)
AC_SUBST(CUDD_LDFLAGS)


##################################################
# Checks for mdpsim
##################################################
dnl Check for mdpsim.

AC_ARG_WITH(mdpsim-prefix,
                        [  --with-mdpsim-prefix=DIR		mdpsim is installed in DIR],
			[with_mdpsim_prefix=$withval], )

MDPSIM_CPPFLAGS="-I$with_mdpsim_prefix/include -I$with_mdpsim_prefix/include/ppddl_planner_mdpsim"
CPPFLAGS="$CPPFLAGS $MDPSIM_CPPFLAGS"
MDPSIM_LDFLAGS="-L$with_mdpsim_prefix/lib"
LDFLAGS="$LDFLAGS $MDPSIM_LDFLAGS"

AC_CHECK_HEADER([ppddl_planner_mdpsim/actions.h], , [AC_MSG_ERROR([Cannot find mdpsim headers: install mdpsim from http://code.google.com/p/mdpsim/])])
AC_CHECK_FILE($with_mdpsim_prefix/lib/libppddl_planner_mdpsim.a, , [AC_MSG_ERROR([Cannot find mdpsim library: install mdpsim from http://code.google.com/p/mdpsim/])])

AC_SUBST(MDPSIM_CPPFLAGS)
AC_SUBST(MDPSIM_LDFLAGS)


##################################################
# Checks for FF
##################################################
dnl Check for FF.


AC_ARG_WITH(ff-command,
                        [  --with-ff-command=COM		FF command is COM],
			[with_ff_command=$withval], )

AC_DEFINE_UNQUOTED([FF_COMMAND], ["$with_ff_command"], [FF command])


##################################################
# Checks for METRIC-FF
##################################################
dnl Check for METRIC-FF.


AC_ARG_WITH(mff-command,
                        [  --with-mff-command=COM		METRIC-FF command is COM],
			[with_mff_command=$withval], )

AC_DEFINE_UNQUOTED([METRIC_FF_COMMAND], ["$with_mff_command"], [METRIC-FF command])


##################################################
# Check for memory relaxed heuristics
##################################################
#dnl Check for memory relaxed heuristics

AC_ARG_ENABLE(mrh,
[  --enable-mrh          Enable memory relaxed heuristics mode (disabled by default)],
[mrh_support=$enableval],
[mrh_support="no"])
if test $mrh_support = "yes"; then
    AC_DEFINE(MEMORY_RELAXED_HEURISTICS, 1, [Turn on memory relaxed heuristics mode])
	AC_MSG_RESULT(Memory relaxed heuristics support: YES)
else
	AC_MSG_RESULT(Memory relaxed heuristics support: NO)
fi


##################################################
# Check for STRIPS determinization
##################################################
#dnl Check for STRIPS determinization

#AC_ARG_ENABLE(strips-determinization,
#[  --enable-strips-determinization          Enable strips determinization (disabled by default, but required to use LPG or YAHSP deterministic planners)],
#[strips_determinization_support=$enableval],
#[strips_determinization_support="no"])
#if test $strips_determinization_support = "yes"; then
#    AC_DEFINE(STRIPS_DETERMINIZATION, 1, [Turn on strips determinization mode])
#	AC_MSG_RESULT(STRIPS determinization support: YES)
#else
#	AC_MSG_RESULT(STRIPS determinization support: NO)
#fi


AC_CONFIG_FILES([Makefile])
#AC_OUTPUT(Makefile ppddl_planner.lsm ppddl_planner.spec)
AC_OUTPUT
