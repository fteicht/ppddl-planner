## Process this file with automake to produce Makefile.in
##
## Copyright 2003-2005 Carnegie Mellon University and Rutgers University
## Copyright 2007 H�kan Younes
##
## Licensed under the Apache License, Version 2.0 (the "License");
## you may not use this file except in compliance with the License.
## You may obtain a copy of the License at
##
##     http://www.apache.org/licenses/LICENSE-2.0
##
## Unless required by applicable law or agreed to in writing, software
## distributed under the License is distributed on an "AS IS" BASIS,
## WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
## See the License for the specific language governing permissions and
## limitations under the License.

MDPSim_includedir = $(pkgincludedir)

MDPSim_include_DATA = \
	config.h\
	client.h\
	strxml.h\
	requirements.h\
	rational.h\
	types.h\
	terms.h\
	predicates.h\
	functions.h\
	refcount.h\
	expressions.h\
	formulas.h\
	effects.h\
	actions.h\
	domains.h\
	problems.h\
	states.h

lib_LTLIBRARIES = libmdpsim.la
libmdpsim_la_SOURCES = \
	client.cc\
	strxml.cc\
	requirements.cc\
	rational.cc\
	types.cc\
	terms.cc\
	predicates.cc\
	functions.cc\
	expressions.cc\
	formulas.cc\
	effects.cc\
	actions.cc\
	domains.cc\
	problems.cc\
	states.cc\
	parser.yy\
	tokenizer.ll
libmdpsim_la_CPPFLAGS = -fPIC

EXTRA_LTLIBRARIES = libmdpsimmtbdd.la

libmdpsimmtbdd_la_SOURCES = mtbdd.cc	mtbdd.h
libmdpsimmtbdd_la_CPPFLAGS = -fPIC

bin_PROGRAMS = mdpsim mdpclient
EXTRA_PROGRAMS = mtbddclient
mdpsim_SOURCES = mdpsim.cc mdpserver.cc mdpserver.h
mdpclient_SOURCES = mdpclient.cc
mtbddclient_SOURCES = mtbddclient.cc

mdpsim_LDADD = @LIBOBJS@ @PTHREADLIB@ libmdpsim.la
mdpclient_LDADD = parser.o @LIBOBJS@ libmdpsim.la
mtbddclient_CPPFLAGS = @CPPFLAGS@ -I"@CUDDDIR@/include"
mtbddclient_LDFLAGS = @LDFLAGS@ -L"@CUDDDIR@/cudd" -L"@CUDDDIR@/epd" -L"@CUDDDIR@/mtr" -L"@CUDDDIR@/st" -L"@CUDDDIR@/util"
mtbddclient_LDADD = parser.o -lcudd -lepd -lmtr -lst -lutil @LIBOBJS@ libmdpsim.la libmdpsimmtbdd.la

CLEANFILES = logs/* last_id mtbddclient
MAINTAINERCLEANFILES = parser.cc tokenizer.cc config.h.in~
EXTRA_DIST = getopt.c getopt1.c comp.cfg examples logs port LICENSE NOTICE $(MDPSim_include_DATA)

AM_YFLAGS = -d
