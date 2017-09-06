# $Id: Makefile,v 1.12 1999/08/31 19:10:05 fabio Exp fabio $
#
# nanotrav: simple FSM traversal program
#---------------------------------------------------------------------------
.SUFFIXES: .lo .o .c .u

CC	= gcc
#CC	= cc
RANLIB	= ranlib
#RANLIB	= :
LIBTOOL	= libtool
PURE	=

EXE	=
#EXE	= .exe
TARGET	= ppddl_planner_nanotrav$(EXE)
TARGETu = nanotrav-u

WHERE	= ..

INCLUDE = $(WHERE)/include

LIBS	= $(WHERE)/dddmp/libppddl_planner_dddmp.la $(WHERE)/cudd/libppddl_planner_cudd.la \
	$(WHERE)/mtr/libppddl_planner_mtr.la $(WHERE)/st/libppddl_planner_st.la $(WHERE)/util/libppddl_planner_util.la \
	$(WHERE)/epd/libppddl_planner_epd.la

MNEMLIB =
#MNEMLIB	= $(WHERE)/mnemosyne/libppddl_planner_mnem.a

BLIBS	= -kL$(WHERE)/dddmp -kldddmp -kL$(WHERE)/cudd -klcudd -kL$(WHERE)/mtr \
	-klmtr -kL$(WHERE)/st -klst -kL$(WHERE)/util -klutil \
	-kL$(WHERE)/epd -klepd

LINTLIBS = $(WHERE)/llib-ldddmp.a $(WHERE)/cudd/llib-lcudd.ln \
	$(WHERE)/mtr/llib-lmtr.ln $(WHERE)/st/llib-lst.ln \
	$(WHERE)/util/llib-lutil.ln $(WHERE)/epd/llib-lepd.ln

SRC	= main.c bnet.c ntr.c ntrHeap.c ntrBddTest.c ntrMflow.c ntrZddTest.c \
	  ntrShort.c chkMterm.c ucbqsort.c
HDR	= bnet.h ntr.h $(WHERE)/include/dddmp.h $(WHERE)/include/cudd.h \
	$(WHERE)/include/cuddInt.h

OBJ		= $(SRC:.c=.o)
LOBJ	= $(SRC:.c=.lo)
UBJ		= $(SRC:.c=.u)

MFLAG	=
#MFLAG	= -DMNEMOSYNE
ICFLAGS = -g -O6 -Wall
#ICFLAGS = -g -Wall
XCFLAGS = -DHAVE_IEEE_754 -DBSD
#XCFLAGS = -xtaso -ieee_with_no_inexact -tune host
CFLAGS	= $(ICFLAGS) $(MFLAG) $(XCFLAGS)
DDDEBUG =
#DDDEBUG = -DDD_STATS -DDD_VERBOSE -DDD_CACHE_PROFILE -DDD_UNIQUE_PROFILE -DDD_DEBUG

LDFLAGS =
#LDFLAGS = -jmpopt
#LDFLAGS = -non_shared -om -taso

LINTFLAGS = -u -n -DDD_STATS -DDD_CACHE_PROFILE -DDD_VERBOSE -DDD_DEBUG -DDD_UNIQUE_PROFILE

#------------------------------------------------------

$(TARGET): $(SRC) $(OBJ) $(HDR) $(LIBS) $(MNEMLIB)
	$(LIBTOOL) --mode=link $(PURE) $(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(OBJ) $(LIBS) $(MNEMLIB) -lm

.c.lo: $(HDR)
	$(LIBTOOL) --mode=compile $(CC) -c -I$(INCLUDE) $(CFLAGS) $(DDDEBUG) $<

.c.o: $(HDR)
	$(CC) -c $< -I$(INCLUDE) $(CFLAGS) $(DDDEBUG)

# if the header files change, recompile
$(OBJ): $(HDR)
$(LOBJ): $(HDR)
$(UBJ): $(HDR)

optimize_dec: $(TARGETu)

# optimize (DECstations only: uses u-code)
$(TARGETu): $(SRC) $(UBJ) $(HDR) $(LIBS:.a=.b)
	cc -O3 $(XCFLAGS) $(LDFLAGS) $(UBJ) -o $@ $(BLIBS) -lm

.c.u: $(SRC) $(HDR)
	cc -j -I$(INCLUDE) $(XCFLAGS) $<

lint: $(SRC) $(HDR) $(LINTLIBS)
	lint $(LINTFLAGS) -I$(INCLUDE) $(SRC) $(LINTLIBS)

tags: $(SRC) $(HDR)
	ctags $(SRC) $(HDR)

all: $(TARGET) $(TARGETu) lint tags

pixie: $(TARGETu)
	pixie $(TARGETu)

clean:
	rm -f *.o *.lo *.u mon.out gmon.out *.pixie *.Addrs *.Counts mnem.* \
	.pure core *.warnings

distclean: clean
	rm -rf $(TARGET) $(TARGETu) .libs *.bak *~ .gdb_history *.qv *.qx
