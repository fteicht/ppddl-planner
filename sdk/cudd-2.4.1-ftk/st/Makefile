# $Id: Makefile,v 1.3 2004/01/01 06:53:06 fabio Exp fabio $
#
# st -- hash table package
#---------------------------------------------------------------------------
.SUFFIXES: .c .lo .o .u

CC	= gcc
RANLIB	= ranlib
LIBTOOL	= libtool

MFLAG	=
ICFLAGS = -g -O6 -Wall
CFLAGS  = $(ICFLAGS) $(MFLAG) $(XCFLAGS)

LINTFLAGS = -u -n

# this is to create the lint library
LINTSWITCH = -o

P	= ppddl_planner_st
PSRC	= st.c
PHDR	= st.h
POBJ	= $(PSRC:.c=.o)
PLOBJ	= $(PSRC:.c=.lo)
PUBJ	= $(PSRC:.c=.u)

WHERE	= ..
INCLUDE = $(WHERE)/include

#---------------------------

lib$(P).la: $(PLOBJ)
	$(LIBTOOL) --mode=link $(CC) -o $@ $?

.c.lo: $(PHDR)
	$(LIBTOOL) --mode=compile $(CC) -c -I$(INCLUDE) $(CFLAGS) $<

lib$(P).a: $(POBJ)
	ar rv $@ $?
	$(RANLIB) $@

.c.o: $(PHDR)
	$(CC) -c $< -I$(INCLUDE) $(CFLAGS)

optimize_dec: lib$(P).b

lib$(P).b: $(PUBJ)
	ar rv $@ $?
	$(RANLIB) $@

.c.u: $(PSRC) $(PHDR)
	cc -j $< -I$(INCLUDE) $(XCFLAGS)

# if the header files change, recompile
$(POBJ): $(PHDR)
$(PLOBJ): $(PHDR)
$(PUBJ): $(PHDR)

lint: llib-l$(P).ln

llib-l$(P).ln: $(PSRC) $(PHDR)
	lint $(LINTFLAGS) $(LINTSWITCH)$(P) -I$(INCLUDE) $(PSRC)

tags: $(PSRC) $(PHDR)
	ctags $(PSRC) $(PHDR)

all: lib$(P).la lib$(P).b llib-l$(P).ln tags

clean:
	rm -f *.o *.lo *.u .pure *.warnings

distclean: clean
	rm -rf .libs lib*.a lib*.la lib$(P).b llib-l$(P).ln tags *~ *.bak *.qv *.qx
