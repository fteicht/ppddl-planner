# $Id: Makefile,v 1.3 2001/03/19 07:34:37 fabio Exp fabio $
#
# obj: CUDD's C++ interface
#---------------------------------------------------------------------------
.SUFFIXES: .lo .o .cc .u

CPP	= g++
RANLIB	= ranlib
LIBTOOL	= libtool
# Define EXE as .exe for MS-DOS and derivatives.
PURE	=
EXE	=
#EXE	= .exe

MFLAG	=
ICFLAGS = -g
XCFLAGS =
CPPFLAGS =
CFLAGS	= $(ICFLAGS) $(MFLAG) $(XCFLAGS) $(CPPFLAGS)
DDDEBUG =

LINTFLAGS = -u -n -DDD_STATS -DDD_CACHE_PROFILE -DDD_VERBOSE -DDD_DEBUG -DDD_UNIQUE_PROFILE

# this is to create the lint library
LINTSWITCH = -o

WHERE	= ..

INCLUDE = $(WHERE)/include

LIBS	= ./libobj.la $(WHERE)/cudd/libcudd.la $(WHERE)/mtr/libmtr.la \
	$(WHERE)/st/libst.la $(WHERE)/util/libutil.la $(WHERE)/epd/libepd.la

MNEMLIB =

BLIBS	= -kL. -klobj -kL$(WHERE)/cudd -klcudd -kL$(WHERE)/mtr -klmtr \
	-kL$(WHERE)/st -klst -kL$(WHERE)/util -klutil -kL$(WHERE)/epd -klepd

LINTLIBS = ./llib-lobj.ln $(WHERE)/cudd/llib-lcudd.ln \
	$(WHERE)/mtr/llib-lmtr.ln $(WHERE)/st/llib-lst.ln \
	$(WHERE)/util/llib-lutil.ln $(WHERE)/epd/llib-lepd.ln

LDFLAGS =

# files for the package
P	= obj
PSRC	= cuddObj.cc
PHDR	= cuddObj.hh $(INCLUDE)/cudd.h
POBJ	= $(PSRC:.cc=.o)
PLOBJ	= $(PSRC:.cc=.lo)
PUBJ	= $(PSRC:.cc=.u)
TARGET	= test$(P)$(EXE)
TARGETu = test$(P)-u

# files for the test program
SRC	= test$(P).cc
OBJ	= $(SRC:.cc=.o)
LOBJ	= $(SRC:.cc=.lo)
UBJ	= $(SRC:.cc=.u)

#------------------------------------------------------

lib$(P).la: $(PLOBJ)
	$(LIBTOOL) --mode=link $(CC) -o $@ $?

.cc.lo: $(PHDR)
	$(LIBTOOL) --mode=compile $(CPP) -c -I$(INCLUDE) $(CFLAGS) $(DDDEBUG) $<

lib$(P).a: $(POBJ)
	ar rv $@ $?
	$(RANLIB) $@

.cc.o: $(PHDR)
	$(CPP) -c $< -I$(INCLUDE) $(CFLAGS) $(DDDEBUG)

optimize_dec: lib$(P).b

lib$(P).b: $(PUBJ)
	ar rv $@ $?
	$(RANLIB) $@

.cc.u: $(PHDR)
	cxx -j $< -I$(INCLUDE) $(CFLAGS)

# if the header files change, recompile
$(POBJ): $(PHDR)
$(PLOBJ): $(PHDR)
$(PUBJ): $(PHDR)
$(OBJ): $(PHDR)
$(LOBJ): $(PHDR)
$(UBJ): $(PHDR)

$(TARGET): $(SRC) $(OBJ) $(HDR) $(LIBS) $(MNEMLIB)
	$(LIBTOOL) --mode=link $(PURE) $(CPP) $(CFLAGS) $(LDFLAGS) -o $@ $(OBJ) $(LIBS) $(MNEMLIB) -lm

# optimize (DECstations and Alphas only: uses u-code)
$(TARGETu): $(SRC) $(UBJ) $(HDR) $(LIBS:.a=.b)
	cxx -O3 -Olimit 1000 $(XCFLAGS) $(LDFLAGS) -o $@ $(UBJ) $(BLIBS) -lm

lint: llib-l$(P).ln

llib-l$(P).ln: $(PSRC) $(PHDR)
	lint $(LINTFLAGS) $(LINTSWITCH)$(P) -I$(INCLUDE) $(PSRC)

lintpgm: lint
	lint $(LINTFLAGS) -I$(INCLUDE) $(SRC) $(LINTLIBS)

tags: $(PSRC) $(PHDR)
	ctags $(PSRC) $(PHDR)

all: lib$(P).la lib$(P).b llib-l$(P).ln tags

programs: $(TARGET) $(TARGETu) lintpgm

clean:
	rm -f *.o *.lo *.u mon.out gmon.out *.pixie *.Addrs *.Counts mnem.* \
	.pure core *.warnings

distclean: clean
	rm -rf $(TARGET) $(TARGETu) .libs lib*.a lib*.la lib$(P).b llib-l$(P).ln \
	*.bak *~ tags .gdb_history *.qv *.qx
