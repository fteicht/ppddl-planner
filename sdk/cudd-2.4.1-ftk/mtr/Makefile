# $Id: Makefile,v 1.2 1994/10/03 23:30:34 fabio Exp fabio $
#
#    mtr - multiway-branching tree package
#---------------------------
.SUFFIXES: .lo .o .c .u

CC	= cc
RANLIB	= ranlib
LIBTOOL	= libtool
PURE	=
# Define EXE as .exe for MS-DOS and derivatives.
EXE	=
#EXE	= .exe

MFLAG   =
ICFLAGS = -g
CFLAGS  = $(ICFLAGS) $(MFLAG) $(XCFLAGS)
MTRDEBUG = -DMTR_STATS -DMTR_VERBOSE -DMTR_DEBUG

LINTFLAGS = -u -n -DMTR_STATS -DMTR_VERBOSE -DMTR_DEBUG

# this is to create the lint library
LINTSWITCH = -o

LDFLAGS	=

WHERE	= ..

INCLUDE = $(WHERE)/include

P	= mtr
PSRC    = mtrBasic.c mtrGroup.c
PHDR    = mtr.h
POBJ	= $(PSRC:.c=.o)
PLOBJ	= $(PSRC:.c=.lo)
PUBJ	= $(PSRC:.c=.u)
SRC	= test$(P).c
HDR	=
OBJ		= $(SRC:.c=.o)
LOBJ	= $(SRC:.c=.lo)
UBJ		= $(SRC:.c=.u)
TARGET	= test$(P)$(EXE)
TARGETu	= test$(P)-u

LIBS	= ./libmtr.la $(WHERE)/util/libutil.la

BLIBS	= -kL. -klmtr -kL$(WHERE)/util -klutil

MNEMLIB	=

LINTLIBS = llib-l$(P).ln

#---------------------------

lib$(P).la: $(PLOBJ)
	$(LIBTOOL) --mode=link $(CC) -o $@ $?

.c.lo: $(PSRC) $(PHDR)
	$(LIBTOOL) --mode=compile $(CC) -c  -I$(INCLUDE) $(CFLAGS) $(MTRDEBUG) $<

lib$(P).a: $(POBJ)
	ar rv $@ $?
	$(RANLIB) $@

.c.o: $(PSRC) $(PHDR)
	$(CC) -c  $< -I$(INCLUDE) $(CFLAGS) $(MTRDEBUG) 

optimize_dec: lib$(P).b

lib$(P).b: $(PUBJ)
	ar rv $@ $?
	$(RANLIB) $@

.c.u: $(PSRC) $(PHDR)
	cc -j $<  -I$(INCLUDE) $(XCFLAGS)

# if the header files change, recompile
$(POBJ): $(PHDR)
$(PLOBJ): $(PHDR)
$(PUBJ): $(PHDR)
$(OBJ): $(PHDR)
$(LOBJ): $(PHDR)
$(UBJ): $(PHDR)

lint: llib-l$(P).ln

llib-l$(P).ln: $(PSRC) $(PHDR)
	lint $(LINTFLAGS) $(LINTSWITCH)$(P) -I$(INCLUDE) $(PSRC)

tags: $(PSRC) $(PHDR)
	ctags $(PSRC) $(PHDR)

all: lib$(P).la lib$(P).b llib-l$(P).ln tags

$(TARGET): $(POBJ) $(OBJ) $(LIBS) $(MNEMLIB)
	$(LIBTOOL) --mode=link $(PURE) $(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(OBJ) $(LIBS) $(MNEMLIB) -lm

$(TARGETu): $(SRC) $(PSRC) $(PHDR) $(UBJ) $(PUBJ) $(LIBS:.a=.b)
	cc -O3 $(XCFLAGS) $(LDFLAGS) -o $@ $(UBJ) $(BLIBS) -lm

clean:
	rm -f *.o *.lo *.u mon.out gmon.out *.pixie *.Addrs *.Counts mnem.* \
	.pure core *.warnings

distclean: clean
	rm -rf $(TARGET) $(TARGETu) .libs lib*.a lib*.la lib$(P).b llib-l$(P).ln \
	*.bak *~ tags *.qv *.qx
