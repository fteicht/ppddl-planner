#! /bin/sh
set -e

echo "Install ppddl-planner under a specific prefix on disk (for make install) or keep it local under the source directory (no make install) [prefix|local]?"
read installpolicy
PREFIX=""
if [ "$installpolicy" == "prefix" ]; then
    echo "Enter the prefix where you want to install ppddl-planner"
    read pfx
    PREFIX=$(realpath ${pfx})
    echo "ppddl-planner will be installed in $PREFIX (you will have to explicitly 'make install' in case you need sudo privileges)"
elif [ "$installpolicy" == "local" ]; then
    realpwd=$(realpath ${PWD})
    echo "ppddl-planner compilation files will be located under $realpwd"
else
    echo "Unexpected choice, exiting now"
    exit 1
fi

CUDD="sdk/cudd-2.4.1-ftk"
MDPSIM="sdk/mdpsim-2.2-ftk"
FF="sdk/FF-v2.3-ftk"
MFF="sdk/Metric-FF-ftk"

echo "HAVE YOU CHECKED $CUDD/Makefile ? [yes|no]"
read cuddcheck
if [ "$cuddcheck" != "yes" ]; then
    echo "Please edit and check $CUDD/Makefile before compiling ppddl-planner"
    exit 1
fi

echo "Entering directory $CUDD"
cd $CUDD
make
cd ../..
echo "Leaving directory $CUDD"

echo "Entering directory $MDPSIM"
cd $MDPSIM
./configure --prefix=$PWD --disable-shared --enable-static CFLAGS="-O3 -DNDEBUG" CXXFLAGS="-O3 -DNDEBUG"
make
make install
cd ../..
echo "Leaving directory $MDPSIM"

echo "Entering directory $FF"
cd $FF
make
cd ../..
echo "Leaving directory $FF"

echo "Entering directory $MFF"
cd $MFF
make
cd ../..
echo "Leaving directory $MFF"

echo "Compiling ppddl-planner"
./configure --prefix=$PREFIX --with-cudd-prefix=$CUDD --with-mdpsim-prefix=$MDPSIM --with-ff-command=$(realpath ${FF}/ff) --with-mff-command=$(realpath ${MFF}/ff) CFLAGS="-O3 -DNDEBUG" CXXFLAGS="-O3 -DNDEBUG"
make

if [ "$installpolicy" == "prefix" ]; then
    echo "Please type now 'make install' to install ppddl-planner to $realpwd"
fi
