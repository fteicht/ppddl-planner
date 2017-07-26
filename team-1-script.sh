#! /bin/sh

if [ $# != 2 ] ; then
	echo "Two arguments required: a planner executable name and a directory name."
	exit -1
fi

if [ ! -d $2 ] ; then
    echo "No directory $2."
    exit -1
fi

while read directory ; do
    echo "Entering directory: $directory"


    # Note: this script processes the problems one after another.
    # There is no obligation to do so. Several problems can be
    # processed simultaneously.

    while read domainName domainFile problemName problemFile ; do
	echo "Planning with $1 on domain $domainName, problem $problemName."
	$1 $2/$directory/$domainFile $2/$directory/$problemFile
    done < $2/$directory/p_map

    echo "Leaving directory: $directory"
    echo
done < $2/p_map

