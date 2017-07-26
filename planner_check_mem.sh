#!/bin/bash
#
# Author: Sourcerer / Andreas Ericsson <ae-n0Zl8IkGad4@xxxxxxxxxxxxxxxx>
#
# Copyright (C) 2004 OP5 AB
# All rights reserved.
#
# The -q option makes it perfect for harvesting statistics
#
usage () {
        echo "Usage: ${0##*/} mem|ram|swap|total -c critical -w warning [ -q -r]"
        echo
        echo "  swap     swap disk"
        echo "  ram      RAM memory (cachebuffers are discarded)"
        echo "  mem      See above"
        echo "  total    RAM+Swap (cachebuffers NOT discarded)"
        echo "  -c       % usage critical threshold value (default 90)"
        echo "  -w       % usage warning threshold value (default 80)"
        echo "  -g       output in GB (bah!)"
        echo "  -m       output in MB"
        echo "  -k       output in kB (default)"
        echo "  -b       output in Bytes"
        echo "  -r       reverse (match FREE instead of USED memory)"
        echo "  -q       brief, easily parseable output"
        echo
        echo "Notes:"
        echo "  We rely on /proc/meminfo for data gathering."
        echo "  If not all of your RAM shows up, some of it may be shadowed by the BIOS."
        echo
        echo "  Output of -q is 'used%:used:total'"
        echo
        exit 3
}

if ! [ -e /proc/meminfo ]; then
        echo "/proc/meminfo doesn't appear to exist"
        exit 3
elif ! [ -r /proc/meminfo ]; then
        echo "/proc/meminfo isn't readable"
        exit 3
fi

# sane defaults. set 'reverse' (or pass -r option) if you want to set
# thresholds based on %free instead of %used
check=
crit=90
warn=80
size='k'
reverse=

# get the arguments ...
while test -n "$1" ; do
        case "$1" in
                M*|m*|R*|r*)
                        check="System RAM"
                ;;
                S*|s*)
                        check="Swap"
                ;;
                T*|t*)
                        check="Total mem"
                ;;
                -c)
                        shift
                        crit=$1
                ;;
                -w)
                        shift
                        warn=$1
                ;;
                -k*|-K*)
                        size="k"
                ;;
                -m*|-M*)
                        size="MB"
                ;;
                -b*|-B*)
                        size="b"
                ;;
                -g*|-G*)
                        size='GB'
                ;;
                -q*|-Q*)
                        quiet="yes"
                ;;
                -r*|-R*)
                        reverse="true"
                ;;
                -)
                        :
                ;;
                *)
                        usage
                ;;
        esac
        shift
done

if [ -z "$check" ]; then
        echo "You must supply one of 'ram', 'mem', 'total' or 'swap'"
        usage
fi

test $warn -lt 0 && warn=0
test $warn -gt 100 && warn=100
test $crit -lt 0 && crit=0
test $crit -gt 100 && crit=100

# set all the variables ...
eval `grep -E 'kB$' /proc/meminfo | sed -e 's/: */=/g' -e 's/ *kB$//g'`
if [ $? -ne 0 ]; then
        echo "UNKNOWN: Unable to grep /proc/meminfo"
        exit 3
fi

# calculate used, free RAM and totals
MemFree=$((MemFree+Cached))
MemUsed=$((MemTotal-MemFree))
SwapUsed=$((SwapTotal-SwapFree))
Total=$((SwapTotal+MemTotal))
Free=$((MemFree+SwapFree))
Used=$((MemUsed+SwapUsed))

# Decide what thresholds should be checked against
if [ "$check" = "System RAM" ]; then
        ValTotal=$MemTotal
        ValFree=$MemFree
        ValUsed=$MemUsed
elif [ "$check" = "Swap" ]; then
        ValTotal=$SwapTotal
        ValFree=$SwapFree
        ValUsed=$SwapUsed
elif [ "$check" = "Total mem" ]; then
        ValTotal=$Total
        ValFree=$Free
        ValUsed=$Used
fi

# avoid division by zero
if [ $ValUsed -eq 0 ]; then
        ValPctUsed=0
        ValPctFree=100
fi
if [ $ValTotal -eq 0 ]; then
        # Since this CAN be valid (if the system uses a network mounted swap
        # partition or something equally stupid), we exit with critical because
        # of this, but not until $quiet has been evaluated.
        # This allows graphing tool (who hardly ever send alerts) to show it
        # as a spike of 100% swap usage until the problem is solved.
        msg="CRITICAL - No $check appears available, systems' /proc is incompatible"
        ValPctUsed=100
        ValPctFree=0
fi

ValPctUsed=`echo "$ValUsed / $ValTotal" | bc -l | cut -b2,3`
ValPctUsed=$((ValPctUsed))
ValPctFree=$((100-ValPctUsed))

if [ "$size" = "MB" ]; then
        test $ValTotal -gt 0 && \
                ValTotal=`echo "$ValTotal / 1024" | bc -l | cut -b-5`
        test $ValUsed -gt 0 && \
                ValUsed=`echo "$ValUsed / 1024" | bc -l | cut -b-5`
        test $ValFree -gt 0 && \
                ValFree=`echo "$ValFree / 1024" | bc -l | cut -b-5`
elif [ "$size" = "GB" ]; then
        test $ValTotal -gt 0 && \
                ValTotal=`echo "$ValTotal / 1048576" | bc -l | cut -b-5`
        test $ValUsed -gt 0 && \
                ValUsed=`echo "$ValUsed / 1048576" | bc -l | cut -b-5`
        test $ValFree -gt 0 && \
                ValFree=`echo "$ValFree / 1048576" | bc -l | cut -b-5`
elif [ "$size" = "b" ]; then
        ValTotal=$((ValTotal*1024))
        ValUsed=$((ValUsed*1024))
        ValFree=$((ValFree*1024))
fi

#echo "ValTotal: $ValTotal$size, ValFree: $ValFree$size ($ValPctFree%), ValUsed: $ValUsed$size ($ValPctUsed%)"

# someone's harvesting statistics, so let's humor him/her with some
# easily parseable output
if [ "$quiet" = "yes" ]; then
        echo "$ValPctUsed:$ValUsed:$ValTotal"
        exit 0
fi

if [ "$msg" ]; then
        echo "$msg"
        exit 2
fi

if [ "$reverse" ]; then
        Val=$ValPctFree; operand='-le'; Div=$ValFree; check="$check free"
else
        Val=$ValPctUsed; operand='-ge'; Div=$ValUsed; check="$check used"
fi

# uncomment this if you want strict option parsing
#test $warn $operand $crit && usage

if [ $Val $operand $crit ]; then
        echo -n "CRITICAL"
        status=2
elif [ $Val $operand $warn ]; then
        echo -n "WARNING"
        status=1
else
        echo -n "OK"
        status=0
fi

size=`echo "$size" | sed -e s/b/Bytes/ -e s/k/kB/ -e s/m/MB/`
echo " - $check: ${Val}% ($Div / $ValTotal ${size})"
exit $status

