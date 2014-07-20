#!/bin/sh

CWD=`pwd`
cd ../cc3k && ./build.sh
cd $CWD

CFLAGS=-I../cc3k/include
LDFLAGS=-L../cc3k

gcc $CFLAGS -o cc3k_test main.c $LDFLAGS -lwiringPi -lcc3k -lbcm2835
