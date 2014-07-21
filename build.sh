#!/bin/sh

export CFLAGS="-DCC3K_DEBUG -ggdb -O0 -I../cc3k/include"
LDFLAGS=-L../cc3k

CWD=`pwd`
cd ../cc3k && ./build.sh
cd $CWD


gcc $CFLAGS -o cc3k_test main.c spi.c $LDFLAGS -lwiringPi -lcc3k -lbcm2835 -lrt
