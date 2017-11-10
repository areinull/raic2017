#!/bin/sh

PACKAGE=strat.zip

rm -f $PACKAGE
zip $PACKAGE MyStrategy* Context*
