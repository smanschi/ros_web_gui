#!/bin/bash
BASEDIR=$(dirname "$0")
cd ${BASEDIR}/..
roscore &
scripts/publisher.py &
scripts/subscriber.py && fg
