#!/bin/bash
BASEDIR=$(dirname "$0")
cd ${BASEDIR}/..
source venv/bin/activate
roscore &
scripts/publisher.py &
scripts/subscriber.py && fg
