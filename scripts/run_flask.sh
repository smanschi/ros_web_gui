#!/bin/bash
export FLASK_APP=ros_web_gui
export FLASK_ENV=development
BASEDIR=$(dirname "$0")
cd ${BASEDIR}/..
flask run --no-reload
