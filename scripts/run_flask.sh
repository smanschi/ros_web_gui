#!/bin/bash
export FLASK_APP=ros_web_gui
export FLASK_ENV=development
BASEDIR=$(dirname "$0")
cd ${BASEDIR}/..
source venv/bin/activate
flask run
