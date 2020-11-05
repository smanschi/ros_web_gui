# Installation
conda install flask pygraphviz pyyaml rospkg

OR

python3 -m venv tutorial-env
pip install -r requirements.txt

# Running
You can run the scripts/run_flask.sh script which will source the virtual environment
and run flask (currently in Debug mode). Then, go to your browser and open
http://localhost:5000.

For trying out the Web Gui, you can also run the scripts/run_ros.sh script which will
start a roscore together with a simple publisher and a subscriber for testing purposes.
