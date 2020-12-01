# About
ros_web_gui is a package for visualizing the available ROS nodes, topics, parameters and service
calls in a web browser. It provides much of the functionality that is also available with
the ROS command line tools and rqt_graph, but with a graphical user interface. When running
the ros_web_gui, a flask-based webserver will be started that can be accessed on port 5000.

# Installation (from github)
First, you should install ros on your machine. It is heavily recommened to use Ubuntu 20.04
or higher as ros-noetic is the first version with support for python3.

After installing ROS, you need the python packages pygraphviz, pyyaml and flask. So you can
set up a conda env or virtual environment with

conda install flask pygraphviz pyyaml

OR

pip install flask pygraphviz pyyaml

After this, you should be ready to use the ros_web_gui.

# Running
You can run the scripts/run_flask.sh script which will source the virtual environment
and run flask. Then, go to your browser and open http://localhost:5000.

For trying out the Web Gui, you can also run the scripts/run_ros.sh script which will
start a roscore together with a simple publisher and a subscriber for testing purposes.
