# About
ros_web_gui is a package for visualizing the available ROS nodes, topics, parameters and service
calls in a web browser. It provides much of the functionality that is also available with
the ROS command line tools and rqt_graph, but with a graphical user interface. When running
the ros_web_gui, a flask-based webserver will be started that can be accessed on port 5000.

# Installation
conda install flask pygraphviz pyyaml rospy

OR

python3 -m venv venv\
pip install -r requirements.txt

# Running
You can run the scripts/run_flask.sh script which will source the virtual environment
and run flask (currently in Debug mode). Then, go to your browser and open
http://localhost:5000.

For trying out the Web Gui, you can also run the scripts/run_ros.sh script which will
start a roscore together with a simple publisher and a subscriber for testing purposes.
