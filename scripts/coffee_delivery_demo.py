#!/usr/bin/env python
import os
os.system("gnome-terminal --tab -e 'bash -c \"roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/ubuntu/turtlebot_ws/src/turtlebot_demos/maps/office_map.yaml; exec bash\"' --tab -e 'bash -c \"python order_coffee.py; exec bash\"' --tab -e 'bash -c \"python coffee_bot_test.py; exec bash\"' --tab -e 'bash -c \"ssh -t ai2@192.168.1.7 '/home/ai2/catkin_ws/src/turtlebot_demos/scripts/coffee_delivery_demo_addon.py'; exec bash\"'")

