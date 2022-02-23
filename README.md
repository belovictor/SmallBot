# SmallBot

SmallBot is a simple 6WD robot simulation based on ROS and Gazebo. The project is being developed and tested on ROS Melodic and Gazebo 9.0.0.

![rviz image](smallbot_description/images/smallbot_rviz.png)

## Working with robot

Checkout the repository into src directory of ros workspace

``git clone https://github.com/belovictor/SmallBot.git``

Build project

``catkin_make``

Run environment setup

``. devel/setup.bash``

### Start model visualization

``roslaunch smallbot_description display_model.launch``

### Start Gazebo simulation

First launch Gazebo simulation

``roslaunch smallbot_gazebo smallbot_world.launch``

Then start control

``rqt``

Open robot control plugin from menu - Plugins -> Robot tools -> Robot steering - and set topic to /cmd_vel
Change linear and angular velocity to make robot moving in Gazebo simulation
