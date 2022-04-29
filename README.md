# SmallBot

SmallBot is a simple 6WD robot simulation based on ROS and Gazebo. The project is being developed and tested on ROS Melodic and Gazebo 9.0.0.

![rviz image](smallbot_description/images/smallbot_rviz.png)

Robot movement and control simulation is based on Gazebo joint_state_publisher and DiffDrivePlugin6W plugins.

## What's in the project

smallbot_base/ - base robot software, not finished yet  
smallbot_control/ - robot control, not finished yet  
smallbot_description/ - xacro/urdf robot model for RVIZ and Gazebo  
smallbot_gazebo/ - Gazebo robot simulation

## Working with robot

Be sure you have ros-melodic-hector-gazebo package installed as SmallBot uses plugins from this package

``sudo apt-get install ros-melodic-hector-gazebo``

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

![gazebo circle move animation](smallbot_gazebo/video/smallbot_circle_move.gif)

Lidar readings can be visualized in RVIZ

![rviz image](smallbot_description/images/smallbot_rviz_lidar.png)
