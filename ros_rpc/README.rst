-----------------------------------------------------------------
ros_rpc (HSR RPC (Remote Procedure Call)) ROS package
-----------------------------------------------------------------

Insall
------

Mkae sure `hsr_rpc_msgs` and `ros_rpc` packages are under you catkin workspace and build them by something like:

::

  mv hsr_rpc_msgs ros_rpc %YOUR_CATKIN_WS%/src
  cd %YOUR_CATKIN_WS%
  rosdep install -r -y --from-paths src --ignore-src
  catkin build ros_rpc
  source devel/setup.bash

Run RPC nodes on simulation
----------------------------

1. Run HSR Gazebo simulation package, by something like:

::

  roslaunch hsrb_gazebo_launch hsrb_empty_world.launch paused:=false

2. Run RPC server that starts ROS nodes for RPC.

::

  rosrun ros_rpc rpc_server_script.py

Run sample remote invocations
------------------------------

1. Start sample script on `ipython` terminal.

::

  ipython -i `rospack find ros_rpc`/script/sample_script.py

2. Following commands are available on the ipython terminal.

* sample_rpc.sample_move_to_neutral: ROS Action
* sample_rpc.sample_omni_base_go: ROS Action
* sample_rpc.sample_omni_base_get_pose: ROS Service

EoF
