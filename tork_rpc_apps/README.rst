================================================================
tork_rpc_apps (RPC (Remote Procedure Call)) ROS package
================================================================

Install
-------

Make sure `baxter_rpc_msgs` and `tork_rpc_util` packages are under you catkin workspace and build them by something like:

::
  cd %YOUR_CATKIN_WS%/src
  wget https://github.com/tork-a/tork_rpc/tree/master/tork_rpc/tork_rpc.rosinstall
  cd %YOUR_CATKIN_WS%  &&  wget update -t src
  rosdep install -r -y --from-paths src --ignore-src
  catkin build
  source devel/setup.bash

Run RPC nodes on simulation
====================================================

Baxter: Run RPC on simulation
------------------------------

1. Run Baxter Gazebo simulation and RPC server:

::

  roslaunch baxter_rpc_server rpc_sim.launch

2. Run sample remote invocations on `ipython` terminal.

::

  ipython -i `rospack find baxter_rpc_server`/script/samplescript_baxter_rpc.py

Following commands are available on the ipython terminal.

* sample_rpc.sample_cartesian_move('{left/right}')

HSR: Run RPC on simulation
------------------------------------
TBD

Hironx: Run RPC on simulation
------------------------------------

1. Run simulation by rtmlaunch and RPC server:
::

  rtmlaunch hironx_ros_bridge hironx_ros_bridge_simulation.launch

2. Run RPC server. NOTE: As of Feb 2017, rtmlaunch above and this needs to be run
separately but it should be possible for both to be run from within rpc_sim.launch.  
  
::

  roslaunch hironx_rpc_server rpc_sim.launch

3. Run sample remote invocations on `ipython` terminal.

::

  ipython -i `rospack find hironx_rpc_server`/script/sample_rpc_script.py

Following commands are available on the ipython terminal.

* sample_rpc.sample_goInitial()


Troubleshooting
====================================================

IK failure with "[/ExternalTools/left/PositionKinematicsNode/IKService] responded with an error"
-------------------------------------------------------------------------------------------------

Make sure Gazebo launch is run from a terminal where `baxter.sh sim` was already run properly.

"rosdep install" returns error
-------------------------------------------------------------------------------------------------

E.g.::

  hsr_interface_rpc: Cannot locate rosdep definition for [hsrb_interface]
  
Any packages whose name start with either `hsr` or `tmc` are not expected to install via `rosdep`. So you can ignore these errors.

EoF
