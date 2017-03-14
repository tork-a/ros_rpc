----------------------------------------------------------------
tork_rpc_util RPC (Remote Procedure Call) ROS package
----------------------------------------------------------------

Introduction
------------

This package provides abstract RPC (Remote Procedure Call) structure for higher level of robot operations for the ROS-based robots.

Some ROS-based robots come with higher level of convenient operation interface (e.g. often written in Python) where robot operators can call complicated tasks by a single command. "RPC" here enables interacting with those commands via ROS Topic/Service/Action from remote nodes.

A few applications of this package can be found at:

* [baxter_app_rpc](http://www.ros.org/news/2012/09/rethink-ros.html) for Baxter.
* [hironx_rpc](http://wiki.ros.org/hironx_rpc) for Hironx dual-arm co-robot.
* [Toyota HSR](http://www.toyota-global.com/innovation/partner_robot/family_2.html) (package not available publicly yet).

Install
--------

Install from binary (recommended)
=================================

::

  sudo apt-get install ros-indigo-tork-rpc

(Optional) Source install
=================================

::
  sudo apt-get install python-catkin-tools python-rosdep python-wstool
  mkdir -p %YOUR_CATKIN_WS%/src
  cd %YOUR_CATKIN_WS%/src
  git clone https://github.com/tork-a/tork_rpc.git
  cd %YOUR_CATKIN_WS%
  rosdep install -r -y --from-paths src --ignore-src
  catkin build
  source devel/setup.bash

Run RPC 
----------------------------

This package only provides libraries/modules, so no executable.

Tech support
--------------

Your contribution in any of the following is appreciated!

* [Report issues](https://github.com/tork-a/tork_rpc/issues)
* Enhancement suggestion [via Github pull request](https://github.com/tork-a/tork_rpc/pulls). 

EoF
