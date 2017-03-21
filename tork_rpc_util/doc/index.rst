-----------------------------------------------------------------
tork_rpc, RPC (Remote Procedure Call) for ROS-based robot
-----------------------------------------------------------------

.. contents:: Table of Contents
   :depth: 2

Introduction
------------

This package suite provides abstract RPC (Remote Procedure Call) structure for higher level of robot operations for the ROS-based robots.

Some ROS-based robots come with higher level of convenient operation interface (e.g. often written in Python) where robot operators can call complicated tasks by a single command. "RPC" here enables interacting with those commands via ROS Topic/Service/Action from remote nodes.

Existing applications of tork_rpc
----------------------------------

===================================================================================== ======================================================================================= =========================================================================================================
Robot name and manufacturer                                                           RPC package location                                                                    RPC readme
===================================================================================== ======================================================================================= =========================================================================================================
`Baxter <http://www.ros.org/news/2012/09/rethink-ros.html>`_, RethinkRobotics.        `baxter_app_rpc <https://github.com/tork-a/baxter_app_rpc>`_                            `README.rst <https://github.com/tork-a/baxter_app_rpc/blob/master/README.rst>`_
`Hironx / NEXTAGE Open <http://wiki.ros.org/hironx_rpc>`_, KawadaRobotics.            `hironx_rpc <https://github.com/start-jsk/rtmros_hironx/tree/indigo-devel/hironx_rpc>`_ `Hironx README.rst <https://github.com/start-jsk/rtmros_hironx/tree/indigo-devel/hironx_rpc/README.rst>`_
`HSR <http://www.toyota-global.com/innovation/partner_robot/family_2.html>`_, Toyota. `hsr_rpc <https://git.hsr.io/isao_saito/hsr_rpc>`_ (limited access)                     `HSR README.rst <https://git.hsr.io/isao_saito/hsr_rpc/blob/master/hsr_interface_rpc/doc/index.md>`_
===================================================================================== ======================================================================================= =========================================================================================================

You can install individual packages above by following each README file.

(Option) Install some apps above
=================================

If you need app packages, run the following set of commands to semi-automatically install them (assuming ~/cws_rpc with your own catkin workspace path.

NOTE: HSR packages need to be manually installed as of March 2017 due to server access limitation. See its readme to do so.

::

  sudo apt-get install python-catkin-tools python-rosdep python-wstool
  mkdir -p ~/cws_rpc/src && cd ~/cws_rpc
  wstool init src
  wstool merge -t src https://raw.githubusercontent.com/tork-a/tork_rpc/master/tork_rpc_util/.rpc_apps.rosinstall
  wstool update -t src
  rosdep install -r -y --from-paths src --ignore-src
  catkin build
  source devel/setup.bash

Install tork_rpc
------------------------

Do the following **ONLY** when you develop your own RPC package based on `tork_rpc`.

If you're just using the existing RPC packages listed above or other, `tork_rpc` will get installed as a dependency.

Install from binary (recommended)
=================================

::

  sudo apt-get install ros-indigo-tork-rpc

(Optional) Source install
=================================

::

  sudo apt-get install python-catkin-tools python-rosdep python-wstool
  mkdir -p ~/cws_rpc/src
  cd ~/cws_rpc/src
  git clone https://github.com/tork-a/tork_rpc.git
  cd ~/cws_rpc
  rosdep install -r -y --from-paths src --ignore-src
  catkin build
  source devel/setup.bash

Run RPC 
----------------------------

This package only provides libraries/modules, so no executable.

Tech support
--------------

Your contribution in any of the following is appreciated!

* `Report issues <https://github.com/tork-a/tork_rpc/issues>`_
* Enhancement suggestion `via Github pull request <https://github.com/tork-a/tork_rpc/pulls>`_. 

EoF
