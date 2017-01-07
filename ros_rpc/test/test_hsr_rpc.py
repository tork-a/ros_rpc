#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 TORK (Tokyo Opensource Robotics Kyokai Association)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import numpy
import random
import os
import re
import time
import tempfile
import unittest

from geometry_msgs.msg import Pose

from ros_rpc.sample import SampleHsrRpc

PKG = 'ros_rpc'


class TestHsrRpc(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.sample_rpc = SampleHsrRpc()

    @classmethod
    def tearDownClass(cls):
        True

    def test_actionserver_alive(self):
        self.assertTrue(False)  # TODO Fails until meaningful test implemented.

    def test_move_to_neutral(self):
        self.assertTrue(self.sample_rpc.sample_move_to_neutral())

    def test_omni_base_go(self):
        # TODO: omni_base.go requires a map server running, so we need to run
        #       a launch from Gazebo that does that. Without the map the action
        #       always fails thus this test never passes. 
        self.assertTrue(self.sample_rpc.sample_omni_base_go())

    def test_omni_base_get_pose(self):
        '''
        Test criteria: The ROS Service return value contains a valid 
                       geometry_msgs/Pose instance.
                       Since robot can be anywhere, the values can be anything.
                       Each value can even be 0.0.
        '''
        try:
            pose = self.sample_rpc.sample_omni_base_get_pose()
            self.assertTrue(isinstance(pose, Pose))
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_hsr_rpc', TestHsrRpc) 

