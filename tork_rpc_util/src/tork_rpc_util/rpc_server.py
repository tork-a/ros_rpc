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

import actionlib
import genpy
import rospy


class ActionServiceInfo(object):
    '''
    A small entity class to store info of a ROS Action.
    '''

    def __init__(self, action_name,
                 action_class,
                 callback_method,
                 action_server=None):
        self.action_name = action_name
        self._action_class = action_class
        self._callback_method = callback_method
        self.action_server = action_server  # Public


class RpcServersHandler(object):
    '''
    RPC (Remote Procedure Call) for methods in HSR class.
    '''
    def _shutdown_hook(self):
        print "ROS RPC Server is shutting down."

    def __init__(self, action_service_info, server_node_name=''):
        '''
        @param action_service_info: Dict of ActionServiceInfo. Example format:

            action_infos = {
                ActionServiceNameDict.omni_base_get_pose: ActionServiceInfo(
                                                             ActionServiceNameDict.omni_base_get_pose,
                                                             srvs.OmnibaseGetPose,
                                                             self._cb_omni_base_get_pose)
            }
        '''
        # Dict to hold ActionServiceInfo instances.
        self.action_infos = action_service_info

        if server_node_name:
            # Only when node name is not None or zero length, run init_node
            # here. Otherwise it might be run in the derived class.
            rospy.init_node(server_node_name)

        # Initialize action servers
        self._init_actionservers_batch(self.action_infos)

        rospy.on_shutdown(self._shutdown_hook)
        rate = rospy.Rate(5.0)
        ticking = 0
        while not rospy.is_shutdown():
            rospy.logdebug('RPC server running: {}'.format(ticking))
            ticking += 1
            rate.sleep()

    def _init_actionservers_batch(self, action_info_list):
        '''
        @param action_info_list: Dictionary of ActionServiceInfo that contains
                                 action_name, action_class, callback_method.
        @type action_info_list: {str: ActionServiceInfo}
        '''
        rospy.loginfo('Len of action_info_list: {}'.format(len(action_info_list)))
        for action_key, action_info in action_info_list.iteritems():
            action_info = self._init_actionserver_batch(action_info)

        rospy.loginfo('ActionServers are ready.')
#        rospy.logdebug('is_active: {}'.format(self._aserver_move_to_neutral.is_active()))
#        rospy.logdebug('new_goal_available: {}'.format(self._aserver_move_to_neutral.is_new_goal_available()))

    def _init_actionserver_batch(self, action_info):
        '''
        @type action_info: ActionServiceInfo
        @return: ActionServiceInfo
        '''
        rospy.loginfo('action_info obj: {}'.format(action_info))
        # *Action class derives from genpy.Message
        if isinstance(action_info._action_class(), genpy.Message):
            _aserver = actionlib.SimpleActionServer(
                action_info.action_name, action_info._action_class,
                execute_cb=action_info._callback_method, auto_start=False)
            # TODO: throw any exception when it happens. But api doc doesn't provide any info about it
            #       http://docs.ros.org/api/actionlib/html/classactionlib_1_1simple__action__server_1_1SimpleActionServer.html#a0a2e0cfe107fe4efb5d883f6754f9c6b

            # Better to explicitly start server.
            # See http://answers.ros.org/question/107126/actionlib-auto_start-parameter/
            _aserver.start()
        # Service class derives `object` (don't get confused with service'
        # *Request and *Response classes that derive genpy.Message
        else:
            ## Not sure if Service requires to be a node.
            # rospy.init_node('add_two_ints_server')
            _aserver = rospy.Service(
                action_info.action_name, action_info._action_class,
                action_info._callback_method)
            # rospy.spin()

        action_info.action_server = _aserver
        return action_info
