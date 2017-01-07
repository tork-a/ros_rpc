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
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
import genpy
from geometry_msgs.msg import Pose, Quaternion
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


class ActionServiceNameDict(object):
    '''
    Static, entity class to hold name for ROS Actions for the robot.
    '''
    omni_ik = 'srv_ik'


class RosRpcServer(object):
    '''
    RPC (Remote Procedure Call) for methods in HSR class.
    '''
    def _shutdown_hook(self):
        print "ROS RPC Server is shutting down."

    def __init__(self):
        '''
        @param args: TODO
        '''
        # Dict to hold ActionServiceInfo instances.
        self.action_infos = {
            ActionServiceNameDict.omni_base_get_pose: ActionServiceInfo(ActionServiceNameDict.omni_base_get_pose,
                                                                        srvs.OmnibaseGetPose,
                                                                        self._cb_omni_base_get_pose)
            }

        # We need to keep hsrb_interface.Robot class running by something
        # like the following (using "with" keyword). Otherwise ROS node
        # finishes despite rospy.spin() calls. Not sure why but this is
        # how ihsrb does.
        with Robot() as robot:
            self._whole_body = robot.try_get('whole_body')
            self._omni_base = robot.try_get('omni_base')
            self._collision_world = robot.try_get('global_collision_world')
            self._suction = robot.try_get('suction')
            self._gripper = robot.try_get('gripper')
            self._wrist_wrench = robot.try_get('wrist_wrench')
            self._marker = robot.try_get('marker')
            self._battery = robot.try_get('battery')
            self._tts = robot.try_get('default_tts')

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

    def _cb_move_to_neutral(self, goal):
        '''
        Because whole_body.move_to_neutral() is preemptable by simply killing
        the command's execution, it's worth calling it via ROS Action.
        '''
        rospy.loginfo('IN _cb_move_to_neutral. goal: []'.format(goal.time))
        _result = msgs.MoveToNeutralResult()
        _action_info = self.action_infos[ActionServiceNameDict.move_to_neutral]
        _aserver = _action_info.action_server
        # check that preempt has not been requested by the client
        if _aserver.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % _action_info.action_name)
            _aserver.set_preempted()
            _result.res = False
        _res_from_remote = self._whole_body.move_to_neutral()

        _result.res = True
        _aserver.set_succeeded(_result)

    def _cb_omni_base_go(self, goal):
        _result = msgs.OmnibaseGoActionResult()
        _action_info = self.action_infos[ActionServiceNameDict.omni_base_go]
        _aserver = _action_info.action_server
        # check that preempt has not been requested by the client
        if _aserver.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % _action_info.action_name)
            _aserver.set_preempted()
            _result.res = False
        _res_from_remote = self._omni_base.go(goal.x, goal.y, goal.theta, goal.timeout, goal.relative)

        _result.res = True
        _aserver.set_succeeded(_result)

    def _cb_omni_base_get_pose(self, service_req):
        if not service_req.ref_frame_id:
            pos, orientation_hsr = self._omni_base.get_pose('map')
        else:
            pos, orientation_hsr = self._omni_base.get_pose(service_req.ref_frame_id)

        # Screw a non-standard hsrb_interface.geometry.Quaternion class usage,
        # which orientation returned by get_pose is implemented with.
        orientation = Quaternion(orientation_hsr.x, orientation_hsr.y,
                                 orientation_hsr.z, orientation_hsr.w)
        return srvs.OmnibaseGetPoseResponse(Pose(pos, (orientation)))

    def get_pose(self, limb):
        '''
        Running internally the code in http://sdk.rethinkrobotics.com/wiki/IK_Service_-_Code_Walkthrough

        @param limb: str 'left' or 'right'
        '''
