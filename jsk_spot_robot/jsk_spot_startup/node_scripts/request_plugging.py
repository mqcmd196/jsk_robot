#!/usr/bin/env python

import actionlib
import roslibpy
import rospy
from std_msgs.msg import Bool
import timeout_decorator
from timeout_decorator import TimeoutError
from jsk_spot_startup.msg import RequestRemoteRobotPluggingAction
from jsk_spot_startup.msg import RequestRemoteRobotPluggingFeedback
from jsk_spot_startup.msg import RequestRemoteRobotPluggingResult


class RequestPluggingServer(object):
    def __init__(self, timeout):
        rospy.init_node('~request_plugoff_server')
        self.hostname = rospy.get_param('~hostname', default='fetch1075.jsk.imi.i.u-tokyo.ac.jp')
        self.port = rospy.get_param('~port', default=9090)
        self.app_start_service_name = rospy.get_param('~remote_robot_app_start_service', default='/fetch1075/start_app')
        self.client = roslibpy.Ros(host=self.hostname, port=self.port)
        self.ros_app_start_service = roslibpy.Service(client, name=self.app_start_service_name, service_type='app_manager/StartApp')
        self.sub_is_cable_connected = rospy.Subscriber('/spot/status/cable_connected', Bool, self.cable_connection_cb)
        self._feedback = RequestRemoteRobotPluggingFeedback()
        # Connection test
        try:
            rospy.loginfo('Start connection test with {}:{}...'.format(hostname, port))
            self.client.run()
        except Exception as e:
            rospy.logwarn('Failed to establish the connection with {}:{}.'.format(hostname, port))
            rospy.logerr(str(e))
        else:
            self.client.terminate()
            rospy.loginfo('Finish connection test.')
        # ActionLib server
        self._as = actionlib.SimpleActionServer(
            '~', RequestRemoteRobotPluggingAction,
            execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        result = RequestRemoteRobotPluggingResult()
        r = rospy.Rate(1)
        # Initialize result
        result.remote_robot_accepted = False
        result.remote_robot_executable = False
        result.done = False
        if goal.plugging == 'plug':
            request = roslibpy.ServiceRequest({'name':'jsk_fetch_startup/plug_spot_power_connector'})
        elif goal.plugging == 'unplug':
            request = roslibpy.ServiceRequest({'name':'jsk_fetch_startup/unplug_spot_power_connector'})
        else:
            e = ValueError('Plugging service just supports plug or unplug.')
            rospy.logerr(str(e))
            raise e
        # Start action
        try:
            self.client.run()
            roslib_result = self.ros_app_start_service.call(request) # Call remote ros service
            self.update_feedback(remote_robot_status=roslib_result['message'])
            result.remote_robot_accepted = True
            result.remote_robot_executable = roslib_result['started'] # TODO: is it enough?
            if result.remote_robot_executable: # After remote robot plugging app started
                result.remote_robot_executable = True
                result.done = self.confirm_cable_connection(goal.plugging)
            else: # When remote robot plugging app cannot started
                self.update_feedback(status='Failed to execute plugging because of the remote robot\'s not ready for executing the app.')
        except TimeoutError as e: # plugging app timeout
            rospy.logwarn('Plugging app timeout')
            rospy.logerr(str(e))
            self.update_feedback(status=str(e))
            # TODO: add the command that making remote robot abort the app and go back their dock
        except Exception as e:  # remote ros service not called correctly. Catching roslibpy exception
            rospy.logerr(str(e))
            self.update_feedback(status=str(e))
        else:
            self.client.terminate()
        finally:
            self._as.publish_feedback()
            r.sleep()
            self._as.set_succeeded(result)

    @timeout_decorator.timeout(timeout)
    def confirm_cable_connection(self, plugging):
        """
        """
        done = False
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.is_cable_connected and plugging == 'plug':
                done = True
                break
            elif (not self.is_cable_connected) and plugging == 'unplug':
                done = True
                break
            else:
                continue
        return done

    def update_feedback(self, status=None, remote_robot_status=None):
        """
        """
        if status:
            self._feedback.status = status
        if remote_robot_status:
            self._feedback.remote_robot_status = remote_robot_status
        self._as.publish_feedback(self._feedback)

    def cable_connection_cb(self, msg):
        """
        """
        self.is_cable_connected = msg

if __name__ == '__main__':
    rospy.init_node('request_remote_robot_plugging_server')
    timeout = rospy.get_param('~timeout', default=120.)
    server = RequestPluggingServer(timeout=timeout)
    rospy.spin()
