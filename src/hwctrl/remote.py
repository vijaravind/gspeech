#!/usr/bin/env python

"""
Module to control the ASR module through remote.
"""


import evdev
import select
import sys
import rospy
from std_msgs.msg import Bool


class RemoteCtrl(object):
    """Remote controller to start / stop speech recognition"""

    def __init__(self):
        """Constructor"""
        # start ROS node
        rospy.init_node('asr_rmtctrl', log_level=rospy.INFO)
        # configure ROS settings
        rospy.on_shutdown(self.shutdown)
        self.pub_start = rospy.Publisher('/asr/start', Bool, queue_size=1)
        self.pub_stop = rospy.Publisher('/asr/stop', Bool, queue_size=1)
        self.sub_recognising = rospy.Subscriber(
            '/asr/recognising', Bool, self.recognising_callback
        )
        # configure device
        self.device = evdev.InputDevice('/dev/input/event9')
        # the event.code for scroll wheel event is 8,
        # buttons: left - 272, right - 273
        self.right_button = 458830
        # run controller
        self.recognising = False
        self.controller()

    def shutdown(self):
        """Stop all system process before killing node"""
        self.sub_recognising.unregister()

    def controller(self):
        """Controller"""
        prev = 0.0
        while True:
            if self.device is not None:
                r, w, x = select.select([self.device], [], [])
                for event in self.device.read():
                    time_diff = time.time() - prev
                    if event.value == self.right_button and time_diff > 1.0:
                        if self.recognising:
                            self.pub_stop.publish(Bool(True))
                            rospy.loginfo("stopping...")
                        else:
                            self.pub_start.publish(Bool(True))
                            rospy.loginfo("starting...")
                        prev = time.time()

    def recognising_callback(self, msg):
        """Callback function for recognising"""
        self.recognising = msg.data


def main():
    ctrl = RemoteCtrl()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        sys.exit(0)


