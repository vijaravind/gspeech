#!/usr/bin/env python

"""
Module to control the ASR module through keyboard.
"""


import sys
import rospy
from std_msgs.msg import Bool


class KeyboardCtrl(object):
    """Keyboard controller to start / stop speech recognition"""

    def __init__(self):
        """Constructor"""
        # start ROS node
        rospy.init_node('asr_kbdctrl', log_level=rospy.INFO)
        # configure ROS settings
        rospy.on_shutdown(self.shutdown)
        self.pub_start = rospy.Publisher('/asr/start', Bool, queue_size=1)
        self.pub_stop = rospy.Publisher('/asr/stop', Bool, queue_size=1)
        self.sub_recognising = rospy.Subscriber(
            '/asr/recognising', Bool, self.recognising_callback
        )
        # run controller
        self.recognising = False
        self.controller()

    def shutdown(self):
        """Stop all system process before killing node"""
        self.sub_recognising.unregister()

    def controller(self):
        """Controller"""
        while True:
            ch = raw_input("Press ENTER to control the ASR module")
            if ch == 'q':
                rospy.signal_shutdown("Exiting...")
            if self.recognising:
                self.pub_stop.publish(Bool(True))
                rospy.loginfo("stopping...")
            else:
                self.pub_start.publish(Bool(True))
                rospy.loginfo("starting...")

    def recognising_callback(self, msg):
        """Callback function for recognising"""
        self.recognising = msg.data


def main():
    ctrl = KeyboardCtrl()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        sys.exit(0)


