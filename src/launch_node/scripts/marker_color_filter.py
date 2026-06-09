#!/usr/bin/env python3

import sys

import rospy
from visualization_msgs.msg import Marker, MarkerArray


class MarkerColorFilter:
    def __init__(self):
        args = rospy.myargv(argv=sys.argv)
        if len(args) != 7:
            raise RuntimeError(
                "usage: marker_color_filter.py INPUT_TOPIC OUTPUT_TOPIC R G B A"
            )

        self.input_topic = args[1]
        self.output_topic = args[2]
        self.r = float(args[3])
        self.g = float(args[4])
        self.b = float(args[5])
        self.a = float(args[6])

        self.publisher = rospy.Publisher(self.output_topic, MarkerArray, queue_size=10)
        self.subscriber = rospy.Subscriber(
            self.input_topic, MarkerArray, self.callback, queue_size=10
        )

    def callback(self, msg):
        filtered = MarkerArray()
        for marker in msg.markers:
            out = marker
            if out.action not in (Marker.DELETE, Marker.DELETEALL):
                out.color.r = self.r
                out.color.g = self.g
                out.color.b = self.b
                out.color.a = self.a if out.color.a <= 0.0 else min(out.color.a, self.a)
            filtered.markers.append(out)
        self.publisher.publish(filtered)


if __name__ == "__main__":
    rospy.init_node("marker_color_filter")
    MarkerColorFilter()
    rospy.spin()
