#!/usr/bin/env python

import re
import rospy
import subprocess
from std_msgs.msg import String

def parse_and_publish_ping(proc, latency_pub):
    while not rospy.is_shutdown():
        line = proc.stdout.readline()
        if not line:
            break

        # Extract latency information from the line
        latency_match = re.search(r'time=(\d+.\d+) ms', line)

        if latency_match:
            latency_pub.publish(latency_match.group(1) + ' ms')

def main():
    rospy.init_node('ping_latency_publisher')

    # Publisher for ping latency
    latency_pub = rospy.Publisher('/network/xavier/latency', String, queue_size=10)

    # Start ping in a subprocess
    proc = subprocess.Popen(['ping', '192.168.1.1'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    try:
        parse_and_publish_ping(proc, latency_pub)
    except rospy.ROSInterruptException:
        pass
    finally:
        proc.terminate()  # Ensure the subprocess is terminated when the script ends

if __name__ == '__main__':
    main()

