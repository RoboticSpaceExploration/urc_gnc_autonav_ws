#!/usr/bin/env python

import re
import rospy
import subprocess
from std_msgs.msg import String

def parse_and_publish_tegrastats(proc, cpu_util_pub, cpu_temp_pub, gpu_util_pub, gpu_temp_pub):
    while not rospy.is_shutdown():
        line = proc.stdout.readline()
        if not line:
            break

        # Extract CPU and GPU utilization and temperatures from the line
        cpu_util_match = re.search(r'CPU\s*\[(.*?)\]', line)
        gpu_util_match = re.search(r'GR3D_FREQ (\d+)%', line)
        temps_match = re.search(r'AUX@(.*?)C CPU@(.*?)C .* GPU@(.*?)C', line)

        if cpu_util_match:
            cpu_util_pub.publish(cpu_util_match.group(1))

        if gpu_util_match:
            gpu_util_pub.publish(gpu_util_match.group(1) + '%')

        if temps_match:
            cpu_temp_pub.publish(temps_match.group(2) + 'C')
            gpu_temp_pub.publish(temps_match.group(3) + 'C')

def main():
    rospy.init_node('tegrastats_publisher')

    # Publishers for each metric
    cpu_util_pub = rospy.Publisher('/cpu/utilization', String, queue_size=10)
    cpu_temp_pub = rospy.Publisher('/cpu/temps', String, queue_size=10)
    gpu_util_pub = rospy.Publisher('/gpu/utilization', String, queue_size=10)
    gpu_temp_pub = rospy.Publisher('/gpu/temps', String, queue_size=10)

    # Start tegrastats in a subprocess
    proc = subprocess.Popen(['tegrastats'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    try:
        parse_and_publish_tegrastats(proc, cpu_util_pub, cpu_temp_pub, gpu_util_pub, gpu_temp_pub)
    except rospy.ROSInterruptException:
        pass
    finally:
        proc.terminate()  # Ensure the subprocess is terminated when the script ends

if __name__ == '__main__':
    main()

