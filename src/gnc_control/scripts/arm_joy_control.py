#!/usr/bin/env python3

import time
import rospy
from std_msgs.msg import Empty, Float64, Int8, Float32
from sensor_msgs.msg import Joy

xbox_btn_names = ('a', 'b', 'y', 'x', 'lb', 'rb', 'back', 'start', 'power', 'btn_stick_left', 'btn_stick_right')
bitdo_btn_names = ('a', 'b', 'empty' 'x', 'y', 'empty', 'lb', 'rb', 'lt', 'rt', 'back', 'start', 'power', 'btn_stick_left', 'btn_stick_right')

class JoyControls:
    def __init__(self):
        rospy.init_node("joy_controls_node")
        rospy.loginfo("started joy controls")
        self.prev_buttons = tuple([0] * 20)
        self.prev_dpad = tuple([0] * 10)
        
        # urc rover arm
        self.arm_joint1_speed_pub = rospy.Publisher('/arm/joint1_speed', Float32, queue_size=10)
        self.arm_joint2_speed_pub = rospy.Publisher('/arm/joint2_speed', Float32, queue_size=10)
        self.arm_joint3_speed_pub = rospy.Publisher('/arm/joint3_speed', Float32, queue_size=10)
        self.arm_joint4_speed_pub = rospy.Publisher('/arm/joint4_speed', Float32, queue_size=10)
        self.arm_slider_relative_pub = rospy.Publisher('/arm/slider_relative', Float32, queue_size=10)
        self.arm_slider_speed_pub = rospy.Publisher('/arm/slider_speed', Float32, queue_size=10)
        self.arm_grip_pub = rospy.Publisher('/arm/grip', Float32, queue_size=10)
        self.last_publish_ms = self.get_current_time_ms()

        controller = rospy.get_param('/controller_type', 'xbox')
        rospy.loginfo(f'Using {controller} controller')
        if controller == '8bitdo':
            self.btn_names = ('a', 'b', 'empty', 'x', 'y', 'empty', 'lb', 'rb', 'lt', 'rt', 'back', 'start', 'power', 'btn_stick_left', 'btn_stick_right')
            self.axes_names = ('left_x', 'left_y', 'right_x', 'right_y', 'rt', 'lt', 'dpad_x', 'dpad_y')
        elif controller == 'xbox':
            self.btn_names = ('a', 'b', 'x', 'y', 'lb', 'rb', 'back', 'start', 'power', 'btn_stick_left', 'btn_stick_right')
            self.axes_names = ('left_x', 'left_y', 'lt', 'right_x', 'right_y', 'rt', 'dpad_x', 'dpad_y')
            
        rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=10)
        rospy.spin()

    def get_current_time_ms(self):
        return time.time_ns() / 1_000_000

    def joy_callback(self, msg):
        # prevent message spamming
        current_time = self.get_current_time_ms()
        if current_time - self.last_publish_ms < 200:
            return
        self.last_publish_ms = current_time
        # check that button was initially pressed versus released
        dpad = dict(zip(self.axes_names[6:], msg.axes[6:]))
        pressed = dict(zip(self.btn_names, msg.buttons))
        started = dict(zip(self.btn_names, (btn[0] == 1 and btn[1] == 0 for btn in zip(msg.buttons, self.prev_buttons))))
        released = dict(zip(self.btn_names, (btn[0] == 0 and btn[1] == 1 for btn in zip(msg.buttons, self.prev_buttons))))
        axes = dict(zip(self.axes_names, msg.axes))
        dpad_started = dict(zip(self.axes_names[6:], (dpad_temp[0] != 0 and dpad_temp[1] == 0 for dpad_temp in zip(msg.axes[6:], self.prev_dpad))))
        dpad_released = dict(zip(self.axes_names[6:], (dpad_temp[0] == 0 and dpad_temp[1] != 0 for dpad_temp in zip(msg.axes[6:], self.prev_dpad))))
        
        # urc rover
        if pressed['a']:
            rospy.loginfo(axes['left_y'])
            if axes['left_y'] > 0:
                self.arm_joint1_speed_pub.publish(Float32(axes['left_y']))
            elif axes['left_y'] < 0:
                self.arm_joint1_speed_pub.publish(Float32(axes['left_y']))
            else:
                self.arm_joint1_speed_pub.publish(Float32(0))
        if pressed['b']:
            rospy.loginfo(axes['left_y'])
            if axes['left_y'] > 0:
                self.arm_joint2_speed_pub.publish(Float32(axes['left_y']))
            elif axes['left_y'] < 0:
                self.arm_joint2_speed_pub.publish(Float32(axes['left_y']))
            else:
                self.arm_joint2_speed_pub.publish(Float32(0))
        if pressed['rb']:
            self.arm_slider_relative_pub.publish(Float32(-1.5))
        if pressed['lb']:
            self.arm_slider_relative_pub.publish(Float32(1.5))
        if axes['lt'] != 1:
            self.arm_grip_pub.publish(Float32(-2))
        if axes['rt'] != 1:
            self.arm_grip_pub.publish(Float32(2))
        
        if pressed['y']:
            if dpad['dpad_y'] != 0 and dpad['dpad_x'] == 0: # wrist pitch
                if dpad['dpad_y'] > 0:
                    self.arm_joint4_speed_pub.publish(Float32(1))
                elif dpad['dpad_y'] < 0:
                    self.arm_joint4_speed_pub.publish(Float32(-1))
            else:
                self.arm_joint4_speed_pub.publish(Float32(0.0))
        if pressed['start']:
            if dpad['dpad_x'] != 0 and dpad['dpad_y'] == 0: # wrist roll
                if dpad['dpad_x'] > 0:
                    self.arm_joint3_speed_pub.publish(Float32(1))
                elif dpad['dpad_x'] < 0:
                    self.arm_joint3_speed_pub.publish(Float32(-1))
            else:
                self.arm_joint3_speed_pub.publish(Float32(0.0))
        
        
        self.prev_dpad = msg.axes[6:]
        self.prev_buttons = msg.buttons
        
def main():
    JoyControls()

if __name__ == "__main__":
    main()

# 0 A
# 1 B
# 2 X
# 3 Y
# 4 LB
# 5 RB
# 6 back
# 7 start
# 8 power
# 9 Button stick left
# 10 Button stick right

# Table of index number of /joy.axes:
# Axis name on the actual controller
# 0 Left/Right Axis stick left
# 1 Up/Down Axis stick left
# 2 Left/Right Axis stick right
# 3 Up/Down Axis stick right
# 4 RT
# 5 LT
# 6 cross key left/right
# 7 cross key up/down
