#!/usr/bin/env python

from std_msgs.msg import Float64
import termios
import rospy
import sys
import tty
from select import select

move_lookup = {
    'x':(0,0),
    'q':(0,100),
    'a':(0,-100),
    'w':(1,100),
    's':(1,-100),
    'e':(2,100),
    'd':(2,-100),
    'r':(3,100),
    'f':(3,-100),
    't':(4,100),
    'g':(4,-100),
    'y':(5,100),
    'h':(5,-100)
}

pub = []
sett = ""
scale = 0.4

print('Welcome to the worst arm controls have fun')
print('Each joint is controlled using the keys \'qwert\' and \'asdfg\'')
print('x = STOP')
print('q = move joint 1 forward')
print('a = move joint 1 backward')
print('w = move joint 2 forward')
print('s = move joint 2 backward')
print('e = move joint 3 forward')
print('d = move joint 3 backward')
print('r = move joint 4 forward')
print('f = move joint 4 backward')
print('t = move joint 5 forward')
print('g = move joint 5 backward')

def init():
    joints = 6
    rospy.init_node('arm_joint_teleop')
    for i in range(1,joints + 1):
        pub.append(rospy.Publisher('/arm/joint'+str(i)+'_position_controller/command', Float64, queue_size = 1))

def loop():
    while not rospy.is_shutdown(): 
        key = getKey(sett,10.0)
        j = []
        d = []
        if key in move_lookup.keys():
            j.append(move_lookup[key][0])
            d.append(move_lookup[key][1])
            if key == 'x':
                del j[:]
                del d[:]
                for p in pub:
                    msg = Float64()
                    msg.data = 0
                    p.publish(msg)
        else:
            if(key == '\x03'):
                break


        for i in range(0,len(j)):
            msg = Float64()
            msg.data = d[i]*scale
            pub[j[i]].publish(msg)



def getKey(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
         key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__=="__main__":
    sett = saveTerminalSettings()
    init()
    try:
        loop()
    except Exception as e:
        print(e)
    finally:
        restoreTerminalSettings(sett)
