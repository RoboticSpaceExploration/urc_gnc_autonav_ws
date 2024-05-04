#!/usr/bin/env python3
import rospy
import serial
import serial.tools.list_ports
import subprocess
import ctypes
import threading
from time import time_ns, sleep
from std_msgs.msg import Float32, UInt32, String, Int32, UInt8, Int8
from gnc_control.srv import EncoderCount, EncoderCountRequest, EncoderCountResponse
from gnc_control.srv import SliderPosition, SliderPositionRequest, SliderPositionResponse, CalibrationService

class SerialHandler:
    def __init__(self, port, baudrate):
        self.serial_port = serial.Serial(port, baudrate)
        self.serial_port.timeout = 0.5
        self.arm_log_pub = rospy.Publisher('/arm/log', String, queue_size=10)
        self.write_interval = 0.1
        self.msg_queue = []
        self.max_queue_length = 20
        self.write_thread = threading.Thread(target=self.threaded_write)
        self.write_thread.daemon = True
        self.write_thread.start()
    
    def write(self, msg: str):
        # discard message if msg_queue is full
        if len(self.msg_queue) < self.max_queue_length:
            self.msg_queue.append(msg)
        rospy.loginfo(self.msg_queue)
    
    def threaded_write(self):
        """
        don't call this function, it is an infinite loop polling thread
        """
        while True:
            if len(self.msg_queue) > 0 and self.serial_port.is_open:
                msg = self.msg_queue.pop(0).encode()
                self.serial_port.write(msg)
                rospy.loginfo(f'sent message {msg}')
            sleep(self.write_interval)

    def write_then_read(self, msg: str) -> str:
        # flush reads
        self.serial_port.reset_input_buffer()
        self.write(msg)
        # wait for response
        result = ' '
        rospy.loginfo('writethenread')
        while result != None and result[0] != '$': # $ indicates data to be read
            result = self.serial_port.readline().strip().decode()
            if result == '$hi':
                continue
            # for logging and debugging
            self.arm_log_pub.publish(result) 
            
        return result[1:]

    # def flush_serial(self):
    #     while not rospy.is_shutdown():
    #         if self.serial_port.in_waiting > 0:
    #             data = self.serial_port.readline().strip().decode()
    #             self.response_pub.publish(String(data))
    
    def close(self):
        self.serial_port.close()
                
def joint1_cb(msg: Float32, arduino_serial: SerialHandler):
    # convert angle to units, 8000 encoder ticks = 2pi radians
    radians_to_encoders = 1273.23954474
    position = int(msg.data * radians_to_encoders)
    arduino_serial.write(f'positionm1 128 1000 1000 1000 {position} 0')

def joint1_speed_cb(msg: Float32, arduino_serial: SerialHandler):
    # percent from 0 to 100 of duty cycle
    # dear god why does python not have unsigned types
    if msg.data > 1:
        msg.data = 1
    speed = ctypes.c_uint8(int(msg.data * 126))
    
    if msg.data < 0.0:
        # invert bits cause python doesnt have unsigned types
        arduino_serial.write(f'backwardm1 128 {~speed.value}')
    else: 
        arduino_serial.write(f'forwardm1 128 {speed.value}')
                
def joint2_cb(msg: Float32, arduino_serial: SerialHandler):
    radians_to_encoders = 1273.23954474
    position = msg.data * radians_to_encoders
    arduino_serial.write(f'positionm2 128 1000 1000 1000 {position} 0')

def joint2_speed_cb(msg: Float32, arduino_serial: SerialHandler):
    if msg.data > 1:
        msg.data = 1
    speed = ctypes.c_uint8(int(msg.data * 126))
    if msg.data < 0.0:
        arduino_serial.write(f'backwardm2 128 {~speed.value}')
    else:
        arduino_serial.write(f'forwardm2 128 {speed.value}')
                
# wrist pitch (up down)
def joint3_cb(msg: Float32, arduino_serial: SerialHandler):
    if msg.data > 1:
        msg.data = 1
    percent_to_speed = 32760
    speed = int(msg.data * percent_to_speed)
    if msg.data < 0.0:
        arduino_serial.write(f'movewristpitch down {~speed}')
    else:
        arduino_serial.write(f'movewristpitch up {speed}')

# wrist roll (rotate wrist)
def joint4_cb(msg: Float32, arduino_serial: SerialHandler):
    if msg.data > 1:
        msg.data = 1
    percent_to_speed = 32760
    speed = int(msg.data * percent_to_speed)
    if msg.data < 0.0:
        arduino_serial.write(f'movewristroll cw {~speed}')
    else:
        arduino_serial.write(f'movewristroll ccw {speed}')

# slider
def slider_cb(msg: Float32, arduino_serial: SerialHandler):
    # mm to steps conversion
    mm_to_steps = 100.91835704
    position = int(msg.data * mm_to_steps)
    arduino_serial.write(f'moveslider {position}')

def slider_relative_cb(msg: Float32, arduino_serial: SerialHandler):
    mm_to_steps = 100.91835704
    position = int(msg.data * mm_to_steps)
    arduino_serial.write(f'movesliderrelative {position}')

def slider_speed_cb(msg: Float32, arduino_serial: SerialHandler):
    mm_to_steps = 100.91835704
    speed = int(msg.data * mm_to_steps)
    if speed < 0:
        arduino_serial.write(f'movesliderspeed right {speed}')
    else:
        arduino_serial.write(f'movesliderspeed left {speed}')

def grip_cb(msg: Float32, arduino_serial: SerialHandler):
    if msg.data < 0:
        arduino_serial.write('movegrip close')
    elif msg.data > 0:
        arduino_serial.write('movegrip open')

def slider_position_service(req: SliderPositionRequest):
    msg = arduino_serial.write_then_read(f'getsliderposition')
    position = SliderPositionResponse()
    try:
        position.steps = int(msg)
        steps_to_mm = 0.009909
        position.mm = steps_to_mm * position.steps
    except:
        rospy.loginfo('Error reading slider position.')
        arduino_serial.arm_log_pub.publish('Error reading slider position')
    rospy.loginfo(f'Parsed slider position: steps={position.steps}, millimeters={position.mm}')
    return position
    
def calibrate_arm_service(req):
    arduino_serial.write(f'calibrate')
    return
    
def read_joint_position_service(req: EncoderCountRequest):
    rospy.loginfo(f'Encoder count service called with args: address={req.address}, motor={req.motor}')
    encoder_count = arduino_serial.write_then_read(f'readencoderm{req.motor} {req.address}') 
    encoders_to_radians = 1 / 1273.23954474
    response = EncoderCountResponse()
    try:
        response.position = int(encoders_to_radians * encoder_count) # data starts with a $
    except:
        rospy.loginfo('Error reading encoder count. Roboclaw might be turned off')
        arduino_serial.arm_log_pub.publish('Error reading encoder count. Roboclaw might be turned off')
    rospy.loginfo(f'Parsed encoder count: {response.position}')
    return response

def find_arduino_usb():
    """
    check that arduino is plugged into usb and return associated device path
    """
    try:
        # This command lists USB devices and grep for Arduino
        result = subprocess.run(['sudo dmesg'], shell=True, text=True, capture_output=True).stdout
    except Exception as e:
        print("Failed to run system command:", str(e))
    lines = [''.join(map(str, line.split(']')[1:])) for line in result.splitlines()]
    arduino_line = list(filter(lambda x: 'arduino' in x.lower(), lines))
    usb_identifier = arduino_line[-1].split(' ')[2]

    # check that arduino is currently plugged in with lsusb
    try:
        lsusb_stdout = subprocess.run(['lsusb'], shell=True, text=True, capture_output=True).stdout
    except Exception as e:
        print("Failed to run system command:", str(e))
    if 'arduino' not in lsusb_stdout.lower():
        return None

    usb_line = list(filter(lambda line: 'ttyACM' in line and usb_identifier in line, lines))[-1]
    return '/dev/' + usb_line.split(' ')[3][:-1] # should return in form ttyACM#, -1 is for colon ":" at end

def shutdown_if_arduino_disconnected(arduino_serial: SerialHandler):
    rospy.Rate(1)
    response = arduino_serial.write_then_read('ping')
    if response != 'hi':
        rospy.signal_shutdown()

if __name__ == "__main__":

    rospy.init_node('arm_arduino_hwin_node')
    arduino_usb_path = find_arduino_usb()
    if arduino_usb_path != None:
        rospy.loginfo("Arduino devices found on the following ports: " + arduino_usb_path)
    else:
        rospy.logerr("No Arduino devices were found. Exiting")
        rospy.signal_shutdown("No Arduino devices were found. Exiting")
        exit(1)

    port = rospy.get_param('/arm/config/serial_port', arduino_usb_path)
    baudrate = rospy.get_param('/arm/config/baudrate', 115200)

    rospy.loginfo(f'Connecting to arduino over serial port: {arduino_usb_path}')
    arduino_serial = SerialHandler(port, baudrate)

    rospy.Subscriber('/arm/joint1_position', Float32, joint1_cb, arduino_serial, queue_size=10)
    rospy.Subscriber('/arm/joint1_speed', Float32, joint1_speed_cb, arduino_serial, queue_size=10)
    rospy.Subscriber('/arm/joint2_position', Float32, joint2_cb, arduino_serial, queue_size=10)
    rospy.Subscriber('/arm/joint2_speed', Float32, joint2_speed_cb, arduino_serial, queue_size=10)
    rospy.Subscriber('/arm/joint3_speed', Float32, joint3_cb, arduino_serial, queue_size=10)
    rospy.Subscriber('/arm/joint4_speed', Float32, joint4_cb, arduino_serial, queue_size=10)
    rospy.Subscriber('/arm/slider', Float32, slider_cb, arduino_serial, queue_size=10)
    rospy.Subscriber('/arm/slider_relative', Float32, slider_relative_cb, arduino_serial, queue_size=10)
    rospy.Subscriber('/arm/slider_speed', Float32, slider_speed_cb, arduino_serial, queue_size=10)
    rospy.Subscriber('/arm/grip', Float32, grip_cb, arduino_serial, queue_size=10)
    readEncService = rospy.Service('/arm/read_encoder', EncoderCount, read_joint_position_service)
    sliderPosService = rospy.Service('/arm/slider_position', SliderPosition, slider_position_service)
    calibrateArmService = rospy.Service('/arm/calibrate_arm', SliderPosition, calibrate_arm_service)
    rospy.spin()
    rospy.loginfo('Closing serial port connection to arduino')
    arduino_serial.close()
