----------------Dependencies--------------
sudo apt-get install usb_cam 
sudo apt-get install ros-*version*-perception



#--------MultiCamera Launch-----------------
Note need to make sure device ports are assigned correctly
 ls /dev/ | grep video (to get ports in use)
Ex. <param name="video_device" value="/dev/video0 <---" />

#1.roscore (tab1)
#2. source ~/ros_camera/devel/setup.bash (tab2)
#3. roslaunch ros_camera multicamera.launch (tab2)

#---------View in Web Server---------------
1.launch camera feed
2. rosrun web_video_server web_video_server
3. http://0.0.0.0:8080/ (currently on default settings, ip should be changed to robot ip)


#--------Run Camera Instructions------------
#1. roscore (tab 1)
#2. source ~/ros_camera/devel/setup.bash (tab2)
#3. rosrun ros_camera camera_publisher.py (tab2)
#4. source ~/ros_camera/devel/setup.bash (tab3)
#5. rosrun ros_camera camera_publisher.py (tab3)
