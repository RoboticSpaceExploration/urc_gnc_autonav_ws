----------------Dependencies--------------
sudo apt-get install usb_cam 
sudo apt-get install ros-*version*-perception

#--------MultiCamera Launch-----------------
#Note need to make sure device ports are assigned correctly
# ls /dev/ | grep video (to identify ports in use)
#Ex. <param name="video_device" value="/dev/video0 <---" />

#1.roscore (tab1)
#2. source ~/urc_gnc_autonav_ws/devel/setup.bash (tab2)
#3. roslaunch gnc_ground multicamera.launch (tab2)

