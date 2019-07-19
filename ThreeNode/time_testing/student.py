#!/usr/bin/env python

#mock file of student
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import String
import time

light = 0
led_level = 255
hum  = 0
w_level = 3

def get_time():
    return time.time()

light_time = get_time()
cam_time = get_time()

rospy.init_node("student", anonymous = True)

wpump_pub = rospy.Publisher("wpump_input", Bool, queue_size = 100)
npump_pub = rospy.Publisher("npump_input", Bool, queue_size = 100)
apump_pub = rospy.Publisher("apump_input", Bool, queue_size = 100)
led_pub = rospy.Publisher("led_input", Int32, queue_size = 100)
fan_pub = rospy.Publisher("fan_input", Bool, queue_size = 100)
cam_pub = rospy.Publisher("cam_input", Bool, queue_size = 100)

def humid_reaction(data):
    global hum
    hum = data.data

def temp_reaction(data):
    True

def light_reaction(data):
    global light
    light = data.data

def level_reaction(data):
    global w_level
    w_level = data.data

def tds_reaction(data):
    True

def cam_reaction(data):
    print ("picture taken\t" + data.data)

temp_sensor = rospy.Subscriber("temp_output", Int32, temp_reaction)
light_sensor = rospy.Subscriber("light_output", Int32, light_reaction)
level_sensor = rospy.Subscriber("level_output", Int32, level_reaction)
tds_sensor = rospy.Subscriber("tds_output", Int32, tds_reaction)
cam_sensor = rospy.Subscriber("cam_raw", String, cam_reaction)

while not rospy.core.is_shutdown():
    time_now = get_time()
    if time_now - light_time > 12 * 3600:
        led_level ^= 255
        light_time = time_now
    """if light < 9000:
        led_level = min(255, led_level + 1)
    else:
        led_level = max(0, led_level - 1)"""
    if time_now - cam_time > 3600:
        cam_pub.publish(True)
        cam_time = time_now

    led_pub.publish(led_level)
#    print([light,led_level])
    fan_pub.publish(True if hum > 70 else False)
    wpump_pub.publish(True if w_level == 0 else False)
    rospy.rostime.wallsleep(1)

