#!/usr/bin/env python

import rospy, sys, select, os
from std_msgs.msg import Float32, Int32, Int32MultiArray, Float32MultiArray, Bool, String
import argparse
import limits
from datetime import datetime
sys.path.insert(0, os.getcwd()[:os.getcwd().find('TerraBot')]+'TerraBot/lib')
from terrabot_utils import clock_time
from freqmsg import tomsg

class Sensors:
    time = 0
    light_level = 0
    moisture = 0
    humidity = 0
    temperature = 0
    water_level = 0
    current = 0
    energy = 0
    light_level_raw = [0,0]
    moisture_raw = [0,0]
    humidity_raw = [0,0]
    temperature_raw = [0,0]

parser = argparse.ArgumentParser(description = "Interactive Agent")
parser.add_argument('-l', '--log', action = 'store_true',
                    help="print sensor values")
parser.add_argument('-s', '--sim', action = 'store_true', help="use simulator")
args = parser.parse_args()

sensorsG = Sensors()
is_logging = args.log
use_simulator = args.sim

def init_sensors():
    global sensorsG
    sensorsG.time = rospy.get_time()

### ROS-related stuff
### Set up publishers, subscribers, and message handlers

def init_ros ():
    global led_pub, wpump_pub, fan_pub, ping_pub, camera_pub, speedup_pub, freq_pub, sensorsG

    if use_simulator: rospy.set_param("use_sim_time", True)
    rospy.init_node("interactive_agent", anonymous = True)

    led_pub = rospy.Publisher("led_input", Int32, latch = True, queue_size = 1)
    wpump_pub = rospy.Publisher("wpump_input", Bool, latch = True,
                                queue_size = 1)
    fan_pub = rospy.Publisher("fan_input", Bool, latch = True, queue_size = 1)

    ping_pub = rospy.Publisher("ping", Bool, latch = True, queue_size = 1)
    camera_pub = rospy.Publisher("camera", String, latch = True, queue_size = 1)
    speedup_pub = rospy.Publisher("speedup", Int32, latch = True, queue_size = 1)
    freq_pub = rospy.Publisher("freq_input", String, latch=True, queue_size=1)

    rospy.Subscriber("smoist_output", Int32MultiArray,
                     moisture_reaction, sensorsG)
    rospy.Subscriber("light_output", Int32MultiArray, light_reaction, sensorsG)
    rospy.Subscriber("level_output", Float32, level_reaction, sensorsG)
    rospy.Subscriber("temp_output", Int32MultiArray, temp_reaction, sensorsG)
    rospy.Subscriber("humid_output", Int32MultiArray, humid_reaction, sensorsG)
    rospy.Subscriber("cur_output", Float32MultiArray, power_reaction, sensorsG)

def moisture_reaction(data, sensorsG):
    sensorsG.moisture = (data.data[0] + data.data[1])/2
    sensorsG.moisture_raw = data.data
    if is_logging: print("    Moisture: %d %d" %(data.data[0], data.data[1]))

def humid_reaction(data, sensorsG):
    sensorsG.humidity = (data.data[0] + data.data[1])/2
    sensorsG.humidity_raw = data.data
    if is_logging: print("    Humidity: %d %d" %(data.data[0], data.data[1]))

def temp_reaction(data, sensorsG):
    sensorsG.temperature = (data.data[0] + data.data[1])/2
    sensorsG.temperature_raw = data.data
    if is_logging: print("    Temperature: %d %d" %(data.data[0], data.data[1]))

def light_reaction(data, sensorsG):
    sensorsG.light_level = (data.data[0] + data.data[1])/2
    sensorsG.light_level_raw = data.data
    if is_logging: print("    Lights: %d %d" %(data.data[0], data.data[1]))

def level_reaction(data, sensorsG):
    sensorsG.water_level = data.data
    if is_logging: print("    Level: %.2f" %data.data)

def power_reaction(data, sensorsG):
    sensorsG.current = -data.data[0]
    sensorsG.energy = -data.data[1]
    if is_logging:
        print("    Current: %f Energy: %f" %(sensorsG.current, sensorsG.energy))

def cam_reaction(data):
    print ("picture taken\t" + data.data)

def ping():
    global last_ping
    print("PING! %s" %clock_time(sensorsG.time))
    last_ping = sensorsG.time
    ping_pub.publish(True)

init_ros()
init_sensors()
rospy.sleep(2) # Give a chance for the initial sensor values to be read
ping()

while not rospy.core.is_shutdown():
    sensorsG.time = rospy.get_time()

    # Ping every 3 minutes, twice as frequently as timeout in TerraBot
    if (sensorsG.time - last_ping) >= 180: ping()

    ### Check for input
    if sys.stdin in select.select([sys.stdin],[],[],0)[0]:
        input = sys.stdin.readline()
        if input[0] == 'q':
            quit()
        elif input[0] == 'f':
            print("Turning fans %s" %("on" if (input.find("on") > 0) else "off"))
            fan_pub.publish(input.find("on") > 0)
        elif input[0] == 'p':
            print("Turning pump %s" %("on" if (input.find("on") > 0) else "off"))
            wpump_pub.publish(input.find("on") > 0)
        elif input[0] == 'l':
            level = (0 if (input.find("off") > 0) else
                     255 if (input.find("on") > 0) else int(input[1:]))
            print("Adjusting light level to %d" %level)
            led_pub.publish(level)
        elif input[0] == 'c':
            print("Taking a picture, storing in %s" %input[2:-1])
            camera_pub.publish(input[2:-1])
        elif input[0] == 'r':
            sensor, freq = input[2:-1].split(" ")
            msg = tomsg(sensor, float(freq))
            if msg is not None:
                print("Updating {} to frequency {}".format(sensor, freq))
                freq_pub.publish(msg)
        elif input[0] == 's':
            speedup_pub.publish(int(input[1:]))
        elif input[0] == 'v':
            print("Sensor values at %s:" %clock_time(sensorsG.time))
            print("  Light level: %.1f (%.1f, %.1f)"
                  %(sensorsG.light_level, sensorsG.light_level_raw[0],
                    sensorsG.light_level_raw[1]))
            print("  Temperature: %.1f (%.1f, %.1f)"
                  %(sensorsG.temperature, sensorsG.temperature_raw[0],
                    sensorsG.temperature_raw[1]))
            print("  Humidity: %.1f (%.1f, %.1f)"
                  %(sensorsG.humidity, sensorsG.humidity_raw[0],
                    sensorsG.humidity_raw[1]))
            print("  Soil moisture: %.1f (%.1f, %.1f)"
                  %(sensorsG.moisture, sensorsG.moisture_raw[0],
                    sensorsG.moisture_raw[1]))
            print("  Reservoir level: %.1f" %sensorsG.water_level)
        else:
            print("Usage: q (quit)\n\tf [on|off] (fan on/off)\n\tp [on|off] (pump on/off)\n\tl [<level>|on|off] (led set to level ('on'=255; 'off'=0)\n\tr <sensor> <frequency> (update sensor to frequency)\n\tc <file> (take a picture, store in 'file')\n\ts [<speedup>] (change current speedup)\n\tv (print sensor values)")

    rospy.sleep(1)
