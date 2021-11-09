#!/usr/bin/env python

# DIY 360 degree LIDAR (GrauLiDAR)
# add this to your launch file:
#
#  <node pkg="sim" name="graulidar" type="graulidar.py" output="screen">
#    <param name="frame_id" value="scanner" />
#    <param name="scan_topic" value="scan" />
#    <param name="device" value="/dev/ttyUSB0" />
#  </node>


import rospy
import serial
import termios
import math
import random
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2

rospy.init_node('graulidar_scan_publisher')

# disable reset after hangup
#with open(rospy.get_param('~device')) as f:
#	attrs = termios.tcgetattr(f)
#	attrs[2] = attrs[2] & ~termios.HUPCL
#	termios.tcsetattr(f, termios.TCSAFLUSH, attrs)	  

# open serial port
ser = serial.Serial(rospy.get_param('~device'), 115200)
scan_pub = rospy.Publisher(rospy.get_param('~scan_topic'), LaserScan, queue_size=50)

num_readings = 360
laser_frequency = 3.6

scan = LaserScan()
scan.header.stamp = rospy.Time.now()
scan.header.frame_id = rospy.get_param('~frame_id') 
scan.angle_min = 0
scan.angle_max = 2*math.pi-1
scan.angle_increment = math.pi*2.0 / num_readings
scan.time_increment = (1.0 / laser_frequency) / (num_readings)
scan.range_min = 0.0
scan.range_max = 40.0
#scan.ranges = []
#scan.intensities = []
scan.ranges = []
scan.intensities = []
for i in range(0, num_readings):
  scan.ranges.append(0)  
  scan.intensities.append(1)

cloud_pub = rospy.Publisher('point_cloud', PointCloud2, queue_size=10)
cloud_points = []
cloud_header = std_msgs.msg.Header()
cloud_header.stamp = rospy.Time.now()
cloud_header.frame_id = rospy.get_param('~frame_id')

ranges = 0
count = 0
idx = 0
sync = 0
insync = False
angle = 0
distance = 0
data = 0
printTime = 0
r = rospy.Rate(10.0)

print "GrauLiDAR client started"        

try:
  while not rospy.is_shutdown():
    while ser.inWaiting() == 0:
      rospy.sleep(0.01)
    chunk = ser.read( ser.inWaiting() )                
    #if len(chunk) > 0:
    #  print "chunk ", len(chunk)
    for ch in chunk:
      data = ord(ch)            
      #if (random.randint(0,10)) == 5:
      #	data = 1500
      #data = int(ch.encode('hex'), 16)
      # print '%x' % ord(data)
      if ((data == 0xCC) and (sync == 0)): sync = sync + 1 
      elif ((data == 0xDD) and (sync == 1)): sync = sync + 1
      elif ((data == 0xEE) and (sync == 2)): sync = sync + 1
      elif ((data == 0xFF) and (sync == 3)):
        sync = sync + 1  
        if count > 0:
          if rospy.get_time() > printTime:
            printTime = rospy.get_time() + 10               
            print "scan ", count , ": ", ranges       
          scan.header.stamp = rospy.Time.now()
          cloud_header.stamp = rospy.Time.now()
          cloud = pcl2.create_cloud_xyz32(cloud_header, cloud_points)     
          #if ranges > 240:  
          scan_pub.publish(scan)
          cloud_pub.publish(cloud)
        count = count + 1
        insync = True
        idx = 0
        ranges = 0
        cloud_points = []
        for i in range(0, num_readings):
          scan.ranges[i]=0                  
      else:
        sync = 0
      if (insync) and (sync != 4):
        if (idx == 0):
          angle = data << 8
          idx = idx + 1
        elif (idx == 1):
          angle |= data
          idx = idx + 1
        elif (idx == 2):
          distance = data << 8
          idx = idx + 1
        elif (idx == 3):
          distance |= data
          distance = distance / 100.0
          angle=num_readings-angle-1
          idx=0
          ranges = ranges + 1
          if distance > 0:
            if (angle >= 0) and (angle < num_readings):
              scan.ranges[angle]= distance
              cloud_points.append([math.cos(angle/180.0*math.pi)*distance, math.sin(angle/180.0*math.pi)*distance, 0])  
              
    #r.sleep()
    #rospy.sleep(0.01)

except:
  rospy.loginfo("Control node terminated.")
  #traceback.print_exc()
  ser.close()


