#!/usr/bin/env python

#import roslib; roslib.load_manifest('laser_assembler')
import rospy
from laser_assembler.srv import *
from sensor_msgs.msg import PointCloud2

rospy.init_node("test_client")
rospy.wait_for_service("assemble_scans2")
assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
pub = rospy.Publisher ("/combined_pointcloud", PointCloud2, queue_size=1)
r = rospy.Rate (1)

while not rospy.is_shutdown():
    try:
        #assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
        resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())
        #print ("Got cloud with %u points", %len(resp.cloud.data))
        print (len(resp.cloud.data))
        pub.publish(resp.cloud)
    except rospy.ServiceException as e:
        #print ("Service call failed: %s", %e)
        print(e)
    r.sleep()
