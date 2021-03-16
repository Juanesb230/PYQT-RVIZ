#!/usr/bin/env python  
#import roslib
#roslib.load_manifest('ouster_example') 
import rospy
import tf
import math
import geometry_msgs.msg

if __name__ == '__main__':
       rospy.init_node('circle_reference')
       ref_pub = rospy.Publisher('/mobile_base_controller/reference', geometry_msgs.msg.Point,queue_size=1)
       br = tf.TransformBroadcaster()
       listener = tf.TransformListener()
       rate = rospy.Rate(10.0)
       while not rospy.is_shutdown():
            t =rospy.Time.now().to_sec() * math.pi
            x = 2.0 * math.cos(t/70)
            y = 2.0 * math.sin(t/70)
            br.sendTransform([ x, y, 0.0],
                            [0.0, 0.0, 0.0, 1.0],
                            rospy.Time.now(),
                            "reference",
                            "odom")
            
            reference = geometry_msgs.msg.Point()
            reference.x = x
            reference.y = y
            ref_pub.publish(reference)

            rate.sleep() 