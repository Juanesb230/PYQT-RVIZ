#!/usr/bin/env python  
import roslib
roslib.load_manifest('diferencial') 
import rospy
import tf
import math
if __name__ == '__main__':
       rospy.init_node('circle_reference')
       br = tf.TransformBroadcaster()
       listener = tf.TransformListener()
       rate = rospy.Rate(10.0)
       while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() * math.pi
            br.sendTransform(( 2.0 * math.cos(t/70), 2.0 * math.sin(t/70), 0.0),
                            (0.0, 0.0, 0.0, 1.0),
                            rospy.Time.now(),
                            "circle",
                            "odom")
            #print rospy.get_rostime()
            rate.sleep() 