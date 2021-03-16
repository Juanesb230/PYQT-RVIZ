#!/usr/bin/env python 
import rospy
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import math
import geometry_msgs.msg
from nav_msgs.msg import Odometry

v=deque(np.zeros(800))
w=deque(np.zeros(800))
vr=deque(np.zeros(800))
wr=deque(np.zeros(800))
t=deque(np.arange(800))
t1= 0.0
x_v = 0.0
v_v = 0.0
a_v = 0.0
x_w = 0.0
v_w = 0.0
a_w = 0.0
vr_ant = 0.0 
wr_ant = 0.0
w_odometry = 0.0
v_odometry = 0.0

#ODOMETRY DATA
def Odom(msg):
    global w_odometry
    global v_odometry
    w_odometry = msg.twist.twist.angular.z
    v_odometry = msg.twist.twist.linear.x
    rate.sleep()

def alfa_beta_gamma(alfa,beta,gamma,xs,vs,aS,wm):
    xk = xs + 0.1 * vs + 0.005 * aS
    vk = vs + 0.1 * aS
    ak = aS
    rk = wm - xk
    xs = xk + alfa * rk
    vs = vk + beta * rk / 0.1
    aS = ak + gamma / 0.02 *rk
    return xs, vs, aS

def nM(u,w1,ur,wr1,ur_ant,wr1_ant):
    A = np.array([[0, 0, -(w1*w1), u, 0, 0],[0, 0, 0, 0, u*w1, w1]])
    B = np.transpose(np.array([0.24089, 0.2424, - 0.00093603, 0.99624, -0.0037256, 1.0915]))
    n = np.dot(A,B)
    eu = ur - u
    ew = wr1 - w1
    sigma1 = (ur - ur_ant) / 0.1 + 3.0*math.tanh(1.7*eu)
    sigma2 = (wr1 - wr1_ant) / 0.1 + 1.0*math.tanh(2*ew) 
    M = np.array([[0.24089, 0],[0, 0.2424]])
    udc = np.dot(M,np.array([[sigma1],[sigma2]])) + n
    return udc[0, 0], udc[1,0], ur, wr1



if __name__ == '__main__':
        #NODE DEFINITION
        rospy.init_node('dynamic_control')
        sub_odom = rospy.Subscriber('/mobile_base_controller/odom',Odometry,Odom)
        diff_vel = rospy.Publisher('/mobile_base_controller/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown() and t1<80:
            print(t1)
            t1 = t1 + 0.1
            linear = 0.5
            angular = 0.5
            vr.popleft()
            wr.popleft()
            vr.append(linear)
            wr.append(angular)
            x_v , v_v , a_v = alfa_beta_gamma(0.1, 0.002, 0.00001, x_v, v_v, a_v, v_odometry)
            x_w , v_w , a_w = alfa_beta_gamma(0.05, 0.005, 0.0005, x_w, v_w, a_w, w_odometry)
            linear1, angular1, vr_ant, wr_ant = nM(x_v, x_w, linear, angular, vr_ant, wr_ant)
            if linear > 1.0:
                linear = 1
            elif linear < -0.5:
                linear = -0.5
            if angular > 1.7 :
                angular = 1.7
            elif angular < -1.7:
                angular = -1.7
            cmd = geometry_msgs.msg.Twist()
            cmd.linear.x = linear
            cmd.angular.z = angular
            diff_vel.publish(cmd)
            v.popleft()
            w.popleft()
            v.append(x_v)
            w.append(w_odometry)
            rate.sleep()
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        diff_vel.publish(cmd)
        plt.subplot(2,1,1)
        plt.plot(t, v,'b',t,vr,'r')
        plt.subplot(2,1,2)
        plt.plot(t, w,'b',t,wr,'r')
        plt.show()