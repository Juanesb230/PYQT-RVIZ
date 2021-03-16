# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'control.ui'
#
# Created by: PyQt4 UI code generator 4.12.1
#
# WARNING! All changes made in this file will be lost!
from __future__ import unicode_literals
import os
import random
import sys

from PyQt4 import QtCore, QtGui
import roslib
roslib.load_manifest('diferencial')
import rospy
import math
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import geometry_msgs.msg
import numpy as np
import threading

from numpy import arange, sin, pi

import matplotlib
matplotlib.use("Qt4Agg")
from matplotlib.backends.backend_qt4agg import (
    FigureCanvasQTAgg as FigureCanvas)
from matplotlib.backends.qt_compat import QtCore, QtGui
from matplotlib.figure import Figure

progname = os.path.basename(sys.argv[0])


class MyMplCanvas(FigureCanvas):
    """Ultimately, this is a QWidget (as well as a FigureCanvasAgg, etc.)."""

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)

        self.compute_initial_figure()

        FigureCanvas.__init__(self, fig)
        self.setParent(parent)

        FigureCanvas.setSizePolicy(self,
                                   QtGui.QSizePolicy.Expanding,
                                   QtGui.QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)

    def compute_initial_figure(self):
        pass


class MyStaticMplCanvas(MyMplCanvas):
    """Simple canvas with a sine plot."""

    def compute_initial_figure(self):
        t = arange(0.0, 3.0, 0.01)
        s = sin(2*pi*t)
        self.axes.plot(t, s)

ESCALAR_FACTOR=70.0
p_ref_ant=np.array([[0.0],[0.0]])
p_ref=np.array([[0.0],[0.0]])
x= 0.0
y= 0.0
theta=0.0
xr= 0.0
yr= 0.0
m=0

class RecurringTimer(threading._Timer):
    """ Own implementation of timer to make it recurring
 
    Timer (based on threading._Timer)
    that invokes a method at a certain interval of seconds
 
    """
     
    def __init__ (self, *args, **kwargs):
        threading._Timer.__init__ (self, *args, **kwargs) 
        self.setDaemon (True)
        self._running = 0
        self._destroy = 0
        self.start()
 
    def run (self):
        while True:
            self.finished.wait (self.interval)
            if self._destroy:
                return
            if self._running:
                self.function (*self.args, **self.kwargs)
 
    def start_timer (self):
        self._running = 1
 
    def stop_timer (self):
        self._running = 0
 
    def is_running (self):
        return self._running
 
    def destroy_timer (self):
        self._destroy = 1

#iNVERSE KINEMATIC PID CONTROL
def Control(angle,x_pos,y_pos,pos_before,pos_ref,K_sin):
    Jr=np.array([[math.cos(angle) ,-0.15 * math.sin(angle)],[math.sin(angle) ,0.15 * math.cos(angle)]])
    Jr_inv=np.linalg.inv(Jr)
    K=np.array([[K_sin[0], 0],[0, K_sin[1]]])
    posp_d=pos_ref - pos_before
    pos=np.array([[x_pos],[y_pos]])
    pos_error=pos_ref - pos
    pos_error[0, 0]=math.tanh(pos_error[0, 0])
    pos_error[1, 0]=math.tanh(pos_error[1, 0])
    c=np.dot(Jr_inv, (10 * posp_d + np.dot(K,pos_error)))
    c1=c[0, 0]
    c2=c[1, 0]
    return c1,c2

#ODOMETRY DATA
def Odom(msg):
    global x
    global y
    global theta

    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    rot_q=msg.pose.pose.orientation
    angles=euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    theta=angles[2]

def sayhi ():
            global m
            global xr
            global yr
            t = rospy.Time.now().to_sec() * math.pi
            if m==1:
                xr = 2.0 * math.cos(t/ESCALAR_FACTOR)
                yr = 2.0 * math.sin(t/ESCALAR_FACTOR)
            elif m==2:
                xr = 2.0 * math.sin(3*t/ESCALAR_FACTOR)
                yr = 2.0 * math.sin(t/ESCALAR_FACTOR)
            elif m==3:
                xr=2
                yr=2

            p_ref[0, 0] = xr
            p_ref[1, 0] = yr
            linear, angular = Control(theta, x, y, p_ref_ant, p_ref, [0.6, 0.6])
            p_ref_ant[0, 0], p_ref_ant[1, 0] = xr, yr

            #CONTROL SIGNAL SATURED
            if linear > 1.0:
                linear = 1
            elif linear < -0.5:
                linear = -0.5
            if angular > 1.7 :
                angular = 1.7
            elif angular < -1.7:
                angular = -1.7
            
            print("S. Control [u w]: [%s %s]"%(linear,angular))
            #SEND VELOCITIES
            cmd = geometry_msgs.msg.Twist()
            cmd.linear.x = linear
            cmd.angular.z = angular
            diff_vel.publish(cmd)

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_Form(object):
    def circle(self):
        global m
        m=1

    def eigth(self):
        global m
        m=2

    def heart(self):
        global m
        m=3

    def setupUi(self, Form):
        Form.setObjectName(_fromUtf8("Form"))
        Form.resize(400, 300)
        l = QtGui.QVBoxLayout(Form)
        sc = MyStaticMplCanvas(Form, width=5, height=4, dpi=100)
        l.addWidget(sc)
        self.pushButton = QtGui.QPushButton(Form)
        self.pushButton.setGeometry(QtCore.QRect(10, 30, 89, 25))
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.pushButton_2 = QtGui.QPushButton(Form)
        self.pushButton_2.setGeometry(QtCore.QRect(150, 30, 89, 25))
        self.pushButton_2.setObjectName(_fromUtf8("pushButton_2"))
        self.pushButton_3 = QtGui.QPushButton(Form)
        self.pushButton_3.setGeometry(QtCore.QRect(300, 30, 89, 25))
        self.pushButton_3.setObjectName(_fromUtf8("pushButton_3"))

        self.retranslateUi(Form)
        QtCore.QObject.connect(self.pushButton, QtCore.SIGNAL(_fromUtf8("clicked()")),self.circle)
        QtCore.QObject.connect(self.pushButton_2, QtCore.SIGNAL(_fromUtf8("clicked()")),self.eigth)
        QtCore.QObject.connect(self.pushButton_3, QtCore.SIGNAL(_fromUtf8("clicked()")),self.heart)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(_translate("Form", "Form", None))
        self.pushButton.setText(_translate("Form", "Circulo", None))
        self.pushButton_2.setText(_translate("Form", "Ocho", None))
        self.pushButton_3.setText(_translate("Form", "Corazon", None))


if __name__ == "__main__":
    import sys
    #NODE DEFINITION
    rospy.init_node('trajectory_control')
    sub_odom = rospy.Subscriber('/mobile_base_controller/odom',Odometry,Odom)
    diff_vel = rospy.Publisher('/mobile_base_controller/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
    p_ref_ant[0, 0], p_ref_ant[1, 0] =xr ,yr
    rate = rospy.Rate(10.0)
    t = RecurringTimer(0.1, sayhi)
    t.start_timer()
    app = QtGui.QApplication(sys.argv)
    Form = QtGui.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())
    

