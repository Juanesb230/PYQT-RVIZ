from __future__ import unicode_literals
import os
import random
import sys

from PyQt4 import QtCore, QtGui
import roslib
roslib.load_manifest('diferencial')
import rospy
import math
import tf
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

from collections import deque

progname = os.path.basename(sys.argv[0])

ESCALAR_FACTOR=70.0
p_ref_ant=np.array([[0.0],[0.0]])
p_ref=np.array([[0.0],[0.0]])
x= 0.0
y= 0.0
theta=0.0
xr= 0.0
yr= 0.0
m=0
dx=deque(np.zeros(40))
dy=deque(np.zeros(40))
dxr=deque(np.zeros(40))
dyr=deque(np.zeros(40))
sample_time=0.1


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

    def __init__(self, *args, **kwargs):
        MyMplCanvas.__init__(self, *args, **kwargs)
        timercanvas = QtCore.QTimer(self)
        timercanvas.timeout.connect(self.update_figure)
        timercanvas.start(3000)
        
    def compute_initial_figure(self):
        global dx
        global dy
        self.axes.plot(dx, dy, 'r.')
        self.axes.set_xlim(-6.0, 6.0)
        self.axes.set_ylim(-6.0, 6.0)

    def update_figure(self):
        global dx
        global dy
        global x
        global y
        global m
        global xr
        global yr
        dx.popleft()
        dy.popleft()
        dx.append(x)
        dy.append(y)
        dxr.popleft()
        dyr.popleft()
        dxr.append(xr)
        dyr.append(yr)
        self.axes.cla()
        self.axes.set_xlim(-4.0, 4.0)
        self.axes.set_ylim(-4.0, 4.0)
        if m == 0:
            self.axes.plot(dx, dy, 'r.',label = 'Robot')
        else:
            self.axes.plot(dxr, dyr, 'b^',label = 'Reference')
            self.axes.plot(dx, dy, 'r.',label = 'Robot')
        self.axes.legend(loc ='upper right')
        self.draw()

class RecurringTimer(threading._Timer):
     
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
        xr = 2.0 * math.sin(t/ESCALAR_FACTOR)/(1+math.pow(math.cos(t/ESCALAR_FACTOR),2))
        yr = 2.0 * math.sin(t/ESCALAR_FACTOR) * math.cos(t/ESCALAR_FACTOR)/(1+math.pow(math.cos(t/ESCALAR_FACTOR),2))
    elif m==3:
        xr = 2.0 * math.sin(t/ESCALAR_FACTOR)/(1+math.pow(math.sin(t/ESCALAR_FACTOR),2))
        yr = 2.0 * math.sin(t/ESCALAR_FACTOR) * math.cos(t/ESCALAR_FACTOR)/(1+math.pow(math.sin(t/ESCALAR_FACTOR),2))

    br.sendTransform(( xr, yr, 0.0),
                    (0.0, 0.0, 0.0, 1.0),
                    rospy.Time.now(),
                    "ref",
                    "odom")

    p_ref[0, 0] = xr
    p_ref[1, 0] = yr
    linear, angular = Control(theta, x, y, p_ref_ant, p_ref, [0.6, 0.6])
    p_ref_ant[0, 0], p_ref_ant[1, 0] = xr, yr

    #CONTROL SIGNAL SATURED
    if linear > 1.0:
        linear = 1
    elif linear < -0.5:
        linear = -0.5
    if angular > 1.7:
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

class ChildWindow(QtGui.QWidget):
    def __init__(self):
        super(ChildWindow, self).__init__()
        timer.stop_timer()
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        diff_vel.publish(cmd)
        self.initUI()
 
    def initUI(self):
        global SpinBox_f
        global SpinBox_s
        btn1 = QtGui.QPushButton("Ok",self)
        btn1.move(195, 200)
        btn1.clicked.connect(self.ok)
        telebox = QtGui.QGroupBox("Teleoperation Settings",self)
        telebox.setCheckable(False)
        telebox.setGeometry(QtCore.QRect(10, 10, 300, 90))
        text_u = QtGui.QLineEdit("Lineal velocity [m/s]:",telebox)
        text_u.setEnabled(True)
        text_u.setGeometry(QtCore.QRect(10, 30, 170, 25))
        text_u.setReadOnly(True)
        text_u.setObjectName(_fromUtf8("text_u"))
        text_w = QtGui.QLineEdit("Angular velocity [rad/s]:",telebox)
        text_w.setEnabled(True)
        text_w.setGeometry(QtCore.QRect(10, 60, 170, 25))
        text_w.setReadOnly(True)
        text_w.setObjectName(_fromUtf8("text_w"))
        SpinBox_u = QtGui.QDoubleSpinBox(telebox)
        SpinBox_u.setGeometry(QtCore.QRect(200, 30, 69, 26))
        SpinBox_u.setDecimals(1)
        SpinBox_u.setMinimum(0.0)
        SpinBox_u.setMaximum(1.5)
        SpinBox_u.setSingleStep(0.1)
        SpinBox_u.setValue(0.2)
        SpinBox_u.setObjectName(_fromUtf8("SpinBox_u"))
        SpinBox_w = QtGui.QDoubleSpinBox(telebox)
        SpinBox_w.setGeometry(QtCore.QRect(200, 60, 69, 26))
        SpinBox_w.setDecimals(1)
        SpinBox_w.setMinimum(0.0)
        SpinBox_w.setMaximum(1.7)
        SpinBox_w.setSingleStep(0.1)
        SpinBox_w.setValue(0.2)
        SpinBox_w.setObjectName(_fromUtf8("SpinBox_w"))
        controlBox = QtGui.QGroupBox("Control Settings",self)
        controlBox.setCheckable(False)
        controlBox.setGeometry(QtCore.QRect(10, 100, 300, 90))
        text_f = QtGui.QLineEdit("Escalar factor time [u]:",controlBox)
        text_f.setEnabled(True)
        text_f.setGeometry(QtCore.QRect(10, 30, 170, 25))
        text_f.setReadOnly(True)
        text_f.setObjectName(_fromUtf8("text_f"))
        SpinBox_f = QtGui.QSpinBox(controlBox)
        SpinBox_f.setGeometry(QtCore.QRect(200, 30, 69, 26))
        SpinBox_f.setMinimum(10)
        SpinBox_f.setMaximum(100)
        SpinBox_f.setSingleStep(10)
        SpinBox_f.setValue(70)
        SpinBox_f.setObjectName(_fromUtf8("SpinBox_f"))
        self.setGeometry(1000, 200, 300, 240)
        self.setWindowTitle('Settings')
        self.show()
 
    def ok(self):
        global ESCALAR_FACTOR
        global timer
        global sample_time
        global SpinBox_f
        ESCALAR_FACTOR = float(SpinBox_f.value())
        self.close()


class Ui_Form(object):
    def __init__(self):
        super(Ui_Form, self).__init__()
        self.children = []

    def teleop(self):
        global dx
        global dy
        global dxr
        global dyr
        global m
        global timer
        self.groupBox_2.setEnabled(True)
        self.groupBox_3.setEnabled(False)
        self.groupBox_4.setEnabled(False)
        dx=deque(np.zeros(40))
        dy=deque(np.zeros(40))
        dxr=deque(np.zeros(40))
        dyr=deque(np.zeros(40))
        timer.stop_timer()
        m = 0
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        diff_vel.publish(cmd)
        

    def posture(self):
        global m
        global xr
        global yr
        global dx
        global dy
        global dxr
        global dyr
        xr=0.0
        yr=0.0
        m=4
        self.groupBox_2.setEnabled(False)
        self.groupBox_3.setEnabled(False)
        self.groupBox_4.setEnabled(True)
        dx=deque(np.zeros(40))
        dy=deque(np.zeros(40))
        dxr=deque(np.zeros(40))
        dyr=deque(np.zeros(40))
        timer.start_timer()

    def tray(self):
        global m
        global xr
        global yr
        global timer
        global dx
        global dy
        global dxr
        global dyr
        xr=0.0
        yr=0.0
        m=4
        self.radioButton_7.setChecked(True)
        self.groupBox_2.setEnabled(False)
        self.groupBox_3.setEnabled(True)
        self.groupBox_4.setEnabled(False)
        dx=deque(np.zeros(40))
        dy=deque(np.zeros(40))
        dxr=deque(np.zeros(40))
        dyr=deque(np.zeros(40))
        timer.start_timer()

    def circle(self):
        global m
        global dx
        global dy
        global dxr
        global dyr
        m=1
        dx=deque(np.zeros(40))
        dy=deque(np.zeros(40))
        dxr=deque(np.zeros(40))
        dyr=deque(np.zeros(40))

    def eigth(self):
        global m
        global dx
        global dy
        global dxr
        global dyr
        m=2
        dx=deque(np.zeros(40))
        dy=deque(np.zeros(40))
        dxr=deque(np.zeros(40))
        dyr=deque(np.zeros(40))

    def heart(self):
        global m
        global dx
        global dy
        global dxr
        global dyr
        m=3
        dx=deque(np.zeros(40))
        dy=deque(np.zeros(40))
        dxr=deque(np.zeros(40))
        dyr=deque(np.zeros(40))

    def init_pos(self):
        global m
        global xr
        global yr
        global dx
        global dy
        global dxr
        global dyr
        xr=0.0
        yr=0.0
        m=4
        dx=deque(np.zeros(40))
        dy=deque(np.zeros(40))
        dxr=deque(np.zeros(40))
        dyr=deque(np.zeros(40))

    def set_pos(self):
        global m
        global xr
        global yr
        m=4
        xr=float(self.doubleSpinBox.value())
        yr=float(self.doubleSpinBox_2.value())
    
    def forward(self):
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = 0.2
        cmd.angular.z = 0.0
        diff_vel.publish(cmd)

    def behind(self):
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = -0.2
        cmd.angular.z = 0.0
        diff_vel.publish(cmd)
    
    def rigth(self):
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = -0.2
        diff_vel.publish(cmd)

    def left(self):
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.2
        diff_vel.publish(cmd)

    def stop(self):
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        diff_vel.publish(cmd)

    def config(self):
        self.radioButton.setChecked(True)
        self.groupBox_2.setEnabled(True)
        self.groupBox_3.setEnabled(False)
        self.groupBox_4.setEnabled(False)
        child = ChildWindow()
        self.children.append(child)

    def setupUi(self, Form):
        Form.setObjectName(_fromUtf8("Form"))
        Form.resize(614, 660) 
        self.configButton = QtGui.QPushButton(Form)
        self.configButton.setGeometry(QtCore.QRect(550, 10, 30, 25))
        self.configButton.setObjectName(_fromUtf8("configButton"))
        self.groupBox = QtGui.QGroupBox(Form)
        self.groupBox.setGeometry(QtCore.QRect(10, 40, 241, 121))
        self.groupBox.setObjectName(_fromUtf8("groupBox"))
        self.radioButton = QtGui.QRadioButton(self.groupBox)
        self.radioButton.setGeometry(QtCore.QRect(20, 30, 121, 23))
        self.radioButton.setChecked(True)
        self.radioButton.setObjectName(_fromUtf8("radioButton"))
        self.radioButton_2 = QtGui.QRadioButton(self.groupBox)
        self.radioButton_2.setGeometry(QtCore.QRect(20, 60, 151, 23))
        self.radioButton_2.setObjectName(_fromUtf8("radioButton_2"))
        self.radioButton_3 = QtGui.QRadioButton(self.groupBox)
        self.radioButton_3.setGeometry(QtCore.QRect(20, 90, 211, 23))
        self.radioButton_3.setObjectName(_fromUtf8("radioButton_3"))
        self.groupBox_2 = QtGui.QGroupBox(Form)
        self.groupBox_2.setGeometry(QtCore.QRect(300, 40, 301, 121))
        self.groupBox_2.setObjectName(_fromUtf8("groupBox_2"))
        self.pushButton = QtGui.QPushButton(self.groupBox_2)
        self.pushButton.setGeometry(QtCore.QRect(120, 30, 71, 25))
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.pushButton_2 = QtGui.QPushButton(self.groupBox_2)
        self.pushButton_2.setGeometry(QtCore.QRect(220, 60, 71, 25))
        self.pushButton_2.setObjectName(_fromUtf8("pushButton_2"))
        self.pushButton_3 = QtGui.QPushButton(self.groupBox_2)
        self.pushButton_3.setGeometry(QtCore.QRect(120, 90, 71, 25))
        self.pushButton_3.setObjectName(_fromUtf8("pushButton_3"))
        self.pushButton_4 = QtGui.QPushButton(self.groupBox_2)
        self.pushButton_4.setGeometry(QtCore.QRect(20, 60, 71, 25))
        self.pushButton_4.setObjectName(_fromUtf8("pushButton_4"))
        self.pushButton_6 = QtGui.QPushButton(self.groupBox_2)
        self.pushButton_6.setGeometry(QtCore.QRect(120, 60, 71, 25))
        self.pushButton_6.setObjectName(_fromUtf8("pushButton_6"))
        self.groupBox_3 = QtGui.QGroupBox(Form)
        self.groupBox_3.setEnabled(False)
        self.groupBox_3.setGeometry(QtCore.QRect(10, 170, 241, 131))
        self.groupBox_3.setObjectName(_fromUtf8("groupBox_3"))
        self.radioButton_4 = QtGui.QRadioButton(self.groupBox_3)
        self.radioButton_4.setGeometry(QtCore.QRect(160, 30, 81, 23))
        self.radioButton_4.setCheckable(True)
        self.radioButton_4.setChecked(False)
        self.radioButton_4.setObjectName(_fromUtf8("radioButton_4"))
        self.radioButton_5 = QtGui.QRadioButton(self.groupBox_3)
        self.radioButton_5.setGeometry(QtCore.QRect(160, 90, 81, 23))
        self.radioButton_5.setChecked(False)
        self.radioButton_5.setObjectName(_fromUtf8("radioButton_5"))
        self.radioButton_6 = QtGui.QRadioButton(self.groupBox_3)
        self.radioButton_6.setGeometry(QtCore.QRect(20, 90, 112, 23))
        self.radioButton_6.setChecked(False)
        self.radioButton_6.setObjectName(_fromUtf8("radioButton_6"))
        self.radioButton_7 = QtGui.QRadioButton(self.groupBox_3)
        self.radioButton_7.setGeometry(QtCore.QRect(20, 30, 121, 23))
        self.radioButton_7.setChecked(True)
        self.radioButton_7.setObjectName(_fromUtf8("radioButton_7"))
        self.groupBox_4 = QtGui.QGroupBox(Form)
        self.groupBox_4.setEnabled(False)
        self.groupBox_4.setGeometry(QtCore.QRect(300, 170, 301, 131))
        self.groupBox_4.setObjectName(_fromUtf8("groupBox_4"))
        self.lineEdit = QtGui.QLineEdit(self.groupBox_4)
        self.lineEdit.setEnabled(True)
        self.lineEdit.setGeometry(QtCore.QRect(10, 30, 113, 25))
        self.lineEdit.setReadOnly(True)
        self.lineEdit.setObjectName(_fromUtf8("lineEdit"))
        self.lineEdit_2 = QtGui.QLineEdit(self.groupBox_4)
        self.lineEdit_2.setEnabled(True)
        self.lineEdit_2.setGeometry(QtCore.QRect(10, 60, 113, 25))
        self.lineEdit_2.setReadOnly(True)
        self.lineEdit_2.setObjectName(_fromUtf8("lineEdit_2"))
        self.doubleSpinBox = QtGui.QDoubleSpinBox(self.groupBox_4)
        self.doubleSpinBox.setGeometry(QtCore.QRect(180, 30, 69, 26))
        self.doubleSpinBox.setDecimals(1)
        self.doubleSpinBox.setMinimum(-4.0)
        self.doubleSpinBox.setMaximum(4.0)
        self.doubleSpinBox.setSingleStep(0.1)
        self.doubleSpinBox.setObjectName(_fromUtf8("doubleSpinBox"))
        self.doubleSpinBox_2 = QtGui.QDoubleSpinBox(self.groupBox_4)
        self.doubleSpinBox_2.setGeometry(QtCore.QRect(180, 60, 69, 26))
        self.doubleSpinBox_2.setDecimals(1)
        self.doubleSpinBox_2.setMinimum(-4.0)
        self.doubleSpinBox_2.setMaximum(4.0)
        self.doubleSpinBox_2.setSingleStep(0.1)
        self.doubleSpinBox_2.setObjectName(_fromUtf8("doubleSpinBox_2"))
        self.pushButton_5 = QtGui.QPushButton(self.groupBox_4)
        self.pushButton_5.setGeometry(QtCore.QRect(170, 100, 89, 25))
        self.pushButton_5.setObjectName(_fromUtf8("pushButton_5"))
        self.groupBox_5 = QtGui.QGroupBox(Form)
        self.groupBox_5.setEnabled(True)
        self.groupBox_5.setGeometry(QtCore.QRect(10, 320, 602, 300))
        self.groupBox_5.setObjectName(_fromUtf8("groupBox_4"))
        l = QtGui.QVBoxLayout(self.groupBox_5)
        sc = MyStaticMplCanvas(self.groupBox_5, width=5, height=4, dpi=100)
        l.addWidget(sc)

        self.retranslateUi(Form)
        QtCore.QObject.connect(self.radioButton, QtCore.SIGNAL(_fromUtf8("clicked()")),self.teleop)
        QtCore.QObject.connect(self.radioButton_2, QtCore.SIGNAL(_fromUtf8("clicked()")),self.posture)
        QtCore.QObject.connect(self.radioButton_3, QtCore.SIGNAL(_fromUtf8("clicked()")),self.tray)
        QtCore.QObject.connect(self.radioButton_4, QtCore.SIGNAL(_fromUtf8("clicked()")),self.circle)
        QtCore.QObject.connect(self.radioButton_5, QtCore.SIGNAL(_fromUtf8("clicked()")),self.eigth)
        QtCore.QObject.connect(self.radioButton_6, QtCore.SIGNAL(_fromUtf8("clicked()")),self.heart)
        QtCore.QObject.connect(self.radioButton_7, QtCore.SIGNAL(_fromUtf8("clicked()")),self.init_pos)
        QtCore.QObject.connect(self.pushButton_5, QtCore.SIGNAL(_fromUtf8("clicked()")),self.set_pos)
        QtCore.QObject.connect(self.pushButton, QtCore.SIGNAL(_fromUtf8("clicked()")),self.forward)
        QtCore.QObject.connect(self.pushButton_2, QtCore.SIGNAL(_fromUtf8("clicked()")),self.rigth)
        QtCore.QObject.connect(self.pushButton_3, QtCore.SIGNAL(_fromUtf8("clicked()")),self.behind)
        QtCore.QObject.connect(self.pushButton_4, QtCore.SIGNAL(_fromUtf8("clicked()")),self.left)
        QtCore.QObject.connect(self.pushButton_6, QtCore.SIGNAL(_fromUtf8("clicked()")),self.stop)
        QtCore.QObject.connect(self.configButton, QtCore.SIGNAL(_fromUtf8("clicked()")),self.config)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(_translate("Form", "Diferential Robot GUI", None))
        icon_forward = QtGui.QIcon("icons/flecha_arriba.png")
        icon_behind = QtGui.QIcon("icons/flecha_abajo.png")
        icon_stop = QtGui.QIcon("icons/stop-player-button.png")
        icon_left = QtGui.QIcon("icons/left_rotation.png")
        icon_rigth = QtGui.QIcon("icons/rigth_rotation.png")
        icon_config = QtGui.QIcon("icons/config.png")
        self.configButton.setIcon(icon_config)
        self.groupBox.setTitle(_translate("Form", "Mode", None))
        self.radioButton.setText(_translate("Form", "Teleoperation", None))
        self.radioButton_2.setText(_translate("Form", "Posture Control", None))
        self.radioButton_3.setText(_translate("Form", "Trajectory Control", None))
        self.groupBox_2.setTitle(_translate("Form", "Teleoperation", None))
        self.pushButton.setIcon(icon_forward)
        self.pushButton_2.setIcon(icon_rigth)
        self.pushButton_3.setIcon(icon_behind)
        self.pushButton_4.setIcon(icon_left)
        self.pushButton_6.setIcon(icon_stop)
        self.groupBox_3.setTitle(_translate("Form", "Trajectory Control", None))
        self.radioButton_4.setText(_translate("Form", "Circle", None))
        self.radioButton_5.setText(_translate("Form", "Eigth", None))
        self.radioButton_6.setText(_translate("Form", "Heart", None))
        self.radioButton_7.setText(_translate("Form", "Initial Position", None))
        self.groupBox_4.setTitle(_translate("Form", "Posture Control", None))
        self.lineEdit.setText(_translate("Form", "x Value (m):", None))
        self.lineEdit_2.setText(_translate("Form", "y Value (m):", None))
        self.pushButton_5.setText(_translate("Form", "Ok", None))
        self.groupBox_5.setTitle(_translate("Form", "Axes", None))


if __name__ == "__main__":
    import sys
    rospy.init_node('trajectory_control')
    br = tf.TransformBroadcaster()
    sub_odom = rospy.Subscriber('/mobile_base_controller/odom',Odometry,Odom)
    diff_vel = rospy.Publisher('/mobile_base_controller/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
    timer = RecurringTimer(0.1, sayhi)
    app = QtGui.QApplication(sys.argv)
    Form = QtGui.QMainWindow()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())

