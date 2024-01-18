#!/usr/bin/env python
# coding:utf-8


import sys

from PyQt5.QtWidgets import*
from PyQt5.QtCore import *
from ui.main_window import *

import rospy
from rosgraph_msgs.msg import Clock
from sss_sim_env.srv import SimClockControl
import threading

class RosCommNode(QObject):
    signal_clock = pyqtSignal(object)
    def __init__(self):
        super(RosCommNode,self).__init__()
        self.clock_service_available = False
        self.clock_sub = rospy.Subscriber("/clock", Clock, self.cb_clock)
        # rospy.wait_for_service('/sss_clock_control', 0.5) # it will block the GUI
        try:
            self.clock_service = rospy.ServiceProxy('/sss_clock_control', SimClockControl)   
        except rospy.ServiceException as e:
            print(e)     
    
    def call_start(self):
        if not self.clock_service_available:
            rospy.wait_for_service('/sss_clock_control', 0.5)
            self.clock_service_available = True
        ret = self.clock_service(proceed=True)
        if ret.success is False:
            print("[ClockUI] call /sss_clock_control service with false return")

    def call_pause(self):
        if not self.clock_service_available:
            rospy.wait_for_service('/sss_clock_control', 0.5)
            self.clock_service_available = True
        ret = self.clock_service(proceed=False)
        if ret.success is False:
            print("[ClockUI] call /sss_clock_control service with false return")
    
    def call_speed_change(self, dial_value):
        new_speed = 0.0
        if dial_value < 10:
            new_speed = 0.5 + (dial_value - 0) / 10 * (1 - 0.5)
        elif dial_value < 20:
            new_speed = 1 + (dial_value - 10) / 10 * (2 - 1)
        elif dial_value < 30:
            new_speed = 2 + (dial_value - 20) / 10 * (10 - 2)
        elif dial_value < 40:
            new_speed = 10 + (dial_value - 30) / 10 * (20 - 10)
        else:
            new_speed = 20 + (dial_value - 40) / 10 * (100 - 20)
        print ("[ClockUI] request new speed {}x".format(new_speed))
        ret = self.clock_service(proceed=True, max_sim_speed=new_speed)
        if ret.success is False:
            print("[ClockUI] call /sss_clock_control service with false return")

    def cb_clock(self,msg):
        # print("clock: {}".format(msg.clock.to_sec()))
        self.signal_clock.emit(msg.clock.to_sec())


class MainWindow(QMainWindow,Ui_MainWindow):
    def __init__(self):
        super(MainWindow,self).__init__()
        self.setupUi(self)
    def set_clock_text(self, msg):
        self.label_time.setText(str(msg) + 's')


def run_spin():
    print("run spin")
    # rospy.spin()

if __name__ == '__main__':
    ### Create ros node
    rospy.init_node("clock_ui_pyqt")
    ### Create QApplication window
    app = QApplication(sys.argv)
    w = MainWindow()
    # w.resize(400, 200)
    # w.move(300, 300)
    w.setWindowTitle("Sim Clock")
    w.show()

    ros_node = RosCommNode()

    ### Connect to button signals
    w.startButton.clicked.connect(lambda:ros_node.call_start())
    w.pauseButton.clicked.connect(lambda:ros_node.call_pause())
    w.speed_dial.valueChanged.connect(lambda:ros_node.call_speed_change(w.speed_dial.value()))

    ### Connect to subscriber signals
    ros_node.signal_clock.connect(w.set_clock_text)

    print("Sim Clock UI")
    th_apin= threading.Thread(target=run_spin)
    th_apin.start()

    # Block the main funcation. Release resourses.
    sys.exit(app.exec_())