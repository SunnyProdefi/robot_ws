#!/usr/bin/env python3
import sys
import rospy
from sensor_msgs.msg import JointState
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QTableWidget, QTableWidgetItem
from PyQt5.QtCore import QTimer
import math

class JointStateViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Joint Angles Viewer (°)")
        self.resize(600, 400)

        self.joint_data = {}
        self.table = QTableWidget()
        self.table.setColumnCount(3)
        self.table.setHorizontalHeaderLabels(["Joint Name", "Angle (°)", "Angle (rad)"])


        layout = QVBoxLayout()
        layout.addWidget(self.table)
        self.setLayout(layout)

        rospy.Subscriber("/joint_states", JointState, self.joint_callback)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_table)
        self.timer.start(100)

    def joint_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joint_data[name] = math.degrees(pos)

    def update_table(self):
        self.table.setRowCount(len(self.joint_data))
        for i, (name, angle) in enumerate(sorted(self.joint_data.items())):
            self.table.setItem(i, 0, QTableWidgetItem(name))
            self.table.setItem(i, 1, QTableWidgetItem(f"{angle:.2f}"))
            self.table.setItem(i, 2, QTableWidgetItem(f"{math.radians(angle):.4f}"))


if __name__ == "__main__":
    rospy.init_node("joint_state_gui_viewer", anonymous=True)
    app = QApplication(sys.argv)
    viewer = JointStateViewer()
    viewer.show()
    sys.exit(app.exec_())
