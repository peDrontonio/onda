#!/usr/bin/env python3
"""
Interactive disturbance control GUI
Allows user to apply disturbances with sliders

Author: Manipulator Test Team
"""

import rospy
from manipulator_msgs.srv import ApplyDisturbance, ApplyDisturbanceRequest
from geometry_msgs.msg import Vector3
import sys

try:
    from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                                 QHBoxLayout, QLabel, QSlider, QPushButton, QGroupBox)
    from PyQt5.QtCore import Qt
except ImportError:
    print("ERROR: PyQt5 not installed. Install with: pip3 install PyQt5")
    sys.exit(1)


class DisturbanceControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Manipulator Disturbance Control")
        self.setGeometry(100, 100, 500, 400)
        
        self.disturbance_active = False
        self.current_force = Vector3(0.0, 0.0, 0.0)
        
        # ROS setup
        rospy.init_node('disturbance_control_gui', anonymous=True)
        self.service_name = '/apply_disturbance'
        
        # Wait for service
        rospy.loginfo(f"Waiting for {self.service_name} service...")
        try:
            rospy.wait_for_service(self.service_name, timeout=5.0)
        except rospy.ROSException:
            rospy.logerr(f"Service {self.service_name} not available!")
            sys.exit(1)
        
        self.service_proxy = rospy.ServiceProxy(self.service_name, ApplyDisturbance)
        
        self.init_ui()
        
    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout()
        
        # Title
        title = QLabel("Manipulator Disturbance Control")
        title.setStyleSheet("font-size: 18px; font-weight: bold;")
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)
        
        # Force controls group
        force_group = QGroupBox("Force Settings (Newtons)")
        force_layout = QVBoxLayout()
        
        # X Force slider
        self.fx_slider, fx_layout = self.create_slider("Force X:", -100, 100, 0)
        force_layout.addLayout(fx_layout)
        
        # Y Force slider
        self.fy_slider, fy_layout = self.create_slider("Force Y:", -100, 100, 0)
        force_layout.addLayout(fy_layout)
        
        # Z Force slider
        self.fz_slider, fz_layout = self.create_slider("Force Z:", -100, 100, 0)
        force_layout.addLayout(fz_layout)
        
        force_group.setLayout(force_layout)
        main_layout.addWidget(force_group)
        
        # Apply/Remove button
        self.toggle_button = QPushButton("Apply Disturbance")
        self.toggle_button.setStyleSheet("font-size: 14px; padding: 10px;")
        self.toggle_button.clicked.connect(self.toggle_disturbance)
        main_layout.addWidget(self.toggle_button)
        
        # Status label
        self.status_label = QLabel("Status: Idle")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("font-size: 12px; padding: 10px;")
        main_layout.addWidget(self.status_label)
        
        central_widget.setLayout(main_layout)
        
    def create_slider(self, label_text, min_val, max_val, default_val):
        layout = QHBoxLayout()
        
        label = QLabel(label_text)
        label.setFixedWidth(70)
        layout.addWidget(label)
        
        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(min_val)
        slider.setMaximum(max_val)
        slider.setValue(default_val)
        slider.setTickPosition(QSlider.TicksBelow)
        slider.setTickInterval(20)
        layout.addWidget(slider)
        
        value_label = QLabel(f"{default_val} N")
        value_label.setFixedWidth(60)
        layout.addWidget(value_label)
        
        slider.valueChanged.connect(lambda v, lbl=value_label: lbl.setText(f"{v} N"))
        
        return slider, layout
        
    def toggle_disturbance(self):
        try:
            # Get current slider values
            fx = float(self.fx_slider.value())
            fy = float(self.fy_slider.value())
            fz = float(self.fz_slider.value())
            
            # Create request
            req = ApplyDisturbanceRequest()
            req.force.x = fx
            req.force.y = fy
            req.force.z = fz
            
            # Call service
            response = self.service_proxy(req)
            
            if response.success:
                self.disturbance_active = not self.disturbance_active
                
                if self.disturbance_active:
                    self.toggle_button.setText("Remove Disturbance")
                    self.toggle_button.setStyleSheet(
                        "font-size: 14px; padding: 10px; background-color: #E53935; color: white;"
                    )
                    self.status_label.setText(f"Status: Active - Force [{fx}, {fy}, {fz}] N")
                    self.status_label.setStyleSheet(
                        "font-size: 12px; padding: 10px; background-color: #FFCDD2;"
                    )
                else:
                    self.toggle_button.setText("Apply Disturbance")
                    self.toggle_button.setStyleSheet("font-size: 14px; padding: 10px;")
                    self.status_label.setText("Status: Idle")
                    self.status_label.setStyleSheet("font-size: 12px; padding: 10px;")
                    
                rospy.loginfo(response.message)
            else:
                rospy.logerr(f"Service failed: {response.message}")
                self.status_label.setText(f"ERROR: {response.message}")
                self.status_label.setStyleSheet(
                    "font-size: 12px; padding: 10px; background-color: #FFCCCC;"
                )
                
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            self.status_label.setText(f"ERROR: Service call failed")


def main():
    app = QApplication(sys.argv)
    gui = DisturbanceControlGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
