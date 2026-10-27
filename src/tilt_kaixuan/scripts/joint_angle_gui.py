#!/usr/bin/env python3
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


JOINT_NAMES = [
    *(f"left_joint{i}" for i in range(1, 8)),
    *(f"right_joint{i}" for i in range(1, 8)),
]

HANGING_POSE_DEG = {
    "left_joint2": -10.0,
    "right_joint2": -10.0,
}

# Raw URDF joint positions corresponding to a displayed/calibrated angle of 0°.
JOINT_ZERO_OFFSETS_DEG = {
    "left_joint2": 180.0,
    "right_joint1": 180.0,
    "right_joint2": -160.0,
    "right_joint3": 180.0,
}

JOINT_LIMITS_DEG = {
    "left_joint7": (-180.0, 180.0),
}


class JointAngleNode(Node):
    def __init__(self):
        super().__init__("joint_angle_input")
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)

    def publish_angles(self, angles_deg):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = JOINT_NAMES
        message.position = [
            math.radians(angles_deg[name] + JOINT_ZERO_OFFSETS_DEG.get(name, 0.0))
            for name in JOINT_NAMES
        ]
        self.publisher.publish(message)


class JointAngleWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.inputs = {}
        self.setWindowTitle("URDF 关节角度输入")

        root = QWidget()
        root_layout = QVBoxLayout(root)

        arms_layout = QHBoxLayout()
        arms_layout.addWidget(self.make_arm_group("左臂", "left"))
        arms_layout.addWidget(self.make_arm_group("右臂", "right"))
        root_layout.addLayout(arms_layout)

        help_label = QLabel("单位：度（标定后角度）。点击数值框后可直接键盘输入，按 Enter 生效。")
        root_layout.addWidget(help_label)

        buttons = QHBoxLayout()
        hanging_button = QPushButton("恢复下垂默认值")
        hanging_button.clicked.connect(self.set_hanging_pose)
        zero_button = QPushButton("全部置零")
        zero_button.clicked.connect(self.set_zero_pose)
        buttons.addWidget(hanging_button)
        buttons.addWidget(zero_button)
        root_layout.addLayout(buttons)

        self.setCentralWidget(root)
        self.set_hanging_pose()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.publish)
        self.timer.start(50)

    def make_arm_group(self, title, prefix):
        group = QGroupBox(title)
        layout = QGridLayout(group)
        layout.addWidget(QLabel("关节"), 0, 0)
        layout.addWidget(QLabel("角度"), 0, 1)
        for index in range(1, 8):
            name = f"{prefix}_joint{index}"
            angle_input = QDoubleSpinBox()
            lower, upper = JOINT_LIMITS_DEG.get(name, (-180.0, 180.0))
            angle_input.setRange(lower, upper)
            angle_input.setDecimals(3)
            angle_input.setSingleStep(1.0)
            angle_input.setSuffix(" °")
            angle_input.setKeyboardTracking(False)
            angle_input.setMinimumWidth(130)
            self.inputs[name] = angle_input
            layout.addWidget(QLabel(name), index, 0)
            layout.addWidget(angle_input, index, 1)
        return group

    def set_hanging_pose(self):
        for name, angle_input in self.inputs.items():
            angle_input.setValue(HANGING_POSE_DEG.get(name, 0.0))
        self.publish()

    def set_zero_pose(self):
        for angle_input in self.inputs.values():
            angle_input.setValue(0.0)
        self.publish()

    def publish(self):
        angles = {name: angle_input.value() for name, angle_input in self.inputs.items()}
        self.node.publish_angles(angles)

    def closeEvent(self, event):
        self.timer.stop()
        event.accept()


def main():
    rclpy.init()
    node = JointAngleNode()
    app = QApplication(sys.argv)
    window = JointAngleWindow(node)
    window.show()
    exit_code = app.exec_()
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
