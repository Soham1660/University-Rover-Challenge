import json
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
)

# If /rover/state hasn't been received in this long, the link banner goes red.
LINK_TIMEOUT_S = 2.0

LED_COLORS = {
    'blue': '#2196f3',
    'green': '#4caf50',
    'red': '#f44336',
}


class OperatorGuiNode(Node):
    """The ROS 2 side of the operator GUI: subscribes to telemetry and
    publishes operator commands. Holds no Qt widgets itself -- it just
    stores the latest state and exposes publish_command() for the window
    to call from button handlers.
    """

    def __init__(self):
        super().__init__('operator_gui_node')

        self.latest_state = None
        self.last_state_time = None

        self.command_pub = self.create_publisher(String, '/rover/command', 10)
        self.create_subscription(String, '/rover/state', self.on_state, 10)

    def on_state(self, msg: String):
        try:
            self.latest_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'Ignoring malformed state: {msg.data!r}')
            return
        self.last_state_time = self.get_clock().now()

    def publish_command(self, command: str):
        msg = String()
        msg.data = json.dumps({'command': command})
        self.command_pub.publish(msg)
        self.get_logger().info(f'Sent command: {command}')


class OperatorWindow(QWidget):
    """The Qt side: builds the widgets and refreshes them from whatever
    OperatorGuiNode has most recently received.
    """

    def __init__(self, ros_node: OperatorGuiNode):
        super().__init__()
        self.ros_node = ros_node

        self.setWindowTitle('URC Operator Console')

        self.link_banner = QLabel('NO SIGNAL')
        self.link_banner.setAlignment(Qt.AlignCenter)

        self.mode_label = QLabel('Mode: --')
        self.target_label = QLabel('Target: --')
        self.distance_label = QLabel('Distance remaining: --')
        self.gnss_label = QLabel('GNSS: --')

        self.led = QLabel()
        self.led.setFixedSize(24, 24)
        self.led.setStyleSheet('border-radius: 12px; background-color: #555;')

        led_row = QHBoxLayout()
        led_row.addWidget(QLabel('LED:'))
        led_row.addWidget(self.led)
        led_row.addStretch()

        abort_button = QPushButton('ABORT')
        abort_button.clicked.connect(lambda: self.ros_node.publish_command('abort'))

        return_button = QPushButton('RETURN TO PREVIOUS')
        return_button.clicked.connect(
            lambda: self.ros_node.publish_command('return_to_previous'))

        button_row = QHBoxLayout()
        button_row.addWidget(abort_button)
        button_row.addWidget(return_button)

        layout = QVBoxLayout()
        layout.addWidget(self.link_banner)
        layout.addWidget(self.mode_label)
        layout.addWidget(self.target_label)
        layout.addWidget(self.distance_label)
        layout.addWidget(self.gnss_label)
        layout.addLayout(led_row)
        layout.addLayout(button_row)
        self.setLayout(layout)

        # Redraw from the node's latest data on a steady clock, independent
        # of how often /rover/state actually arrives.
        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self.refresh)
        self.refresh_timer.start(200)  # 5 Hz UI refresh

    def refresh(self):
        state = self.ros_node.latest_state
        last_time = self.ros_node.last_state_time

        if last_time is not None:
            age_s = (self.ros_node.get_clock().now() - last_time).nanoseconds / 1e9
        else:
            age_s = None

        if age_s is None or age_s > LINK_TIMEOUT_S:
            self.link_banner.setText('NO SIGNAL')
            self.link_banner.setStyleSheet(
                'background-color: #f44336; color: white; font-weight: bold; padding: 6px;')
        else:
            self.link_banner.setText('LINK OK')
            self.link_banner.setStyleSheet(
                'background-color: #4caf50; color: white; font-weight: bold; padding: 6px;')

        if state is None:
            return

        self.mode_label.setText(f'Mode: {state["mode"]}')
        self.target_label.setText(
            f'Target: {state["target_name"]} ({state["target_type"]})')
        self.distance_label.setText(
            f'Distance remaining: {state["distance_remaining_m"]} m')
        self.gnss_label.setText(
            f'GNSS: {state["gnss_lat"]:.5f}, {state["gnss_lon"]:.5f}')

        color = LED_COLORS.get(state['led_state'], '#555')
        self.led.setStyleSheet(f'border-radius: 12px; background-color: {color};')


def main():
    rclpy.init()
    ros_node = OperatorGuiNode()

    app = QApplication(sys.argv)
    window = OperatorWindow(ros_node)
    window.show()

    # Qt owns the event loop here (app.exec_() blocks), so ROS callbacks
    # can't be serviced by a blocking rclpy.spin(). Instead, a QTimer pumps
    # spin_once() on a fast interval, letting Qt's loop and ROS's callback
    # processing interleave on the same thread.
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    ros_timer.start(50)  # service ROS callbacks at ~20 Hz

    exit_code = app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
