import json
import math
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
)
from pyqtlet2 import L, MapWidget as LeafletMapWidget

BG = '#F6F3EE'
BORDER_STRONG = '#C9C2B4'
TEXT_PRIMARY = '#2B2924'
TEXT_SECONDARY = '#7A7566'
C_BLUE = '#D9E9FA'
C_CORAL = '#FBE2D6'
C_TEAL = '#DDF2EA'

ROUTE_COLOR = '#378ADD'
START_COLOR = '#888780'
ROVER_COLOR = '#1D9E75'
ROVER_STROKE = '#04342C'
WP_FILL = '#F0997B'
WP_STROKE = '#4A1B0C'
PICK_COLORS = ['#1D4ED8', '#059669']  # 1st pick, 2nd pick ring colors
MEASURE_LINE_COLOR = '#1D4ED8'


def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


WAYPOINT_CLICK_RADIUS_M = 35  # how close a map click needs to be to a marker


class OperatorGuiNode(Node):
    """The ROS 2 side of the operator GUI: subscribes to telemetry and
    publishes operator commands. Holds no Qt widgets itself -- it just
    stores the latest state and exposes publish_command() for the window
    to call from button/row handlers.
    """

    def __init__(self):
        super().__init__('operator_gui_node')

        self.latest_state = None

        self.command_pub = self.create_publisher(String, '/rover/command', 10)
        self.create_subscription(String, '/rover/state', self.on_state, 10)

    def on_state(self, msg: String):
        try:
            self.latest_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'Ignoring malformed state: {msg.data!r}')
            return

    def publish_command(self, command: str):
        msg = String()
        msg.data = json.dumps({'command': command})
        self.command_pub.publish(msg)
        self.get_logger().info(f'Sent command: {command}')


class WaypointRow(QWidget):
    def __init__(self, name, index, on_click):
        super().__init__()
        self.index = index
        self.on_click = on_click
        self.setCursor(Qt.PointingHandCursor)

        self.name_label = QLabel(name)
        self.name_label.setStyleSheet(f'color: {TEXT_PRIMARY}; font-size: 13px;')
        self.name_label.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.dist_label = QLabel('')
        self.dist_label.setStyleSheet(f'color: {TEXT_SECONDARY}; font-size: 12px;')
        self.dist_label.setAttribute(Qt.WA_TransparentForMouseEvents)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.addWidget(self.name_label)
        layout.addStretch()
        layout.addWidget(self.dist_label)

        self.set_selected(False)

    def set_selected(self, selected):
        bg = 'rgba(255, 255, 255, 0.4)' if selected else 'transparent'
        self.setStyleSheet(f'background: {bg}; border-radius: 6px;')

    def set_distance(self, text):
        self.dist_label.setText(text)

    def mousePressEvent(self, event):
        self.on_click(self.index)


def card(bg_color, extra_style=''):
    widget = QWidget()
    widget.setStyleSheet(f'background: {bg_color}; border-radius: 12px; {extra_style}')
    return widget


class OperatorWindow(QWidget):
    def __init__(self, ros_node: OperatorGuiNode):
        super().__init__()
        self.ros_node = ros_node
        self.picked_indices = []  # up to 2 waypoint indices, for distance measurement
        self.rows = []
        self.map_ready = False
        self.waypoint_markers = []
        self.waypoint_positions = []
        self.waypoint_names = []
        self.rover_marker = None
        self.measure_line = None
        self.last_popup_text = []

        self.setWindowTitle('URC Operator Console')
        self.setStyleSheet(f'background: {BG};')
        self.resize(1100, 700)

        root = QHBoxLayout(self)
        root.setSpacing(16)

        content = QVBoxLayout()
        content.setSpacing(12)

        top_row = QHBoxLayout()
        top_row.setSpacing(12)
        top_row.addWidget(self._build_map_card(), 22)
        top_row.addWidget(self._build_waypoints_card(), 10)
        content.addLayout(top_row)

        content.addWidget(self._build_distance_bar())

        content_widget = QWidget()
        content_widget.setLayout(content)
        root.addWidget(content_widget, 1)

        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self.refresh)
        self.refresh_timer.start(200)

    def _build_map_card(self):
        self.map_card = card(C_BLUE)
        self.map_card.setFixedHeight(560)
        layout = QVBoxLayout(self.map_card)
        layout.setContentsMargins(12, 12, 12, 12)

        title = QLabel('Map')
        title.setStyleSheet(f'color: {TEXT_SECONDARY}; font-size: 12px;')
        layout.addWidget(title)

        self.leaflet_widget = LeafletMapWidget()
        layout.addWidget(self.leaflet_widget)

        self.leaflet_map = L.map(self.leaflet_widget)
        L.tileLayer(
            'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
            {'attribution': '© OpenStreetMap contributors', 'maxZoom': 19},
        ).addTo(self.leaflet_map)
        # Markers registered with the map's webchannel after the initial
        # connection don't get their own click signals reliably wired up in
        # pyqtlet2, so route all map clicks through the Map object's own
        # (reliable, first-registered) click signal and hit-test in Python.
        self.leaflet_map.clicked.connect(self.on_map_clicked)
        return self.map_card

    def on_map_clicked(self, event):
        latlng = event.get('latlng', {})
        lat, lon = latlng.get('lat'), latlng.get('lng')
        if lat is None or lon is None or not self.waypoint_positions:
            return
        distances = [haversine_m(lat, lon, wlat, wlon) for wlat, wlon in self.waypoint_positions]
        closest = min(range(len(distances)), key=lambda i: distances[i])
        if distances[closest] <= WAYPOINT_CLICK_RADIUS_M:
            self.on_select(closest)

    def _init_leaflet_map(self, start, waypoints):
        points = [(start['lat'], start['lon'])] + [(wp['lat'], wp['lon']) for wp in waypoints]

        self.leaflet_map.setView([start['lat'], start['lon']], 16)
        L.polyline(
            [[lat, lon] for lat, lon in points],
            {'color': ROUTE_COLOR, 'weight': 2, 'dashArray': '4, 6'},
        ).addTo(self.leaflet_map)
        L.circleMarker([start['lat'], start['lon']], {
            'radius': 5, 'color': START_COLOR, 'fillColor': START_COLOR,
            'fillOpacity': 1, 'weight': 1,
        }).addTo(self.leaflet_map)

        self.waypoint_positions = [(wp['lat'], wp['lon']) for wp in waypoints]
        self.waypoint_names = [wp['name'] for wp in waypoints]

        self.waypoint_markers = []
        for wp in waypoints:
            marker = L.circleMarker([wp['lat'], wp['lon']], {
                'radius': 8, 'color': WP_STROKE, 'fillColor': WP_FILL,
                'fillOpacity': 1, 'weight': 2,
            }).addTo(self.leaflet_map)
            marker.bindPopup(wp['name'])
            self.waypoint_markers.append(marker)
        self.last_popup_text = [None] * len(waypoints)

        self.rover_marker = L.circleMarker([start['lat'], start['lon']], {
            'radius': 7, 'color': ROVER_STROKE, 'fillColor': ROVER_COLOR,
            'fillOpacity': 1, 'weight': 1.5,
        }).addTo(self.leaflet_map)

        self.map_ready = True

    def _style_waypoint_marker(self, index):
        marker = self.waypoint_markers[index]
        if index in self.picked_indices:
            rank = self.picked_indices.index(index)
            style = {'radius': 11, 'color': PICK_COLORS[rank], 'fillColor': WP_FILL,
                     'fillOpacity': 1, 'weight': 4}
        else:
            style = {'radius': 8, 'color': WP_STROKE, 'fillColor': WP_FILL,
                     'fillOpacity': 1, 'weight': 2}
        marker.runJavaScriptForMapIndex(f'{marker.layerName}.setStyle({json.dumps(style)})')

    def _update_measurement(self):
        if self.measure_line is not None:
            self.leaflet_map.removeLayer(self.measure_line)
            self.measure_line = None

        if len(self.picked_indices) == 2:
            i, j = self.picked_indices
            lat1, lon1 = self.waypoint_positions[i]
            lat2, lon2 = self.waypoint_positions[j]
            self.measure_line = L.polyline(
                [[lat1, lon1], [lat2, lon2]],
                {'color': MEASURE_LINE_COLOR, 'weight': 3},
            ).addTo(self.leaflet_map)
            dist = haversine_m(lat1, lon1, lat2, lon2)
            self.measure_label.setText(
                f'{self.waypoint_names[i]} ↔ {self.waypoint_names[j]}: {dist:.1f} m')
        elif len(self.picked_indices) == 1:
            self.measure_label.setText(f'{self.waypoint_names[self.picked_indices[0]]} picked — pick one more')
        else:
            self.measure_label.setText('Pick two waypoints to measure distance')

    def _build_waypoints_card(self):
        waypoints_card = card(C_CORAL)
        waypoints_card.setFixedHeight(560)
        layout = QVBoxLayout(waypoints_card)
        layout.setContentsMargins(12, 12, 12, 12)

        title = QLabel('Waypoints')
        title.setStyleSheet(f'color: {TEXT_SECONDARY}; font-size: 12px;')
        layout.addWidget(title)

        self.rows_layout = QVBoxLayout()
        self.rows_layout.setSpacing(6)
        layout.addLayout(self.rows_layout)
        layout.addStretch()

        self.measure_label = QLabel('Pick two waypoints to measure distance')
        self.measure_label.setWordWrap(True)
        self.measure_label.setStyleSheet(f'color: {TEXT_PRIMARY}; font-size: 12px;')
        layout.addWidget(self.measure_label)
        return waypoints_card

    def _build_distance_bar(self):
        bar = card(C_TEAL)
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(16, 14, 16, 14)

        text_col = QVBoxLayout()
        title = QLabel('Distance to next waypoint')
        title.setStyleSheet(f'color: {TEXT_SECONDARY}; font-size: 12px;')
        self.dist_value_label = QLabel('-- m')
        self.dist_value_label.setStyleSheet(
            f'color: {TEXT_PRIMARY}; font-size: 22px; font-weight: 500;')
        text_col.addWidget(title)
        text_col.addWidget(self.dist_value_label)
        layout.addLayout(text_col)
        layout.addStretch()

        ping_button = QPushButton('⟳  Ping rover')
        ping_button.setCursor(Qt.PointingHandCursor)
        ping_button.setStyleSheet(f'''
            QPushButton {{
                background: none;
                border: 0.5px solid {BORDER_STRONG};
                border-radius: 8px;
                padding: 8px 14px;
                font-size: 13px;
                color: {TEXT_PRIMARY};
            }}
            QPushButton:hover {{ background: rgba(255, 255, 255, 0.4); }}
        ''')
        ping_button.clicked.connect(lambda: self.ros_node.publish_command('ping'))
        layout.addWidget(ping_button)
        return bar

    def on_select(self, index):
        if index in self.picked_indices:
            self.picked_indices.remove(index)
        else:
            if len(self.picked_indices) >= 2:
                self.picked_indices.pop(0)
            self.picked_indices.append(index)

        for row in self.rows:
            row.set_selected(row.index in self.picked_indices)
        if self.map_ready:
            for i in range(len(self.waypoint_markers)):
                self._style_waypoint_marker(i)
            self._update_measurement()

    def _ensure_rows(self, waypoints):
        if len(self.rows) == len(waypoints):
            return
        for row in self.rows:
            row.setParent(None)
        self.rows = []
        for i, wp in enumerate(waypoints):
            row = WaypointRow(wp['name'], i, self.on_select)
            self.rows_layout.addWidget(row)
            self.rows.append(row)

    def refresh(self):
        state = self.ros_node.latest_state
        if state is None:
            return

        waypoints = state.get('waypoints', [])
        start = state.get('start')
        if not waypoints or start is None:
            return
        self._ensure_rows(waypoints)
        if not self.map_ready:
            self._init_leaflet_map(start, waypoints)

        current_index = state.get('current_index', 0)
        mode = state['mode']
        distance_remaining = state['distance_remaining_m']
        last_idx = len(waypoints) - 1
        done_flags = [
            i < current_index or (mode == 'idle' and i == last_idx)
            for i in range(len(waypoints))
        ]

        for i, wp in enumerate(waypoints):
            if done_flags[i]:
                dist_text = '0.0 m'
            elif i == current_index:
                dist_text = f'{distance_remaining:.1f} m'
            else:
                dist_text = f"{wp['distance_m']:.1f} m"
            self.rows[i].set_distance(dist_text)

            popup_text = f"{wp['name']}: {dist_text}"
            if popup_text != self.last_popup_text[i]:
                self.waypoint_markers[i].bindPopup(popup_text)
                self.last_popup_text[i] = popup_text

        self.dist_value_label.setText(f'{distance_remaining:.1f} m')

        leg_total = waypoints[current_index]['distance_m'] if current_index < len(waypoints) else 1.0
        points = [(start['lat'], start['lon'])] + [(wp['lat'], wp['lon']) for wp in waypoints]
        from_pt = points[min(current_index, len(points) - 1)]
        to_pt = points[min(current_index + 1, len(points) - 1)]
        frac = 0.0
        if leg_total > 0:
            frac = max(0.0, min(1.0, 1 - distance_remaining / leg_total))
        rover_lat = from_pt[0] + (to_pt[0] - from_pt[0]) * frac
        rover_lon = from_pt[1] + (to_pt[1] - from_pt[1]) * frac
        # CircleMarker (unlike Marker) has no setLatLng() wrapper in pyqtlet2.
        self.rover_marker.runJavaScriptForMapIndex(
            f'{self.rover_marker.layerName}.setLatLng([{rover_lat}, {rover_lon}])')


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
