import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# The mission is just a list of waypoints the "rover" drives to in order.
# Each one has a human-readable name, a type (as URC missions have distinct
# legs: autonomous navigation posts, GNSS-only markers, equipment servicing
# stations, etc.), a starting distance in meters, and a fake GNSS fix.
WAYPOINTS = [
    {'name': 'Gate 1', 'type': 'gnss_post', 'distance_m': 120.0, 'lat': 38.4062, 'lon': -110.7917},
    {'name': 'Gate 2', 'type': 'gnss_post', 'distance_m': 85.0, 'lat': 38.4070, 'lon': -110.7925},
    {'name': 'Equipment Servicing', 'type': 'equipment', 'distance_m': 60.0, 'lat': 38.4078, 'lon': -110.7933},
    {'name': 'Final Marker', 'type': 'gnss_post', 'distance_m': 40.0, 'lat': 38.4085, 'lon': -110.7940},
]

# Where the rover starts before the first waypoint -- not itself a mission
# leg, just an anchor so the GUI's map has somewhere to draw the first route
# segment from.
START = {'lat': 38.4055, 'lon': -110.7908}

# How fast the fake rover "closes distance" on each tick, in meters/tick.
DRIVE_RATE_M = 4.0


class RoverSimNode(Node):
    """Simulates a rover running an autonomous waypoint mission.

    Publishes JSON telemetry on /rover/state and accepts operator
    commands (abort / return_to_previous) on /rover/command.
    """

    def __init__(self):
        super().__init__('rover_sim_node')

        self.state_pub = self.create_publisher(String, '/rover/state', 10)
        self.command_sub = self.create_subscription(
            String, '/rover/command', self.on_command, 10)

        # Mission state. current_index points into WAYPOINTS; distance_remaining
        # counts down to 0 as the rover "drives"; mode is what the GUI shows.
        self.current_index = 0
        self.previous_index = 0
        self.distance_remaining = WAYPOINTS[self.current_index]['distance_m']
        self.mode = 'autonomous'
        self.led_state = 'blue'  # blue = autonomous drive, green = arrived/idle, red = aborted

        # Publish telemetry at 2 Hz.
        self.create_timer(0.5, self.tick)

        self.get_logger().info('rover_sim_node started, beginning mission')

    def on_command(self, msg: String):
        try:
            command = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'Ignoring malformed command: {msg.data!r}')
            return

        action = command.get('command')

        if action == 'abort':
            self.mode = 'teleop'
            self.led_state = 'red'
            self.get_logger().info('Mission aborted by operator, switching to teleop')

        elif action == 'return_to_previous':
            self.current_index = self.previous_index
            self.distance_remaining = WAYPOINTS[self.current_index]['distance_m']
            self.mode = 'autonomous'
            self.led_state = 'blue'
            self.get_logger().info(
                f'Returning to previous target: {WAYPOINTS[self.current_index]["name"]}')

        elif action == 'ping':
            self.get_logger().info('Ping received, sending immediate state update')
            self.publish_state()

        else:
            self.get_logger().warn(f'Unknown command: {action!r}')

    def tick(self):
        # Only "drive" while in autonomous mode and there's still a mission to run.
        if self.mode == 'autonomous':
            self.distance_remaining -= DRIVE_RATE_M
            if self.distance_remaining <= 0.0:
                self.arrive_at_waypoint()

        self.publish_state()

    def arrive_at_waypoint(self):
        self.get_logger().info(f'Arrived at {WAYPOINTS[self.current_index]["name"]}')
        self.previous_index = self.current_index
        self.led_state = 'green'

        if self.current_index + 1 < len(WAYPOINTS):
            self.current_index += 1
            self.distance_remaining = WAYPOINTS[self.current_index]['distance_m']
            self.led_state = 'blue'
        else:
            # Mission complete -- loop back to the start so the sim keeps
            # producing live telemetry for the GUI instead of sitting idle
            # at 0 forever.
            self.current_index = 0
            self.distance_remaining = WAYPOINTS[0]['distance_m']
            self.mode = 'autonomous'
            self.led_state = 'blue'

    def publish_state(self):
        target = WAYPOINTS[self.current_index]
        state = {
            'mode': self.mode,
            'target_name': target['name'],
            'target_type': target['type'],
            'distance_remaining_m': round(max(self.distance_remaining, 0.0), 1),
            'gnss_lat': target['lat'],
            'gnss_lon': target['lon'],
            'led_state': self.led_state,
            'current_index': self.current_index,
            'start': START,
            'waypoints': [
                {'name': wp['name'], 'distance_m': wp['distance_m'],
                 'lat': wp['lat'], 'lon': wp['lon']}
                for wp in WAYPOINTS
            ],
        }
        msg = String()
        msg.data = json.dumps(state)
        self.state_pub.publish(msg)


def main():
    rclpy.init()
    node = RoverSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
