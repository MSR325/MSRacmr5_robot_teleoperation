#!/usr/bin/env python3
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class SnakeTeleop(Node):
    def __init__(self):
        super().__init__("snake_teleop")
        self.num_modules = 8
        # Publishers for command topics
        self.v_pubs = [
            self.create_publisher(Float64, f"/snake_robot/v_joint_{i}/command", 10)
            for i in range(self.num_modules)
        ]
        self.h_pubs = [
            self.create_publisher(Float64, f"/snake_robot/h_joint_{i}/command", 10)
            for i in range(self.num_modules)
        ]
        # State storage
        self.v_states = [0.0] * self.num_modules
        self.h_states = [0.0] * self.num_modules
        # Command amplitudes
        self.v_amp = 0.0
        self.h_amp = 0.0

        # Subscribers for state feedback (optional for logging)
        for i in range(self.num_modules):
            self.create_subscription(
                Float64,
                f"/snake_robot/v_joint_{i}/state",
                self._state_callback(i, "v"),
                10,
            )
            self.create_subscription(
                Float64,
                f"/snake_robot/h_joint_{i}/state",
                self._state_callback(i, "h"),
                10,
            )

        self.get_logger().info(
            "Teleop snake robot started (i/k=vertical, j/l=horizontal, x=reset, ESC=exit)"
        )

        # Configure terminal for non-blocking input
        self.old_term = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # Timer for polling keyboard at 10 Hz
        self.create_timer(0.1, self._timer_callback)

    def _state_callback(self, index, joint_type):
        def callback(msg):
            if joint_type == "v":
                self.v_states[index] = msg.data
            else:
                self.h_states[index] = msg.data

        return callback

    def _timer_callback(self):
        # Check for keypress
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key == "i":
                self.v_amp = min(1.0, self.v_amp + 0.1)
            elif key == "k":
                self.v_amp = max(-1.0, self.v_amp - 0.1)
            elif key == "j":
                self.h_amp = max(-1.0, self.h_amp - 0.1)
            elif key == "l":
                self.h_amp = min(1.0, self.h_amp + 0.1)
            elif key == "x":
                self.v_amp = 0.0
                self.h_amp = 0.0
            elif key == "\x1b":  # ESC
                rclpy.shutdown()
                return

            # Publish commands
            v_msg = Float64(data=self.v_amp)
            h_msg = Float64(data=self.h_amp)
            for pub in self.v_pubs:
                pub.publish(v_msg)
            for pub in self.h_pubs:
                pub.publish(h_msg)

            self.get_logger().info(
                f"Commands → Vertical: {self.v_amp:.2f}, Horizontal: {self.h_amp:.2f}"
            )

    def destroy_node(self):
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_term)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SnakeTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
