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
        self.v_pubs = []
        self.h_pubs = []
        self.v_states = [0.0] * self.num_modules
        self.h_states = [0.0] * self.num_modules
        self.v_amp = 0.0
        self.h_amp = 0.0

        qos = 10
        for i in range(self.num_modules):
            # Publicadores de comandos
            v_topic = f"/snake_robot/v_joint_{i}/command"
            h_topic = f"/snake_robot/h_joint_{i}/command"
            self.v_pubs.append(self.create_publisher(Float64, v_topic, qos))
            self.h_pubs.append(self.create_publisher(Float64, h_topic, qos))
            # Suscriptores de estado
            self.create_subscription(
                Float64,
                f"/snake_robot/v_joint_{i}/state",
                self._state_callback(i, "v"),
                qos,
            )
            self.create_subscription(
                Float64,
                f"/snake_robot/h_joint_{i}/state",
                self._state_callback(i, "h"),
                qos,
            )

        self.get_logger().info(
            "Teleop snake robot iniciado: i/k vertical, j/l horizontal, x reset, ESC salir"
        )
        self.rate = self.create_rate(10)  # 10 Hz

    def _state_callback(self, index, joint_type):
        def callback(msg):
            if joint_type == "v":
                self.v_states[index] = msg.data
            else:
                self.h_states[index] = msg.data

        return callback

    def _get_key(self, timeout=0.1):
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return None

    def _publish(self):
        v_msg = Float64()
        h_msg = Float64()
        v_msg.data = self.v_amp
        h_msg.data = self.h_amp
        for pub in self.v_pubs:
            pub.publish(v_msg)
        for pub in self.h_pubs:
            pub.publish(h_msg)

    def run(self):
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok():
                key = self._get_key()
                if key == "i":
                    self.v_amp = min(1.0, self.v_amp + 0.1)
                elif key == "k":
                    self.v_amp = max(-1.0, self.v_amp - 0.1)
                elif key == "l":
                    self.h_amp = min(1.0, self.h_amp + 0.1)
                elif key == "j":
                    self.h_amp = max(-1.0, self.h_amp - 0.1)
                elif key == "x":
                    self.v_amp = 0.0
                    self.h_amp = 0.0
                elif key == "\x1b":  # ESC
                    break

                # Publica comandos siempre que cambien
                self._publish()
                self.get_logger().info(
                    f"Comandos → Vertical: {self.v_amp:.2f}, Horizontal: {self.h_amp:.2f}"
                )

                rclpy.spin_once(self, timeout_sec=0.01)
                self.rate.sleep()

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            # Detener movimientos al salir
            stop_msg = Float64()
            stop_msg.data = 0.0
            for pub in self.v_pubs + self.h_pubs:
                pub.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    teleop = SnakeTeleop()
    try:
        teleop.run()
    except KeyboardInterrupt:
        pass
    finally:
        teleop.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
