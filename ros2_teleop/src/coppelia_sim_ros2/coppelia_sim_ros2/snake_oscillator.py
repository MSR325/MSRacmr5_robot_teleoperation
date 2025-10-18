#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64


class SnakeOscillator(Node):
    def __init__(self):
        super().__init__("snake_oscillator")

        # Parámetros
        self.num_modules = 8
        self.phase_shift = 2 * math.pi / self.num_modules
        self.t = 0.0
        self.timer_period = 0.02  # 50 Hz

        # Últimos comandos de teleop_twist_keyboard
        self.amplitude = 0.0  # vendrá de linear.x
        self.frequency = 0.0  # vendrá de angular.z

        # Suscripción a cmd_vel
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        # Publicadores para cada joint
        self.v_pubs = [
            self.create_publisher(Float64, f"/snake_robot/v_joint_{i}/command", 10)
            for i in range(self.num_modules)
        ]
        self.h_pubs = [
            self.create_publisher(Float64, f"/snake_robot/h_joint_{i}/command", 10)
            for i in range(self.num_modules)
        ]

        # Timer para generar la onda
        self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(
            "Nodo snake_oscillator inicializado: suscrito a /cmd_vel, publicando joints oscilatorios"
        )

    def cmd_vel_callback(self, msg: Twist):
        # Usamos linear.x como amplitud y angular.z como frecuencia
        self.amplitude = msg.linear.x
        self.frequency = msg.angular.z

    def timer_callback(self):
        self.t += self.timer_period
        A = self.amplitude
        ω = 2 * math.pi * self.frequency

        for i in range(self.num_modules):
            # Ángulo de joint i
            angle = A * math.sin(ω * self.t + i * self.phase_shift)

            # Publica a joints:
            #   -- Verticales en cero (puedes cambiar esto si prefieres)
            self.v_pubs[i].publish(Float64(data=0.0))
            #   -- Horizontales con el ángulo oscilatorio
            self.h_pubs[i].publish(Float64(data=angle))

        # Opción: loggear cada X segundos
        # self.get_logger().debug(f'Amplitude={A:.2f}, Frequency={self.frequency:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = SnakeOscillator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
