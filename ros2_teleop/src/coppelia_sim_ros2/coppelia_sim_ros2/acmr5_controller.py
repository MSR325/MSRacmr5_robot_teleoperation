#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import select
import termios
import tty


class SnakeTeleop(Node):
   def __init__(self):
       super().__init__('acmr5_controller')


       self.num_modules = 8
       self.v_joint_pubs = []
       self.h_joint_pubs = []
       self.v_joint_states = [0.0] * self.num_modules
       self.h_joint_states = [0.0] * self.num_modules


       for i in range(self.num_modules):
           v_topic = f'/snake_robot/v_joint_{i}/command'
           h_topic = f'/snake_robot/h_joint_{i}/command'
           self.v_joint_pubs.append(self.create_publisher(Float64, v_topic, 10))
           self.h_joint_pubs.append(self.create_publisher(Float64, h_topic, 10))


           # Subscribers para los estados
           v_state_topic = f'/snake_robot/v_joint_{i}/state'
           h_state_topic = f'/snake_robot/h_joint_{i}/state'
           self.create_subscription(Float64, v_state_topic, self.create_state_callback(i, 'v'), 10)
           self.create_subscription(Float64, h_state_topic, self.create_state_callback(i, 'h'), 10)


       self.v_amp = 0.0
       self.h_amp = 0.0


       self.print_instructions()


   def create_state_callback(self, index, joint_type):
       def callback(msg):
           if joint_type == 'v':
               self.v_joint_states[index] = msg.data
           else:
               self.h_joint_states[index] = msg.data
       return callback


   def print_instructions(self):
       print("""
============================
TELEOP ROBOT SERPIENTE ROS 2
============================
Controles:
 i/k : Incrementar/Decrementar amplitud vertical
 j/l : Incrementar/Decrementar amplitud horizontal
 x   : Resetear a 0.0
 ESC : Salir
============================
""")


   def get_key(self):
       if select.select([sys.stdin], [], [], 0.1) == ([sys.stdin], [], []):
           return sys.stdin.read(1)
       return None


   def publish_commands(self):
       v_msg = Float64()
       h_msg = Float64()
       v_msg.data = self.v_amp
       h_msg.data = self.h_amp


       for pub in self.v_joint_pubs:
           pub.publish(v_msg)
       for pub in self.h_joint_pubs:
           pub.publish(h_msg)


   def run(self):
       old_settings = termios.tcgetattr(sys.stdin)
       try:
           tty.setcbreak(sys.stdin.fileno())


           while rclpy.ok():
               key = self.get_key()


               if key:
                   if key == 'i':
                       self.v_amp = min(1.0, self.v_amp + 0.1)
                   elif key == 'k':
                       self.v_amp = max(-1.0, self.v_amp - 0.1)
                   elif key == 'j':
                       self.h_amp = max(-1.0, self.h_amp - 0.1)
                   elif key == 'l':
                       self.h_amp = min(1.0, self.h_amp + 0.1)
                   elif key == 'x':
                       self.v_amp = 0.0
                       self.h_amp = 0.0
                   elif key == '\x1b':  # ESC
                       break


                   print(f"Vertical: {self.v_amp:.2f} | Horizontal: {self.h_amp:.2f}")


                   self.publish_commands()


               rclpy.spin_once(self, timeout_sec=0.01)
               print("Joint states:")
               for i in range(self.num_modules):
                   print(f"  v_joint_{i}: {self.v_joint_states[i]:.2f}, h_joint_{i}: {self.h_joint_states[i]:.2f}")
               print()




       finally:
           termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
           # Parar todo al salir
           stop_msg = Float64()
           stop_msg.data = 0.0
           for pub in self.v_joint_pubs + self.h_joint_pubs:
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


if __name__ == '__main__':
   main()