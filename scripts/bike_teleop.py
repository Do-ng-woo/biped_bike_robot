#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import select
import termios
import tty

msg = """
Control Your Biped Bike!
---------------------------
Moving around:
        w
   a    s    d

w/s : increase/decrease forward speed
a/d : turn left/right (Differential Drive)
q   : force stop
space: force stop

CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0),
    's': (-1, 0),
    'a': (0, 1),
    'd': (0, -1),
    'q': (0, 0),
    ' ': (0, 0),
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class BikeTeleopNode(Node):
    def __init__(self):
        super().__init__('bike_teleop_node')
        self.declare_parameter('speed_step', 0.5)
        self.declare_parameter('turn_step', 0.5)
        self.declare_parameter('max_wheel_speed', 2.0)
        # JointGroupVelocityController commands
        self.publisher_ = self.create_publisher(Float64MultiArray, '/wheel_velocity_controller/commands', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        self.speed_step = float(self.get_parameter('speed_step').value)
        self.turn_step = float(self.get_parameter('turn_step').value)
        self.max_wheel_speed = float(self.get_parameter('max_wheel_speed').value)

        self.get_logger().info(msg)

    def update_velocities(self, key):
        if key in moveBindings.keys():
            lin, ang = moveBindings[key]
            if lin == 0 and ang == 0:
                self.linear_vel = 0.0
                self.angular_vel = 0.0
            else:
                self.linear_vel += lin * self.speed_step
                self.angular_vel += ang * self.turn_step
                self.linear_vel = max(
                    -self.max_wheel_speed,
                    min(self.max_wheel_speed, self.linear_vel),
                )
                self.angular_vel = max(
                    -self.max_wheel_speed,
                    min(self.max_wheel_speed, self.angular_vel),
                )
            
            self.get_logger().info(f'Speed: {self.linear_vel} | Turn: {self.angular_vel}')
        else:
            # If any other key is pressed (not in bindings), do nothing.
            # Stop linearly when released? Turtlebot teleop usually holds previous state.
            pass

    def timer_callback(self):
        cmd = Float64MultiArray()
        
        # Diff drive kinematics:
        # We need to map [linear, angular] to [l_knee_wheel, r_knee_wheel, arm_wheel]
        
        # Base kinematics (assumes same physical orientation)
        raw_l = self.linear_vel - self.angular_vel
        raw_r = self.linear_vel + self.angular_vel
        raw_f = self.linear_vel
        
        # --- URDF Axis Correction (Final Flip) ---
        # 팀장님 피드백 반영: 전진/후진 및 좌/우 사출 방향을 완전히 반전시켰습니다.
        # L_axis: 0 0 -1 | R_axis: 0 0 1 | Arm_axis: 0 0 -1
        l_speed = raw_l * -1.0
        r_speed = raw_r * 1.0   
        f_speed = raw_f * -1.0
        
        cmd.data = [l_speed, r_speed, f_speed]
        self.publisher_.publish(cmd)

def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = BikeTeleopNode()
    
    try:
        while rclpy.ok():
            key = getKey(settings)
            if key == '\x03': # CTRL-C
                break
            
            node.update_velocities(key)
            rclpy.spin_once(node, timeout_sec=0.0)
            
    except Exception as e:
        print(e)
    finally:
        # Stop wheels before exiting
        cmd = Float64MultiArray()
        cmd.data = [0.0, 0.0, 0.0]
        node.publisher_.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.1)
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
