import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from tf2_ros import TransformListener, Buffer, TransformException
from math import pi, hypot
import math


class TurtlebotController(Node):

    def __init__(self):
        super().__init__('turtlebot_controller')
        self.publisher_ = self.create_publisher(Twist, 'robot/cmd', 10)
        self.timer_cmd = self.create_timer(0.01, self.timer_cmd_callback)

        self.target_frame = self.declare_parameter('target_frame', 'robot').value
        self.source_frame = self.declare_parameter('source_frame', 'world').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer_tf = self.create_timer(0.01, self.timer_tf_callback)
       
        self.declaration()


    def declaration(self):
        self.l = [2, 0.001, 0]
        self.a = [7, 0.01, 0]
        self.v = 0
        self.goal_x = 0
        self.goal_y = 0
        self.goal_theta = 0
        self.real_x = 0
        self.real_y = 0
        self.real_theta = 0


    def timer_tf_callback(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                rclpy.time.Time()
            )

            self.real_x = t.transform.translation.x
            self.real_y = t.transform.translation.y
            q = t.transform.rotation

            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.real_theta = math.atan2(siny_cosp, cosy_cosp)

        except TransformException as ex:
            self.get_logger().info(f'Could not transform {self.target_frame} to {self.source_frame}: {ex}')


    def timer_cmd_callback(self):
        cmd = Twist()
        zero = 0
        prev_error_distance = 2
        prev_error_theta = 0
        dt = 0.01

        if (self.v == 0):
            self.goal_x = zero + 2
            self.goal_y = zero
            self.goal_theta = 0
        elif (self.v == 1):
            self.goal_x = zero + 2
            self.goal_y = zero
            self.goal_theta = pi/2
        elif (self.v == 2):
            self.goal_x = zero + 2
            self.goal_y = zero + 2
            self.goal_theta = pi/2
        elif (self.v == 3):
            self.goal_x = zero + 2
            self.goal_y = zero + 2
            self.goal_theta = pi
        elif (self.v == 4):
            self.goal_x = zero
            self.goal_y = zero + 2
            self.goal_theta = pi
        elif (self.v == 5):
            self.goal_x = zero 
            self.goal_y = zero + 2
            if (self.real_theta > 0):
                self.goal_theta = 3 * pi/2
            else:
                self.goal_theta = - pi/2
        elif (self.v == 6):
            self.goal_x = zero 
            self.goal_y = zero
            self.goal_theta = - pi/2
        elif (self.v == 7):
            self.goal_x = zero 
            self.goal_y = zero
            self.goal_theta = 0
        
        error_distance = hypot((self.goal_x - self.real_x), (self.goal_y - self.real_y)) 
        error_theta = self.goal_theta - self.real_theta
        i_error_distance = prev_error_distance + error_distance * dt
        i_error_theta = prev_error_theta + error_theta * dt
       
        p_control_linear = self.l[0] * error_distance
        p_control_angular = self.a[0] * error_theta

        d_control_linear = self.l[1] * (prev_error_distance - error_distance) / dt
        d_control_angular = self.a[1] * (prev_error_theta - error_theta) / dt

        i_control_linear = self.l[2] * i_error_distance * dt
        i_control_angular = self.a[2] * i_error_theta * dt

        if (self.v % 2 == 0):
            cmd.linear.x = p_control_linear + d_control_linear + i_control_linear
            cmd.angular.z = 0.0

        elif (self.v % 2 == 1):
            cmd.linear.x = 0.0
            cmd.angular.z = p_control_angular + d_control_angular + i_control_angular

        self.publisher_.publish(cmd)

        if ((abs(error_theta) < 0.001) and (abs(error_distance) < 0.01)):
            self.v = (self.v + 1) % 8
            i_error_distance = 0
            i_error_theta = 0

        prev_error_distance = error_distance
        prev_error_theta = error_theta


def main(args=None):
    rclpy.init(args=args)
    turtlebot_controller = TurtlebotController()
    rclpy.spin(turtlebot_controller)
    turtlebot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()