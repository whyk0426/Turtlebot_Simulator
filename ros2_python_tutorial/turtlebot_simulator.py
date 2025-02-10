import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
import math 


class Simulator(Node):
    def __init__(self):
        super().__init__('turtlebot_simulator')
        self.publisher_ = self.create_publisher(MarkerArray, 'robot/pose', 10)
        self.subscription_ = self.create_subscription(
            Twist,
            'robot/cmd',
            self.topic_callback,
            10
        )
        self.init_state()
        self.timer_ = self.create_timer(0.01, self.timer_callback)
        self.turtlename = "robot"

        self.tf_broadcaster = TransformBroadcaster(self)


    def init_state(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0


    def timer_callback(self):
        self.update_state()
        self.publish_marker_pose()
        self.broadcast_tf(self)
    

    def topic_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z


    def update_state(self):
        dt = 0.01
        self.x = self.x + math.cos(self.theta) * self.v * dt
        self.y = self.y + math.sin(self.theta) * self.v * dt
        self.theta = self.theta + self.w * dt
        

    def publish_marker_pose(self):
        marker_array = MarkerArray()

        # Set marker parameters
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'pose'
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = "package://ros2_python_tutorial/mesh/quadrotor_3.dae"
        marker.action = Marker.ADD
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(self.theta * 0.5)
        marker.pose.orientation.w = math.cos(self.theta * 0.5)
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0  
        marker.color.g = 1.0

        marker_array.markers.append(marker)
        self.publisher_.publish(marker_array)

    
    def broadcast_tf(self, msg):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.turtlename

        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        q = quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    turtlebot_simulator = Simulator()
    rclpy.spin(turtlebot_simulator)
    turtlebot_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
