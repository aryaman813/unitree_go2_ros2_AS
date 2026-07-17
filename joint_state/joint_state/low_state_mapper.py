#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from unitree_go.msg import LowState, SportModeState

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class LowStateMapper(Node):
    def __init__(self):
        super().__init__('low_state_mapper')
        
        self.tf_broadcaster = TransformBroadcaster(self)

        self.joint_names = [
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
        ]

        # Live tracking variables for base translation
        self.base_x = 0.0
        self.base_y = 0.0
        self.base_z = 0.6 

        # Live tracking variables for base orientation (Unitree order: W, X, Y, Z)
        self.base_qx = 0.0
        self.base_qy = 0.0
        self.base_qz = 0.0
        self.base_qw = 1.0

        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Correctly assigned subscriber to real-world live odometry positions
        self.odom_sub = self.create_subscription(
            SportModeState, '/lf/odommodestate', self.odom_callback, 10
        )

        self.low_state_sub = self.create_subscription(
            LowState, '/lf/lowstate', self.low_state_callback, 10
        )

        self.get_logger().info("Unified Joint and Odometry TF Mapper Active with /lf/odommodestate.")

    def odom_callback(self, msg):
        """Asynchronously updates the robot tracking coordinate and orientation cache."""
        # 1. Update position array variables
        self.base_x = float(msg.position[0])
        self.base_y = float(msg.position[1])
        self.base_z = float(msg.position[2])

        # 2. Safely capture orientation matrix directly from working SportModeState
        # Unitree structure arrays use [0]=W, [1]=X, [2]=Y, [3]=Z layout matches your echo
        self.base_qw = float(msg.imu_state.quaternion[0])
        self.base_qx = float(msg.imu_state.quaternion[1])
        self.base_qy = float(msg.imu_state.quaternion[2])
        self.base_qz = float(msg.imu_state.quaternion[3])

    def low_state_callback(self, msg):
        current_time = self.get_clock().now().to_msg()

        # 1. Map joint angles to animate leg links
        joint_state = JointState()
        joint_state.header.stamp = current_time
        joint_state.name = self.joint_names

        # Safeguard to prevent out-of-bounds mapping errors
        num_motors = min(len(msg.motor_state), 12)
        for i in range(num_motors):
            joint_state.position.append(msg.motor_state[i].q)
            joint_state.velocity.append(msg.motor_state[i].dq)
            joint_state.effort.append(msg.motor_state[i].tau_est)

        # Fallback if motor_state array length isn't complete yet
        while len(joint_state.position) < 12:
            joint_state.position.append(0.0)
            joint_state.velocity.append(0.0)
            joint_state.effort.append(0.0)

        self.joint_state_pub.publish(joint_state)

        # 2. Extract base link transform relative to running frame
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Inject real-time coordinates pulled from subscriber updates
        t.transform.translation.x = self.base_x
        t.transform.translation.y = self.base_y
        t.transform.translation.z = self.base_z

        # Extract stable, clean orientation tracking coordinates 
        t.transform.rotation.x = self.base_qx
        t.transform.rotation.y = self.base_qy
        t.transform.rotation.z = self.base_qz
        t.transform.rotation.w = self.base_qw

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = LowStateMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
