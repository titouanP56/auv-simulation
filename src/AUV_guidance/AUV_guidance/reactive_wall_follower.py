#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import numpy as np
import math
from rclpy.time import Time

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs # x
    q[1] = cj*ss + sj*cc # y
    q[2] = cj*cs - sj*sc # z
    q[3] = cj*cc + sj*ss # w
    return q

class ReactiveWallFollower(Node):
    def __init__(self):
        super().__init__('reactive_wall_follower')

        # Parameters
        self.declare_parameter('enabled', True)
        self.declare_parameter('target_distance', 1.5)
        self.declare_parameter('nominal_velocity', 0.3)
        self.declare_parameter('gain_correction', 0.8)
        self.declare_parameter('circle_center_x', 0.0)
        self.declare_parameter('circle_center_y', 0.0)
        self.declare_parameter('circle_radius', 25.0)
        self.declare_parameter('lookahead_time', 2.0)

        # Initialize parameter variables
        self.update_parameters()

        # State variables
        self.phase2_done = False
        self.origin_recorded = False
        self.origin_pose = None # [x_g, y_g, z_g, yaw_g]
        self.last_net_time = None
        self.net_pose = None
        self.current_pose = None
        self.z_target = 0.0 # Keep fixed depth for 2D following
        self.start_transition_time = None

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.net_frame_sub = self.create_subscription(PoseStamped, '/perception/net_local_frame', self.net_frame_callback, 10)
        self.phase2_sub = self.create_subscription(Bool, '/mission/phase2_done', self.phase2_done_callback, 10)
        self.origin_sub = self.create_subscription(PoseStamped, '/mission/local_origin', self.origin_callback, 10)

        # Publisher to MPC
        self.target_pub = self.create_publisher(PoseStamped, '/cmd_setpoint', 10)

        # Timer for control loop at 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f"Reactive Wall Follower initialized (Enabled: {self.enabled}).")

    def update_parameters(self):
        self.enabled = self.get_parameter('enabled').value
        self.target_distance = self.get_parameter('target_distance').value
        self.nominal_velocity = self.get_parameter('nominal_velocity').value
        self.gain_correction = self.get_parameter('gain_correction').value
        self.circle_center_x = self.get_parameter('circle_center_x').value
        self.circle_center_y = self.get_parameter('circle_center_y').value
        self.circle_radius = self.get_parameter('circle_radius').value
        self.lookahead_time = self.get_parameter('lookahead_time').value

    def origin_callback(self, msg):
        if not self.origin_recorded:
            p = msg.pose.position
            # On récupère l'orientation sous le nom "yaw_origin"
            _, _, yaw_origin = euler_from_quaternion(
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            )
            self.origin_pose = [p.x, p.y, p.z, yaw_origin]
            self.origin_recorded = True
            
            # --- CALCUL DYNAMIQUE DU CENTRE DE LA CAGE ---
            # Dans net_approach, yaw_origin pointe vers l'extérieur du filet.
            # Pour pointer vers le centre de la cage, on fait -pi.
            yaw_vers_centre = yaw_origin - math.pi
            
            print("yaw_vers_centre", yaw_vers_centre)
            # La distance de l'origine (le filet) au centre de la cage est JUSTE le rayon (25m)
            distance_au_centre = self.circle_radius
            
            # On écrase les paramètres par la vraie position de la cage
            self.circle_center_x = p.x - distance_au_centre * math.cos(yaw_vers_centre)
            self.circle_center_y = p.y - distance_au_centre * math.sin(yaw_vers_centre)
            
            self.get_logger().info(f"Local Origin recorded: {self.origin_pose}")
            self.get_logger().info(f"Calculated Cage Center (Odom): X={self.circle_center_x:.2f}, Y={self.circle_center_y:.2f}")

    def phase2_done_callback(self, msg):
        if msg.data and not self.phase2_done:
            self.phase2_done = True
            self.start_transition_time = self.get_clock().now()
            if self.enabled:
                self.get_logger().info("Phase 2 done signal received. Activating Reactive Wall Follower.")

    def net_frame_callback(self, msg):
        self.net_pose = msg.pose
        self.last_net_time = self.get_clock().now()

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def control_loop(self):
        if not self.enabled or not self.phase2_done or self.current_pose is None or not self.origin_recorded:
            return

        self.update_parameters()

        # Robot position in Odom
        x_g = self.current_pose.position.x
        y_g = self.current_pose.position.y
        
        # Transformation to Local Frame (matching MPC)
        dx = x_g - self.origin_pose[0]
        dy = y_g - self.origin_pose[1]
        psi_0 = self.origin_pose[3]

        x_L = dx * math.cos(psi_0) + dy * math.sin(psi_0)
        y_L = -dx * math.sin(psi_0) + dy * math.cos(psi_0)

        # Theoretical Vector Field (computed in Odom for simplicity relative to center)
        om_x = x_g - self.circle_center_x
        om_y = y_g - self.circle_center_y
        dist_to_center = math.hypot(om_x, om_y)

        if dist_to_center < 0.1: dist_to_center = 0.1

        u_om_x = om_x / dist_to_center
        u_om_y = om_y / dist_to_center

        # Tangent vector CCW in odom
        v_t_g_x = -u_om_y
        v_t_g_y = u_om_x

        v_n_g_x = 0.0
        v_n_g_y = 0.0
        target_yaw_g = 0.0

        # Perception logic
        current_time = self.get_clock().now()
        use_perception = False
        if self.last_net_time is not None:
            time_diff = (current_time - self.last_net_time).nanoseconds / 1e9
            if time_diff < 1.0 and self.net_pose is not None:
                use_perception = True

        if use_perception:
            p_cx = self.net_pose.position.x
            p_cy = self.net_pose.position.y
            _, _, net_yaw_g = euler_from_quaternion(
                self.net_pose.orientation.x, self.net_pose.orientation.y,
                self.net_pose.orientation.z, self.net_pose.orientation.w
            )

            # NOTE: L'orientation calculée par pca pointe déjà vers le filet.
            # L'opération modulo que tu as ajoutée ne modifiait pas la valeur finale
            # (car x + pi % 2pi - pi redonne x). On la retire pour plus de clarté.
            normal_g_x = math.cos(net_yaw_g)
            normal_g_y = math.sin(net_yaw_g)

            dx_net = p_cx - x_g
            dy_net = p_cy - y_g
            d_mesuree = math.hypot(dx_net, dy_net)
            error = d_mesuree - self.target_distance

            v_n_g_x = error * normal_g_x
            v_n_g_y = error * normal_g_y
            target_yaw_g = net_yaw_g
        else:
            # Inspection depuis l'intérieur : le robot suit un cercle plus petit
            tracking_radius = self.circle_radius - self.target_distance
            error = dist_to_center - tracking_radius
            
            # Si dist < tracking_radius (trop près du centre / trop loin du filet), error < 0.
            # On veut pousser vers l'extérieur (dans le sens de u_om). 
            # Donc v_n = -error * u_om = positive * u_om (vers l'extérieur)
            v_n_g_x = -error * u_om_x
            v_n_g_y = -error * u_om_y

            # Pointer vers le filet (radialement vers l'EXTÉRIEUR du centre)
            target_yaw_g = math.atan2(u_om_y, u_om_x)

        # Resultant vector in Odom
        v_res_g_x = v_t_g_x + self.gain_correction * v_n_g_x
        v_res_g_y = v_t_g_y + self.gain_correction * v_n_g_y

        # Normalize and apply velocity
        norm_res = math.hypot(v_res_g_x, v_res_g_y)
        if norm_res > 0.001:
            v_res_g_x = (v_res_g_x / norm_res) * self.nominal_velocity
            v_res_g_y = (v_res_g_y / norm_res) * self.nominal_velocity
        
        # Look ahead Odom Target
        target_g_x = x_g + v_res_g_x * self.lookahead_time
        target_g_y = y_g + v_res_g_y * self.lookahead_time
        target_yaw_L = target_yaw_g - psi_0

        # Transform Target to Local Frame
        tdx = target_g_x - self.origin_pose[0]
        tdy = target_g_y - self.origin_pose[1]
        target_x_L = tdx * math.cos(psi_0) + tdy * math.sin(psi_0)
        target_y_L = -tdx * math.sin(psi_0) + tdy * math.cos(psi_0)

        # Soft start / Initial setpoint at 0 0 0
        elapsed_since_start = (current_time - self.start_transition_time).nanoseconds / 1e9
        if elapsed_since_start < 1.0:
            # Gradually move from 0,0,0 to calculated target over 1 second
            target_x_L = target_x_L * elapsed_since_start
            target_y_L = target_y_L * elapsed_since_start
            # You could also interpolate yaw but 0 to target is usually fine

        # Publish target pose in LOCAL frame
        target_msg = PoseStamped()
        target_msg.header.stamp = current_time.to_msg()
        target_msg.header.frame_id = 'local_origin' # Informative
        
        target_msg.pose.position.x = target_x_L
        target_msg.pose.position.y = target_y_L
        target_msg.pose.position.z = self.z_target

        q = quaternion_from_euler(0, 0, target_yaw_L)
        target_msg.pose.orientation.x = q[0]
        target_msg.pose.orientation.y = q[1]
        target_msg.pose.orientation.z = q[2]
        target_msg.pose.orientation.w = q[3]

        self.target_pub.publish(target_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveWallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
