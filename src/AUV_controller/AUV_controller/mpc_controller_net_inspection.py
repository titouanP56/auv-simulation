import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import numpy as np
import do_mpc
import casadi as ca
import math
import time

# Constants for thruster coefficients and water density
THRUST_COEFFS = [-0.002, 0.002, 0.002, -0.002, -0.002, 0.002, 0.002, -0.002]
RHO = 997.0
BUOYANCY_NET = 3.0 

class MPCControllerNetInspection(Node):
    def __init__(self):
        super().__init__('mpc_controller_net_inspection')

        # State
        self.phase2_done = False
        
        # Publishers for the 8 thrusters commands
        self.pubs = []
        for i in range(1, 9):
            self.pubs.append( 
                self.create_publisher(Float64, f'/cmd_vel_{i}', 10)
            )

        self.error_pub = self.create_publisher(Float64MultiArray, '/mpc_tracking_error', 10)

        # Subscribers
        self.sub_phase2 = self.create_subscription(
            Bool, '/mission/phase2_done', self.phase2_done_callback, 10
        )
        self.sub_origin = self.create_subscription(
            PoseStamped, '/mission/local_origin', self.origin_callback, 10
        )
        self.sub_odom = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10
        )
        self.sub_setpoint = self.create_subscription(
            PoseStamped, '/cmd_setpoint', self.setpoint_callback, 10
        )

        # Local Origin recording
        self.origin_recorded = False
        self.origin_pose = None # [x_g, y_g, z_g, yaw_g]
        
        # Current state and target (in LOCAL frame)
        self.current_target = np.array([0.0, 0.0, 0.0, 0.0]) # [x_L, y_L, z_L, yaw_L]
        self.current_state = np.zeros(8) # [x_L, y_L, z_L, psi_L, u, v, w, r]
        self.odom_received = False

        # Initialize the do_mpc mathematical model
        self.setup_mpc()

        # Main control loop running at 10Hz (0.1s period)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Net Inspection MPC Controller Initialized")

    def origin_callback(self, msg):
        """Receives the origin defined by Phase 2."""
        if not self.origin_recorded:
            p_g = msg.pose.position
            o_g = msg.pose.orientation
            
            # Global yaw
            siny_cosp = 2 * (o_g.w * o_g.z + o_g.x * o_g.y)
            cosy_cosp = 1 - 2 * (o_g.y * o_g.y + o_g.z * o_g.z)
            psi_g = np.arctan2(siny_cosp, cosy_cosp)
            
            self.origin_pose = np.array([p_g.x, p_g.y, p_g.z, psi_g])
            self.origin_recorded = True
            self.get_logger().info(f"Origin received from Phase 2: {self.origin_pose}")

    def phase2_done_callback(self, msg):
        if msg.data and not self.phase2_done:
            self.phase2_done = True
            self.get_logger().info("Phase 2 signal received! MPC controller now active.")

    def setpoint_callback(self, msg):
        """Receives setpoint in LOCAL frame."""
        # Extract yaw from quaternion
        o = msg.pose.orientation
        siny_cosp = 2 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1 - 2 * (o.y * o.y + o.z * o.z)
        yaw_L = np.arctan2(siny_cosp, cosy_cosp)
        
        self.current_target = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            yaw_L
        ])

    def odom_callback(self, msg):
        """Updates the current state in LOCAL frame."""
        if not self.phase2_done or not self.origin_recorded:
            return

        p_g = msg.pose.pose.position
        o_g = msg.pose.pose.orientation
        v_b = msg.twist.twist.linear
        w_b = msg.twist.twist.angular

        # Global yaw
        siny_cosp = 2 * (o_g.w * o_g.z + o_g.x * o_g.y)
        cosy_cosp = 1 - 2 * (o_g.y * o_g.y + o_g.z * o_g.z)
        psi_g = np.arctan2(siny_cosp, cosy_cosp)

        if not self.origin_recorded:
            return

        # Transformation to Local Frame
        dx = p_g.x - self.origin_pose[0]
        dy = p_g.y - self.origin_pose[1]
        psi_0 = self.origin_pose[3]

        x_L = dx * math.cos(psi_0) + dy * math.sin(psi_0)
        y_L = -dx * math.sin(psi_0) + dy * math.cos(psi_0)
        z_L = p_g.z - self.origin_pose[2]
        psi_L = psi_g - psi_0
        
        # Normalize psi_L
        psi_L = (psi_L + math.pi) % (2 * math.pi) - math.pi

        self.current_state = np.array([
            x_L, y_L, z_L, psi_L,
            v_b.x, v_b.y, v_b.z, w_b.z,
        ])
        self.odom_received = True

    def setup_mpc(self):
        """Identical model and TAM to mpc_controller_sensors.py"""
        model_type = 'continuous'
        self.model = do_mpc.model.Model(model_type)

        x   = self.model.set_variable(var_type='_x', var_name='x')
        y   = self.model.set_variable(var_type='_x', var_name='y')
        z   = self.model.set_variable(var_type='_x', var_name='z')
        psi = self.model.set_variable(var_type='_x', var_name='psi')
        u = self.model.set_variable(var_type='_x', var_name='u')   
        v = self.model.set_variable(var_type='_x', var_name='v')   
        w = self.model.set_variable(var_type='_x', var_name='w')   
        r = self.model.set_variable(var_type='_x', var_name='r')   

        t1 = self.model.set_variable(var_type='_u', var_name='t1')
        t2 = self.model.set_variable(var_type='_u', var_name='t2')
        t3 = self.model.set_variable(var_type='_u', var_name='t3')
        t4 = self.model.set_variable(var_type='_u', var_name='t4')
        t5 = self.model.set_variable(var_type='_u', var_name='t5')
        t6 = self.model.set_variable(var_type='_u', var_name='t6')
        t7 = self.model.set_variable(var_type='_u', var_name='t7')
        t8 = self.model.set_variable(var_type='_u', var_name='t8')

        # Parameters from original script
        mass_body = 12.9
        m_eff_u, m_eff_v, m_eff_w = mass_body + 6.36, mass_body + 7.12, mass_body + 12.0
        Izz_eff = 0.16 + 0.15
        Xu_lin, Xu_quad = 4.03, 18.18
        Yv_lin, Yv_quad = 6.22, 21.66
        Zw_lin, Zw_quad = 5.18, 39.99
        Nr_lin, Nr_quad = 0.07, 1.55

        # Defining symbolic setpoints (MUST BE BEFORE model.setup())
        x_ref = self.model.set_variable(var_type='_p', var_name='x_ref')
        y_ref = self.model.set_variable(var_type='_p', var_name='y_ref')
        z_ref = self.model.set_variable(var_type='_p', var_name='z_ref')
        psi_ref = self.model.set_variable(var_type='_p', var_name='psi_ref')

        # Exact TAM from original script
        sin45, lever, z_arm = 0.7071, 0.1697, 0.1
        TAM = np.array([
            [ sin45,        sin45,       -sin45,       -sin45,        0.0,    0.0,    0.0,    0.0 ],
            [ sin45,       -sin45,        sin45,       -sin45,        0.0,    0.0,    0.0,    0.0 ],
            [ 0.0,          0.0,          0.0,          0.0,         -1.0,    1.0,    1.0,   -1.0 ],
            [-z_arm*sin45,  z_arm*sin45, -z_arm*sin45,  z_arm*sin45,  0.218,  0.218,  0.218,  0.218 ],
            [ z_arm*sin45,  z_arm*sin45, -z_arm*sin45, -z_arm*sin45,  0.12,  -0.12,   0.12,  -0.12  ],
            [ lever,       -lever,       -lever,        lever,        0.0,    0.0,    0.0,    0.0 ]
        ])

        u_vec = ca.vertcat(t1, t2, t3, t4, t5, t6, t7, t8)
        tau = ca.mtimes(TAM, u_vec)

        self.model.set_rhs('x', u * ca.cos(psi) - v * ca.sin(psi))
        self.model.set_rhs('y', u * ca.sin(psi) + v * ca.cos(psi))
        self.model.set_rhs('z', w)
        self.model.set_rhs('psi', r)

        eps = 1e-4
        self.model.set_rhs('u', (tau[0] - Xu_lin*u - Xu_quad*u*ca.sqrt(u**2 + eps)) / m_eff_u)
        self.model.set_rhs('v', (tau[1] - Yv_lin*v - Yv_quad*v*ca.sqrt(v**2 + eps)) / m_eff_v)
        self.model.set_rhs('w', (tau[2] - Zw_lin*w - Zw_quad*w*ca.sqrt(w**2 + eps) + BUOYANCY_NET) / m_eff_w)
        self.model.set_rhs('r', (tau[5] - Nr_lin*r - Nr_quad*r*ca.sqrt(r**2 + eps)) / Izz_eff)

        self.model.setup()

        self.mpc = do_mpc.controller.MPC(self.model)
        self.mpc.set_param(n_horizon=4, t_step=0.3, n_robust=0, store_full_solution=False)
        self.mpc.set_param(nlpsol_opts={
            'ipopt.max_iter': 10,
            'ipopt.warm_start_init_point': 'yes',
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.tol': 1e-1,
            'ipopt.acceptable_tol': 5e-1,
        })

        # Cost function
        # Note: We will update the reference parameters dynamically or use set_objective in loop
        # For simplicity in do_mpc, we can use 'p' variables for setpoints if we want to change them often,
        # but here we'll just use the objective with the current_target values.
        
        # Accessing existing symbolic variables
        x_vars = self.model.x
        x, y, z, psi, r_val = x_vars['x'], x_vars['y'], x_vars['z'], x_vars['psi'], x_vars['r']
        
        u_vars = self.model.u
        t1, t2, t3, t4, t5, t6, t7, t8 = u_vars['t1'], u_vars['t2'], u_vars['t3'], u_vars['t4'], u_vars['t5'], u_vars['t6'], u_vars['t7'], u_vars['t8']
        u_vec_cost = ca.vertcat(t1, t2, t3, t4, t5, t6, t7, t8)

        # Accessing existing symbolic setpoints
        x_ref = self.model.p['x_ref']
        y_ref = self.model.p['y_ref']
        z_ref = self.model.p['z_ref']
        psi_ref = self.model.p['psi_ref']

        # --- COST FUNCTION (Objectif d'inspection stricte) ---
        
        # 1. ORIENTATION (Priorité absolue)
        # Poids gigantesque sur le maintien du cap. Le ROV préférera être légèrement 
        # en retard sur sa trajectoire plutôt que de détourner le regard du filet.
        yaw_err = 2000.0 * (1 - ca.cos(psi - psi_ref))
        
        # 2. POSITION (Suivi de trajectoire)
        # On augmente les poids de position pour qu'il suive bien le chemin.
        # Z (profondeur) a un poids un peu plus fort car la dynamique verticale 
        # combat la flottabilité.
        mterm = (
            100.0 * (x - x_ref)**2 +  # Distance au filet (X)
            100.0 * (y - y_ref)**2 +  # Décalage latéral (Y - très important pour le scan)
            150.0 * (z - z_ref)**2 +  # Profondeur (Z - descente/remontée)
            yaw_err                   # Orientation (Psi)
        )
        
        # 3. STABILITÉ DYNAMIQUE (lterm)
        # On pénalise très fortement 'r_val' (la vitesse de rotation) : on lui interdit de pivoter.
        # On garde une toute petite pénalité d'énergie (u_vec_cost) pour éviter que les moteurs 
        # ne saturent inutilement, mais assez faible pour qu'il ait la force de bouger.
        lterm = mterm + 400.0 * r_val**2 + 0.005 * (ca.sumsqr(u_vec_cost))
        
        self.mpc.set_objective(mterm=mterm, lterm=lterm)

        # 4. LISSAGE DES COMMANDES (rterm)
        # On pénalise la variation brutale des propulseurs entre deux pas de temps.
        # Une valeur de 0.1 est un bon compromis : assez souple pour éviter les à-coups 
        # (qui feraient tanguer le robot), mais assez réactif pour suivre les cibles dynamiques.
        self.mpc.set_rterm(
            t1=0.1, t2=0.1, t3=0.1, t4=0.1, 
            t5=0.1, t6=0.1, t7=0.1, t8=0.1
        )

        for name in ['t1', 't2', 't3', 't4', 't5', 't6', 't7', 't8']:
            self.mpc.bounds['lower', '_u', name] = -5.0
            self.mpc.bounds['upper', '_u', name] =  5.0

        # Constraints (same as original)
        F_surge_struct = sin45 * (t1 + t2 - t3 - t4)
        F_sway_struct  = sin45 * (t1 - t2 + t3 - t4)
        pitch_torque_balance = (t5 - t6 + t7 - t8) + (0.1 / 0.12) * F_surge_struct
        self.mpc.set_nl_cons('eq_pitch_max', pitch_torque_balance, ub=0.5)
        self.mpc.set_nl_cons('eq_pitch_min', -pitch_torque_balance, ub=0.5)
        roll_torque_balance = (t5 + t6 + t7 + t8) - (0.1 / 0.218) * F_sway_struct
        self.mpc.set_nl_cons('eq_roll_max', roll_torque_balance, ub=0.5)
        self.mpc.set_nl_cons('eq_roll_min', -roll_torque_balance, ub=0.5)

        # Set p_fun template BEFORE setup
        p_template = self.mpc.get_p_template(1)
        def p_fun(t_now):
            p_template['_p', 0, 'x_ref'] = self.current_target[0]
            p_template['_p', 0, 'y_ref'] = self.current_target[1]
            p_template['_p', 0, 'z_ref'] = self.current_target[2]
            p_template['_p', 0, 'psi_ref'] = self.current_target[3]
            return p_template
        
        self.mpc.set_p_fun(p_fun)

        self.mpc.setup()
        self.mpc.set_initial_guess()

    def control_loop(self):
        if not self.phase2_done or not self.odom_received:
            return

        # --- AJOUT POUR LES GRAPHIQUES FOXGLOVE ---
        err_x = self.current_target[0] - self.current_state[0]
        err_y = self.current_target[1] - self.current_state[1]
        err_z = self.current_target[2] - self.current_state[2]
        
        # Pour le yaw, on gère le passage par pi/-pi pour éviter les sauts
        err_yaw = self.current_target[3] - self.current_state[3]
        err_yaw = (err_yaw + math.pi) % (2 * math.pi) - math.pi

        msg = Float64MultiArray()
        # Tableau : [Erreur X, Erreur Y, Erreur Z, Erreur Yaw]
        msg.data = [float(err_x), float(err_y), float(err_z), float(err_yaw)]
        self.error_pub.publish(msg)
        # ------------------------------------------

        try:
            x0 = self.current_state.reshape(-1, 1)
            u0 = self.mpc.make_step(x0)
            cmd = u0.flatten()

            for i in range(8):
                desired_force = float(cmd[i])
                c = THRUST_COEFFS[i]
                msg = Float64()
                msg.data = desired_force * math.copysign(1.0, c)
                self.pubs[i].publish(msg)

        except Exception as e:
            self.get_logger().error(f"MPC Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MPCControllerNetInspection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
