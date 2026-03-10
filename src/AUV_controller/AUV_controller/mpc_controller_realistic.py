import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import numpy as np
import do_mpc
import casadi as ca
import math
import time
# --- CONFIGURATION DES PROPULSEURS (Internalisée) ---
_SIN45 = 0.70710678118
THRUST_COEFFS = [-0.02, 0.02, -0.02, 0.02, -0.02, 0.02, 0.02, -0.02]
POSITIONS = [
    [ 0.135, -0.11,  0.0], [ 0.135,  0.11,  0.0],
    [-0.135, -0.11,  0.0], [-0.135,  0.11,  0.0],
    [ 0.12, -0.218,  0.0], [ 0.12,  0.218,  0.0],
    [-0.12, -0.218,  0.0], [-0.12,  0.218,  0.0]
]
DIRECTIONS = [
    [ _SIN45,  _SIN45, 0.0], [ _SIN45, -_SIN45, 0.0],
    [ _SIN45, -_SIN45, 0.0], [ _SIN45,  _SIN45, 0.0],
    [0.0, 0.0, -1.0], [0.0, 0.0,  1.0],
    [0.0, 0.0,  1.0], [0.0, 0.0, -1.0]
]


BUOYANCY_NET = 2.0 


class MPCControllerBlueROV(Node):
    def __init__(self):
        super().__init__('mpc_controller_bluerov')
        
        # --- Parameters ---
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.pubs = []
        for i in range(1, 9):
            self.pubs.append( 
                self.create_publisher(Float64, f'/cmd_vel_{i}', 10)
            )

        # Primary state source: /odometry/filtered (EKF sensor fusion)
        self.sub_odom = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10
        )
        # Secondary/comparison: Gazebo truth (kept for logging)
        self.sub_odom_exacte = self.create_subscription(
            Odometry, '/odom', self.odom_callback_exacte, 10
        )

        self.current_target = np.array([15.0, 2.0, -1.0, 0.0])
        self.current_state = np.zeros(8)
        self.current_state_exacte = np.zeros(8)
        self.odom_received = False
        self.get_logger().info("Waiting for first odometry on /odometry/filtered...")


        self.setup_mpc()


        self.timer = self.create_timer(0.15, self.control_loop)

        self.get_logger().info(
            "BlueROV2 MPC Controller Initialized (8 Thrusters)"
        )


    def odom_callback_exacte(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        vel = msg.twist.twist.linear
        ang = msg.twist.twist.angular

        siny_cosp = 2 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1 - 2 * (o.y * o.y + o.z * o.z)
        psi = np.arctan2(siny_cosp, cosy_cosp)

        self.current_state_exacte = np.array([
            p.x, p.y, p.z, psi,
            vel.x, vel.y, vel.z, ang.z,
        ])
        self.odom_received = True

    # ------------------------------------------------------------------
    #  MPC Model
    # ------------------------------------------------------------------
    def setup_mpc(self):
        model_type = 'continuous'
        self.model = do_mpc.model.Model(model_type)

        # --- 1. States (8) ---
        x   = self.model.set_variable(var_type='_x', var_name='x')
        y   = self.model.set_variable(var_type='_x', var_name='y')
        z   = self.model.set_variable(var_type='_x', var_name='z')
        psi = self.model.set_variable(var_type='_x', var_name='psi')

        u = self.model.set_variable(var_type='_x', var_name='u')   
        v = self.model.set_variable(var_type='_x', var_name='v')   
        w = self.model.set_variable(var_type='_x', var_name='w')   
        r = self.model.set_variable(var_type='_x', var_name='r')   

        # --- 2. Inputs (8 thrusters) ---
        t1 = self.model.set_variable(var_type='_u', var_name='t1')
        t2 = self.model.set_variable(var_type='_u', var_name='t2')
        t3 = self.model.set_variable(var_type='_u', var_name='t3')
        t4 = self.model.set_variable(var_type='_u', var_name='t4')
        t5 = self.model.set_variable(var_type='_u', var_name='t5')
        t6 = self.model.set_variable(var_type='_u', var_name='t6')
        t7 = self.model.set_variable(var_type='_u', var_name='t7')
        t8 = self.model.set_variable(var_type='_u', var_name='t8')

        # --- 3. Physical Parameters ---
        mass_body = 12.9
        m_eff_u = mass_body + 6.36    
        m_eff_v = mass_body + 7.12    
        m_eff_w = mass_body + 12.0    
        Izz_eff = 0.16 + 0.15         

        Xu_lin = 4.03;   Xu_quad = 18.18   
        Yv_lin = 6.22;   Yv_quad = 21.66   
        Zw_lin = 5.18;   Zw_quad = 39.99   
        Nr_lin = 0.07;   Nr_quad = 1.55    

        # --- 4. Allocation Matrix (TAM) ---
        # Computed from empirically-calibrated DIRECTIONS and POSITIONS in thruster_config.py
        # TAM[dof, i] = contribution of thruster i to degree of freedom dof
        # DOFs: [Fx, Fy, Fz, Mx, My, Mz]
        # For each thruster at (px, py, pz) with direction (dx, dy, dz):
        #   Fx=dx, Fy=dy, Fz=dz
        #   Mx = py*dz - pz*dy,  My = pz*dx - px*dz,  Mz = px*dy - py*dx
        # (pz=0 for all, so: Mx=py*dz, My=-px*dz, Mz=px*dy-py*dx)
        import numpy as _np
        _dirs = DIRECTIONS
        _pos  = POSITIONS
        _n = 8
        _TAM = _np.zeros((6, _n))
        for _i in range(_n):
            dx, dy, dz = _dirs[_i]
            px, py, _pz = _pos[_i]
            _TAM[0, _i] = dx                          # Fx
            _TAM[1, _i] = dy                          # Fy
            _TAM[2, _i] = dz                          # Fz
            _TAM[3, _i] = py * dz - _pz * dy         # Mx (roll)
            _TAM[4, _i] = _pz * dx - px * dz         # My (pitch)
            _TAM[5, _i] = px * dy - py * dx          # Mz (yaw)
        TAM = _TAM

        u_vec = ca.vertcat(t1, t2, t3, t4, t5, t6, t7, t8)
        tau = ca.mtimes(TAM, u_vec)

        F_surge = tau[0]
        F_sway  = tau[1]
        F_heave = tau[2]
        M_roll  = tau[3]
        M_pitch = tau[4]
        M_yaw   = tau[5]

        # --- 5. Equations of Motion ---
        self.model.set_rhs('x', u * ca.cos(psi) - v * ca.sin(psi))
        self.model.set_rhs('y', u * ca.sin(psi) + v * ca.cos(psi))
        self.model.set_rhs('z', w)
        self.model.set_rhs('psi', r)

        eps = 1e-4
        self.model.set_rhs('u', (F_surge - Xu_lin*u - Xu_quad*u*ca.sqrt(u**2 + eps)) / m_eff_u)
        self.model.set_rhs('v', (F_sway  - Yv_lin*v - Yv_quad*v*ca.sqrt(v**2 + eps)) / m_eff_v)
         
        # CORRECTION : Ajout de la flottabilité nette
        self.model.set_rhs('w', (F_heave - Zw_lin*w - Zw_quad*w*ca.sqrt(w**2 + eps) + BUOYANCY_NET) / m_eff_w)
        
        self.model.set_rhs('r', (M_yaw   - Nr_lin*r - Nr_quad*r*ca.sqrt(r**2 + eps)) / Izz_eff)

        self.model.setup()

        # --- Controller ---
        self.mpc = do_mpc.controller.MPC(self.model)

        setup_mpc = {
            'n_horizon': 8, 
            't_step': 0.15,  
            'n_robust': 0,
            'store_full_solution': False,
        }
        self.mpc.set_param(**setup_mpc)
        self.mpc.set_param(nlpsol_opts={
            'ipopt.max_iter': 25,
            'ipopt.warm_start_init_point': 'yes',
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.tol': 5e-2,
            'ipopt.acceptable_tol': 1e-1,
            'ipopt.acceptable_obj_change_tol': 1e-1,
        })

        # Cost function — tuned for stability (reduce to prevent overshoot, raise to chase harder)
        x_ref, y_ref, z_ref, psi_ref = self.current_target
        yaw_err = 20.0 * (1 - ca.cos(psi - psi_ref))  # ↑ from 10 (improves heading stability)
        mterm = (
            25.0 * (x - x_ref)**2    # ↓ from 50  (less aggressive, smoother approach)
            + 25.0 * (y - y_ref)**2  # ↓ from 50
            + 50.0 * (z - z_ref)**2  # ↓ from 100 (depth still prioritized)
            + yaw_err
        )
        # Velocity damping: penalizes fast surge/sway to prevent oscillation
        lterm = mterm + 15.0 * r**2 + 20.0 * u**2 + 20.0 * v**2

        lterm += 0.01 * (t1**2 + t2**2 + t3**2 + t4**2   # ↑ from 0.001 (penalizes large thrusts)
                       + t5**2 + t6**2 + t7**2 + t8**2)

        self.mpc.set_objective(mterm=mterm, lterm=lterm)

        # rterm: penalizes rapid changes in thruster commands → smoother behavior
        self.mpc.set_rterm(
            t1=0.1, t2=0.1, t3=0.1, t4=0.1,   # ↑ from 0.01
            t5=0.1, t6=0.1, t7=0.1, t8=0.1,   # ↑ from 0.01
        )

        for name in ['t1', 't2', 't3', 't4', 't5', 't6', 't7', 't8']:
            self.mpc.bounds['lower', '_u', name] = -5.0
            self.mpc.bounds['upper', '_u', name] =  5.0

        u_vars = self.model.u
        t1_, t2_, t3_, t4_ = u_vars['t1'], u_vars['t2'], u_vars['t3'], u_vars['t4']
        t5_, t6_, t7_, t8_ = u_vars['t5'], u_vars['t6'], u_vars['t7'], u_vars['t8']
        
        x_vars = self.model.x
        u_state, r_state = x_vars['u'], x_vars['r']

        u_vec_state = ca.vertcat(t1_, t2_, t3_, t4_, t5_, t6_, t7_, t8_)
        tau_struct = ca.mtimes(TAM, u_vec_state)

        # Updated surge/sway for the corrected TAM:
        # F_surge = sin45*(t1+t2+t3+t4)  — all horizontal thrusters contribute +X
        # F_sway  = sin45*(t1-t2-t3+t4)  — T1,T4 are +Y; T2,T3 are -Y
        sin45 = 0.7071
        F_surge_struct = sin45 * (t1_ + t2_ + t3_ + t4_)
        F_sway_struct  = sin45 * (t1_ - t2_ - t3_ + t4_)

        # Pitch balance: My = 0.12*(t5 - t6 + t7 - t8) = 0  (horizontal thrusters have My=0 since pz=0)
        pitch_torque = 0.12 * (t5_ - t6_ + t7_ - t8_)
        self.mpc.set_nl_cons('eq_pitch_max',  pitch_torque, ub=0.5)
        self.mpc.set_nl_cons('eq_pitch_min', -pitch_torque, ub=0.5)

        # Roll balance: Mx = 0.218*(t5 + t6 - t7 - t8) = 0
        roll_torque = 0.218 * (t5_ + t6_ - t7_ - t8_)
        self.mpc.set_nl_cons('eq_roll_max',  roll_torque, ub=0.5)
        self.mpc.set_nl_cons('eq_roll_min', -roll_torque, ub=0.5)

        self.mpc.set_nl_cons('u_max',  u_state, ub=0.8,  soft_constraint=True, penalty_term_cons=100)
        self.mpc.set_nl_cons('u_min', -u_state, ub=0.8,  soft_constraint=True, penalty_term_cons=100)
        self.mpc.set_nl_cons('r_max',  r_state, ub=1.0,  soft_constraint=True, penalty_term_cons=100)
        self.mpc.set_nl_cons('r_min', -r_state, ub=1.0,  soft_constraint=True, penalty_term_cons=100)

        self.mpc.setup()
        self.mpc.set_initial_guess()

    # ------------------------------------------------------------------
    #  Odometry
    # ------------------------------------------------------------------
    def odom_callback(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        vel = msg.twist.twist.linear
        ang = msg.twist.twist.angular

        siny_cosp = 2 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1 - 2 * (o.y * o.y + o.z * o.z)
        psi = np.arctan2(siny_cosp, cosy_cosp)

        self.current_state = np.array([
            p.x, p.y, p.z, psi,
            vel.x, vel.y, vel.z, ang.z,
        ])
        self.odom_received = True

    # ------------------------------------------------------------------
    #  Control Loop
    # ------------------------------------------------------------------
    def control_loop(self):
        if not self.odom_received:
            return

        try:
            # First solve log
            if not hasattr(self, '_first_solve_done'):
                self.get_logger().info("Starting control loop (First odom received)")
                self._first_solve_done = True
                
            x0 = self.current_state.reshape(-1, 1)

            t0 = time.perf_counter()
            u0 = self.mpc.make_step(x0)
            solve_ms = (time.perf_counter() - t0) * 1000.0

            if solve_ms > 250.0:
                self.get_logger().warn(
                    f"MPC solve too slow: {solve_ms:.0f}ms (budget: 250ms)"
                )

            cmd = u0.flatten()

            for i in range(8):
                desired_force = float(cmd[i])
                c = THRUST_COEFFS[i]

                # REALISTIC MAPPING: F = c * omega * |omega|  => omega = sign(F/c) * sqrt(|F/c|)
                # Note: math.sqrt requires positive argument, abs() handles both directions
                # The sign of (desired_force / c) determines the rotation direction.
                omega = math.copysign(math.sqrt(abs(desired_force) / abs(c)), desired_force / c)

                msg = Float64()
                msg.data = float(omega)
                self.pubs[i].publish(msg)

            if not hasattr(self, '_tick'):
                self._tick = 0
            self._tick += 1
            if self._tick % 20 == 0:
                s = self.current_state
                s2 = self.current_state_exacte
                t = self.current_target
                dist = math.sqrt((s[0]-t[0])**2 + (s[1]-t[1])**2 + (s[2]-t[2])**2)
                self.get_logger().info(
                    f"dist={dist:.2f}m  solve={solve_ms:.0f}ms\n"
                    f"pos vu=({s[0]:.2f},{s[1]:.2f},{s[2]:.2f}) "
                    f"pos exacte=({s2[0]:.2f},{s2[1]:.2f},{s2[2]:.2f}) "
                    f"psi={math.degrees(s[3]):.0f}°\n"
                    f"→ tgt=({t[0]:.1f},{t[1]:.1f},{t[2]:.1f})\n"
                    f"cmd=[{cmd[0]:.1f},{cmd[1]:.1f},{cmd[2]:.1f},{cmd[3]:.1f}|"
                    f"{cmd[4]:.1f},{cmd[5]:.1f},{cmd[6]:.1f},{cmd[7]:.1f}]"
                )

        except Exception as e:
            self.get_logger().error(f"MPC Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MPCControllerBlueROV()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()