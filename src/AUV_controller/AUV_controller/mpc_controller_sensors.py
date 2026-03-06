import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import numpy as np
import do_mpc
import casadi as ca
import math
import time

# Constants for thruster coefficients and water density
THRUST_COEFFS = [-0.002, 0.002, 0.002, -0.002, -0.002, 0.002, 0.002, -0.002]
RHO = 997.0

# Net buoyancy force representing the difference between weight and Archimedes' thrust
BUOYANCY_NET = 2.0 

class MPCControllerBlueROV(Node):
    """
    Model Predictive Control (MPC) node for the BlueROV2.
    
    This node subscribes to filtered odometry (from an EKF) and calculates the optimal
    thruster commands to reach a target position while respecting the robot's hydrodynamic
    model and physical constraints (e.g., maximum thrust, maximum pitch/roll torques).
    """
    def __init__(self):
        super().__init__('mpc_controller_bluerov')

        # Publishers for the 8 thrusters commands
        self.pubs = []
        for i in range(1, 9):
            self.pubs.append( 
                self.create_publisher(Float64, f'/cmd_vel_{i}', 10)
            )

        # Subscribers for odometry
        # /odometry/filtered is the output of the EKF (Sensor Fusion)
        self.sub_odom = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10
        )
        # /odom is the exact ground truth from Gazebo (used for comparison/debugging)
        self.sub_odom_exacte = self.create_subscription(
            Odometry, '/odom', self.odom_callback_exacte, 10
        )

        # Target setpoint: [x, y, z, yaw]
        self.current_target = np.array([15.0, 2.0, -1.0, 0.0])
        
        # Current state: [x, y, z, psi, u, v, w, r]
        self.current_state = np.zeros(8)
        self.current_state_exacte = np.zeros(8)
        self.odom_received = False

        # Initialize the do_mpc mathematical model
        self.setup_mpc()

        # Main control loop running at 4Hz (0.25s period)
        self.timer = self.create_timer(0.25, self.control_loop)

        self.get_logger().info(
            "BlueROV2 MPC Controller Initialized (8 Thrusters)"
        )


    def odom_callback_exacte(self, msg):
        """Callback to store exact odometry from Gazebo for debugging purposes."""
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        vel = msg.twist.twist.linear
        ang = msg.twist.twist.angular

        # Convert quaternion to Euler angles (yaw extraction)
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
        """
        Defines the mathematical model of the BlueROV2, the cost function, 
        and configures the non-linear solver (CasADi / do_mpc).
        """
        model_type = 'continuous'
        self.model = do_mpc.model.Model(model_type)

        # --- 1. States (8 variables) ---
        # 4 Poses (Earth frame)
        x   = self.model.set_variable(var_type='_x', var_name='x')
        y   = self.model.set_variable(var_type='_x', var_name='y')
        z   = self.model.set_variable(var_type='_x', var_name='z')
        psi = self.model.set_variable(var_type='_x', var_name='psi') # Yaw

        # 4 Velocities (Body frame: surge, sway, heave, yaw rate)
        u = self.model.set_variable(var_type='_x', var_name='u')   
        v = self.model.set_variable(var_type='_x', var_name='v')   
        w = self.model.set_variable(var_type='_x', var_name='w')   
        r = self.model.set_variable(var_type='_x', var_name='r')   

        # --- 2. Inputs (8 variables for 8 thrusters) ---
        t1 = self.model.set_variable(var_type='_u', var_name='t1')
        t2 = self.model.set_variable(var_type='_u', var_name='t2')
        t3 = self.model.set_variable(var_type='_u', var_name='t3')
        t4 = self.model.set_variable(var_type='_u', var_name='t4')
        t5 = self.model.set_variable(var_type='_u', var_name='t5')
        t6 = self.model.set_variable(var_type='_u', var_name='t6')
        t7 = self.model.set_variable(var_type='_u', var_name='t7')
        t8 = self.model.set_variable(var_type='_u', var_name='t8')

        # --- 3. Physical Parameters (Added mass & Drag) ---
        mass_body = 12.9
        m_eff_u = mass_body + 6.36    
        m_eff_v = mass_body + 7.12    
        m_eff_w = mass_body + 12.0    
        Izz_eff = 0.16 + 0.15         

        Xu_lin = 4.03;   Xu_quad = 18.18   
        Yv_lin = 6.22;   Yv_quad = 21.66   
        Zw_lin = 5.18;   Zw_quad = 39.99   
        Nr_lin = 0.07;   Nr_quad = 1.55    

        # --- 4. Thruster Allocation Matrix (TAM) ---
        # Converts individual thruster forces into total forces and torques acting on the body
        sin45 = 0.7071
        lever = 0.1697
        z_arm = 0.1
        
        TAM = np.array([
            [ sin45,        sin45,       -sin45,       -sin45,        0.0,    0.0,    0.0,    0.0 ], # F_surge (X)
            [ sin45,       -sin45,        sin45,       -sin45,        0.0,    0.0,    0.0,    0.0 ], # F_sway (Y)
            [ 0.0,          0.0,          0.0,          0.0,         -1.0,    1.0,    1.0,   -1.0 ], # F_heave (Z)
            [-z_arm*sin45,  z_arm*sin45, -z_arm*sin45,  z_arm*sin45,  0.218,  0.218,  0.218,  0.218 ], # M_roll
            [ z_arm*sin45,  z_arm*sin45, -z_arm*sin45, -z_arm*sin45,  0.12,  -0.12,   0.12,  -0.12  ], # M_pitch
            [ lever,       -lever,       -lever,        lever,        0.0,    0.0,    0.0,    0.0 ]  # M_yaw
        ])

        u_vec = ca.vertcat(t1, t2, t3, t4, t5, t6, t7, t8)
        tau = ca.mtimes(TAM, u_vec)

        F_surge = tau[0]
        F_sway  = tau[1]
        F_heave = tau[2]
        M_roll  = tau[3]
        M_pitch = tau[4]
        M_yaw   = tau[5]

        # --- 5. Equations of Motion (Kinematics & Dynamics) ---
        # Kinematics: converting body velocities to earth velocities
        self.model.set_rhs('x', u * ca.cos(psi) - v * ca.sin(psi))
        self.model.set_rhs('y', u * ca.sin(psi) + v * ca.cos(psi))
        self.model.set_rhs('z', w)
        self.model.set_rhs('psi', r)

        # Dynamics: F = m*a => a = F/m (including linear and quadratic drag)
        eps = 1e-4 # Epsilon to avoid divide-by-zero or non-differentiable sqrt(0)
        self.model.set_rhs('u', (F_surge - Xu_lin*u - Xu_quad*u*ca.sqrt(u**2 + eps)) / m_eff_u)
        self.model.set_rhs('v', (F_sway  - Yv_lin*v - Yv_quad*v*ca.sqrt(v**2 + eps)) / m_eff_v)
         
        # CORRECTION : Adding Net Buoyancy to Z axis dynamics
        self.model.set_rhs('w', (F_heave - Zw_lin*w - Zw_quad*w*ca.sqrt(w**2 + eps) + BUOYANCY_NET) / m_eff_w)
        
        self.model.set_rhs('r', (M_yaw   - Nr_lin*r - Nr_quad*r*ca.sqrt(r**2 + eps)) / Izz_eff)

        self.model.setup()

        # --- 6. Controller Setup ---
        self.mpc = do_mpc.controller.MPC(self.model)

        setup_mpc = {
            'n_horizon': 10, # Prediction horizon
            't_step': 0.15,  # Time step for prediction
            'n_robust': 0,
            'store_full_solution': False,
        }
        self.mpc.set_param(**setup_mpc)
        
        # Solver options (IPOPT)
        self.mpc.set_param(nlpsol_opts={
            'ipopt.max_iter': 25,
            'ipopt.warm_start_init_point': 'yes',
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.tol': 1e-2,
            'ipopt.acceptable_tol': 5e-2,
            'ipopt.acceptable_obj_change_tol': 5e-2,
        })

        # --- 7. Cost function (Objective) ---
        x_ref, y_ref, z_ref, psi_ref = self.current_target
        yaw_err = 10.0 * (1 - ca.cos(psi - psi_ref))  # Penalty for yaw deviation
        
        # mterm: terminal cost
        mterm = (
            50.0 * (x - x_ref)**2
            + 50.0 * (y - y_ref)**2
            + 100.0 * (z - z_ref)**2
            + yaw_err
        )
        
        # lterm: stage cost (cost at each step along the horizon)
        lterm = mterm + 10.0 * r**2  

        # Penalty on using excessive thruster force (energy saving and smoothing)
        lterm += 0.001 * (t1**2 + t2**2 + t3**2 + t4**2
                        + t5**2 + t6**2 + t7**2 + t8**2)

        self.mpc.set_objective(mterm=mterm, lterm=lterm)

        # Rterm: Penalty on the change of thruster inputs between steps (smooth commands)
        self.mpc.set_rterm(
            t1=0.01, t2=0.01, t3=0.01, t4=0.01,
            t5=0.01, t6=0.01, t7=0.01, t8=0.01,
        )

        # Limits for thruster forces
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

        # --- 8. Non-Linear Constraints (Stability & Realism) ---
        # Prevent the MPC from commanding thruster combinations that would cause massive pitch/roll
        sin45 = 0.7071
        F_surge_struct = sin45 * (t1_ + t2_ - t3_ - t4_)
        F_sway_struct  = sin45 * (t1_ - t2_ + t3_ - t4_)

        pitch_torque_balance = (t5_ - t6_ + t7_ - t8_) + (0.1 / 0.12) * F_surge_struct
        self.mpc.set_nl_cons('eq_pitch_max', pitch_torque_balance, ub=0.5)
        self.mpc.set_nl_cons('eq_pitch_min', -pitch_torque_balance, ub=0.5)
        
        roll_torque_balance = (t5_ + t6_ + t7_ + t8_) - (0.1 / 0.218) * F_sway_struct
        self.mpc.set_nl_cons('eq_roll_max', roll_torque_balance, ub=0.5)
        self.mpc.set_nl_cons('eq_roll_min', -roll_torque_balance, ub=0.5)

        # Soft constraints on maximum velocities and yaw rate
        self.mpc.set_nl_cons('u_max',  u_state, ub=2.0,  soft_constraint=True, penalty_term_cons=100)
        self.mpc.set_nl_cons('u_min', -u_state, ub=2.0,  soft_constraint=True, penalty_term_cons=100)
        self.mpc.set_nl_cons('r_max',  r_state, ub=2.0,  soft_constraint=True, penalty_term_cons=100)
        self.mpc.set_nl_cons('r_min', -r_state, ub=2.0,  soft_constraint=True, penalty_term_cons=100)

        self.mpc.setup()
        self.mpc.set_initial_guess()

    # ------------------------------------------------------------------
    #  Odometry Callback
    # ------------------------------------------------------------------
    def odom_callback(self, msg):
        """
        Callback to update the current state from EKF odometry.
        Extracts position, yaw angle, and linear/angular velocities.
        """
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        vel = msg.twist.twist.linear
        ang = msg.twist.twist.angular

        # Convert quaternion to yaw based on Z axis rotation
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
        """
        Main loop: takes current state, feeds it to the MPC solver, and publishes commands.
        Executes iteratively based on node timer frequency.
        """
        if not self.odom_received:
            return

        try:
            x0 = self.current_state.reshape(-1, 1)

            # Solve MPC step measuring time taken
            t0 = time.perf_counter()
            u0 = self.mpc.make_step(x0)
            solve_ms = (time.perf_counter() - t0) * 1000.0

            # Warning if solver takes too long (exceeds budget)
            if solve_ms > 250.0:
                self.get_logger().warn(
                    f"MPC solve too slow: {solve_ms:.0f}ms (budget: 250ms)"
                )

            # Retrieve computed forces: u0 contains [t1...t8]
            cmd = u0.flatten()

            # Publish thruster commands with appropriate conversion coefficients
            for i in range(8):
                desired_force = float(cmd[i])
                c = THRUST_COEFFS[i]

                msg = Float64()
                # Apply coefficient sign to compensate for Gazebo propeller rotation definition
                msg.data = desired_force * math.copysign(1.0, c)
                self.pubs[i].publish(msg)

            # Logger logic for telemetry (executed every ~20 ticks)
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
                    f"pos=({s[0]:.2f},{s[1]:.2f},{s[2]:.2f}) "
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