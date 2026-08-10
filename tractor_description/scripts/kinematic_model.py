import numpy as np

class TractorTrailerModel:
    def __init__(self, L0, trailers, dt=0.1, max_steering_angle=np.radians(30), max_drawbar_angle=np.radians(30), max_velocity=5.0):
        """
        Initialize the kinematic model parameters for Tractor + N Trailers.
        
        Args:
            L0 (float): Tractor wheelbase.
            trailers (list of dict): List of trailer configurations. Each dict must contain:
                - 'L_bar': Drawbar length
                - 'L_trl': Trailer length (dolly to axle)
                - 'dh_prev': Hitch offset from the previous unit (tractor or previous trailer)
            dt (float): Time step.
        """
        self.L0 = L0
        self.trailers = trailers
        self.num_trailers = len(trailers)
        self.dt = dt
        self.max_steering_angle = max_steering_angle
        self.max_drawbar_angle = max_drawbar_angle
        self.max_velocity = max_velocity

    def get_state_derivative(self, state, v0, delta):
        """
        Calculate the time derivative of the state.
        
        Args:
            state (np.array): [x0, y0, theta0, theta1, theta2, ..., theta_{2N}]
            
        Returns:
            np.array: Time derivative [dx0, dy0, dtheta0, dtheta1, ..., dtheta_{2N}]
        """
        # Compute full kinematic vector
        q_kin = self.get_kinematic_vector(state, v0, delta)
        
        # Extract state derivatives
        # q_kin structure:
        # [dx0, dy0, dtheta0, v1, dtheta1, v2, dtheta2, ..., v_{2N}, dtheta_{2N}]
        # Indices to extract: 0, 1, 2 (tractor), then 4, 6, 8... (dtheta_i)
        
        indices = [0, 1, 2] # Tractor derivatives
        for i in range(1, 2 * self.num_trailers + 1):
            indices.append(2 + 2 * i) # Index of dtheta_i in q_kin
            
        dx = q_kin[indices]
        
        return dx

    def get_kinematic_vector(self, state, v0, delta):
        """
        Calculate the full kinematic vector using recursive matrix formulation.
        """
        # Unpack Tractor State
        x0, y0, theta0 = state[0], state[1], state[2]
        
        # 1. Tractor Input
        dtheta0 = (v0 / self.L0) * np.tan(delta)
        v0_vec = np.array([v0, dtheta0])
        
        # Helper: Transform A (Tractor/Trailer -> Drawbar)
        def transform_A(v_prev, theta_prev, theta_curr, L_bar, d_h_prev):
            delta_theta = theta_prev - theta_curr
            c, s = np.cos(delta_theta), np.sin(delta_theta)
            M_A = np.array([
                [c, d_h_prev * s],
                [(1/L_bar) * s, -(d_h_prev/L_bar) * c]
            ])
            return M_A @ v_prev

        # Helper: Transform B (Drawbar -> Trailer)
        def transform_B(v_prev, theta_prev, theta_curr, L_trl):
            delta_theta = theta_prev - theta_curr
            c, s = np.cos(delta_theta), np.sin(delta_theta)
            M_B = np.array([
                [c, 0],
                [(1/L_trl) * s, 0]
            ])
            return M_B @ v_prev

        # Recursive Calculation
        kinematic_vars = [] # To store [v_i, dtheta_i] pairs
        
        v_prev_vec = v0_vec
        theta_prev = theta0
        
        # Loop through each trailer unit (Drawbar + Trailer Body)
        for i, trailer in enumerate(self.trailers):
            # Indices in state vector:
            # Tractor: 0, 1, 2
            # Trailer i (Drawbar): 3 + 2*i
            # Trailer i (Body): 3 + 2*i + 1
            idx_drawbar = 3 + 2*i
            idx_trailer = 3 + 2*i + 1
            
            theta_drawbar = state[idx_drawbar]
            theta_trailer = state[idx_trailer]
            
            # --- Drawbar (Transform A) ---
            v_drawbar_vec = transform_A(v_prev_vec, theta_prev, theta_drawbar, trailer['L_bar'], trailer['dh_prev'])
            kinematic_vars.extend(v_drawbar_vec) # Add v, dtheta
            
            # --- Trailer Body (Transform B) ---
            v_trailer_vec = transform_B(v_drawbar_vec, theta_drawbar, theta_trailer, trailer['L_trl'])
            kinematic_vars.extend(v_trailer_vec) # Add v, dtheta
            
            # Update for next iteration
            v_prev_vec = v_trailer_vec
            theta_prev = theta_trailer

        # Assemble Full Vector
        dx0 = v0 * np.cos(theta0)
        dy0 = v0 * np.sin(theta0)
        
        # [dx0, dy0, dtheta0, v1, dtheta1, v2, dtheta2, ...]
        q_kin = np.concatenate(([dx0, dy0, dtheta0], kinematic_vars))
        
        return q_kin

    def update(self, state, v0, delta):
        """
        Update the state using Euler integration.
        """
        # Clamp Steering Angle
        delta = np.clip(delta, -self.max_steering_angle, self.max_steering_angle)
        
        # Clamp Velocity
        v0 = np.clip(v0, -self.max_velocity, self.max_velocity)
        
        state = np.array(state)
        derivative = self.get_state_derivative(state, v0, delta)
        new_state = state + derivative * self.dt
        
        # Helper to normalize and clamp relative angle
        def clamp_angle(theta_curr, theta_prev, max_angle):
            rel = theta_curr - theta_prev
            while rel > np.pi: rel -= 2*np.pi
            while rel < -np.pi: rel += 2*np.pi
            
            if rel > max_angle:
                return theta_prev + max_angle
            elif rel < -max_angle:
                return theta_prev - max_angle
            return theta_curr

        # Clamp Drawbar Angles
        # Drawbar angles are at indices 3, 5, 7, ... (3 + 2*i)
        # Previous angles (Tractor/Trailer) are at 2, 4, 6, ... (2 + 2*i)
        for i in range(self.num_trailers):
            idx_curr = 3 + 2*i
            idx_prev = 2 + 2*i
            new_state[idx_curr] = clamp_angle(new_state[idx_curr], new_state[idx_prev], self.max_drawbar_angle)
        
        return new_state

    def get_coordinates(self, state):
        """
        Calculate coordinates of all key points for visualization.
        Returns a list of points: [p0, p0_f, h1, p1, p2, h2, p3, p4, ...]
        """
        x0, y0, theta0 = state[0], state[1], state[2]
        
        coords = []
        
        # Tractor Rear Axle
        p0 = np.array([x0, y0])
        coords.append(p0)
        
        # Tractor Front Axle
        p0_f = p0 + self.L0 * np.array([np.cos(theta0), np.sin(theta0)])
        coords.append(p0_f)
        
        p_prev = p0
        theta_prev = theta0
        
        for i, trailer in enumerate(self.trailers):
            # Indices
            idx_drawbar = 3 + 2*i
            idx_trailer = 3 + 2*i + 1
            
            theta_drawbar = state[idx_drawbar]
            theta_trailer = state[idx_trailer]
            
            # Hitch (from previous unit)
            h_curr = p_prev - trailer['dh_prev'] * np.array([np.cos(theta_prev), np.sin(theta_prev)])
            coords.append(h_curr)
            
            # Dolly Axle (from Hitch)
            p_dolly = h_curr - trailer['L_bar'] * np.array([np.cos(theta_drawbar), np.sin(theta_drawbar)])
            coords.append(p_dolly)
            
            # Trailer Axle (from Dolly)
            p_trailer = p_dolly - trailer['L_trl'] * np.array([np.cos(theta_trailer), np.sin(theta_trailer)])
            coords.append(p_trailer)
            
            # Update for next iteration
            p_prev = p_trailer
            theta_prev = theta_trailer
            
        return coords
