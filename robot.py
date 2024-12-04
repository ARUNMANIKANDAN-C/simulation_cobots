import scipy.integrate
import numpy as np

class RoboticArm:
    def __init__(self, dh_params, masses, lengths, gravity=9.81):
        """
        Initialize the robotic arm.
        :param dh_params: List of DH parameters [a, alpha, d, theta]
        :param masses: Masses of the links (kg)
        :param lengths: Lengths of the links (m)
        :param gravity: Gravitational acceleration (m/s²)
        """
        self.dh_params = dh_params
        self.masses = masses
        self.lengths = lengths
        self.gravity = gravity

    def dh_transform(self, a, alpha, d, theta):
        """Compute the DH Transformation matrix."""
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self, theta):
        """Compute the forward kinematics."""
        T = np.eye(4)
        for i, params in enumerate(self.dh_params):
            params[3] = theta[i]  # Update theta in the DH parameters
            T = np.dot(T, self.dh_transform(*params))
        return T

    def compute_jacobian(self, theta):
        """Compute the Jacobian matrix."""
        n = len(theta)
        T = np.eye(4)
        z = np.array([0, 0, 1])
        o = np.array([0, 0, 0])

        Ts = []
        origins = [o]

        for i, params in enumerate(self.dh_params):
            params[3] = theta[i]
            T = np.dot(T, self.dh_transform(*params))
            Ts.append(T)
            origins.append(T[:3, 3])

        J = np.zeros((6, n))
        for i in range(n):
            z_i = Ts[i - 1][:3, 2] if i > 0 else z
            o_i = origins[i]
            o_n = origins[-1]
            J[:3, i] = np.cross(z_i, o_n - o_i)
            J[3:, i] = z_i

        return J

    def compute_velocity_acceleration(self, theta, theta_dot, theta_ddot):
        """Compute velocity and acceleration of the end-effector."""
        J = self.compute_jacobian(theta)
        velocity = np.dot(J, theta_dot)
        linear_velocity = velocity[:3]
        angular_velocity = velocity[3:]

        J_dot = np.zeros_like(J)  # Placeholder; proper computation requires symbolic derivatives
        acceleration = np.dot(J, theta_ddot) + np.dot(J_dot, theta_dot)
        linear_acceleration = acceleration[:3]
        angular_acceleration = acceleration[3:]

        return linear_velocity, angular_velocity, linear_acceleration, angular_acceleration

    def inverse_kinematics(self, target_position, max_iterations=100, tolerance=1e-6):
        """Perform inverse kinematics to compute joint angles."""
        theta = np.zeros(len(self.dh_params))  # Initial guess for joint angles

        for _ in range(max_iterations):
            T = self.forward_kinematics(theta)
            current_position = T[:3, 3]
            position_error = target_position - current_position
            error_norm = np.linalg.norm(position_error)

            if error_norm < tolerance:
                break

            J = self.compute_jacobian(theta)
            position_jacobian = J[:3, :]  # Linear velocity part of the Jacobian
            delta_theta = np.dot(np.linalg.pinv(position_jacobian), position_error)
            theta += delta_theta

        return theta

    # Previous methods (dh_transform, forward_kinematics, etc.) remain unchanged

    def compute_inverse_dynamics(self, theta, theta_dot, theta_ddot):
        """Compute joint torques using inverse dynamics."""
        n = len(theta)
        M = np.zeros((n, n))  # Mass/Inertia matrix
        G = np.zeros(n)       # Gravity vector
        C = np.zeros((n, n))  # Coriolis matrix

        for i in range(n):
            for j in range(i, n):
                M[i, j] = M[j, i] = self.masses[j] * self.lengths[j]**2  # Simplified inertia
            G[i] = -self.masses[i] * self.gravity * self.lengths[i] * np.cos(theta[i])  # Gravity torque
            for j in range(n):
                for k in range(n):
                    C[i, j] += 0.5 * self.masses[j] * self.lengths[j]**2 * theta_dot[k]

        tau = np.dot(M, theta_ddot) + np.dot(C, theta_dot) + G
        return tau

    def compute_forward_dynamics(self, theta, theta_dot, tau):
        """
        Compute the forward dynamics to get angular accelerations (theta_ddot).
        :param theta: Joint angles (rad)
        :param theta_dot: Joint angular velocities (rad/s)
        :param tau: Joint torques (N·m)
        :return: Angular accelerations (theta_ddot)
        """
        n = len(theta)
        M = np.zeros((n, n))  # Mass/Inertia matrix
        G = np.zeros(n)       # Gravity vector
        C = np.zeros((n, n))  # Coriolis matrix

        for i in range(n):
            for j in range(i, n):
                M[i, j] = M[j, i] = self.masses[j] * self.lengths[j]**2  # Simplified inertia
            G[i] = -self.masses[i] * self.gravity * self.lengths[i] * np.cos(theta[i])  # Gravity torque
            for j in range(n):
                for k in range(n):
                    C[i, j] += 0.5 * self.masses[j] * self.lengths[j]**2 * theta_dot[k]

        # Solve for theta_ddot: tau = M * theta_ddot + C * theta_dot + G
        theta_ddot = np.linalg.solve(M, tau - np.dot(C, theta_dot) - G)
        return theta_ddot

# Example usage
dh_params = [
    [0,          np.pi/2,  0.089,  0],  # Joint 1
    [0.425,      0,        0,      0],  # Joint 2
    [0.392,      0,        0,      0],  # Joint 3
    [0,          np.pi/2,  0.109,  0],  # Joint 4
    [0,         -np.pi/2,  0,      0],  # Joint 5
    [0,          0,        0.082,  0]   # Joint 6
]

masses = [2.0, 1.5, 1.0, 0.5, 0.3, 0.2]  # Mass of each link (kg)
lengths = [0.5, 0.4, 0.3, 0.2, 0.1, 0.1]  # Length of each link (m)
arm = RoboticArm(dh_params, masses, lengths)

# Define initial conditions
theta0 = [0, np.pi/4, -np.pi/2, np.pi/4, np.pi/6, -np.pi/3]
theta_dot0 = [0, 0, 0, 0, 0, 0]
