import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class robot_arm():
    # Robot link lengths and environment constraints
    l1, l2, l3 = 1, 0.8, 0.7
    obstacle_1, obstacle_2 = np.array([0.05, 1]), np.array([-0.6, 0.6])
    r = 0.2
    lim_min = np.array([-np.pi, 0, -np.pi])
    lim_max = np.array([np.pi, np.pi, np.pi])

    def __init__(self):
        pass

    def inverse_kinematics(self, X):
        """Converts end-effector pose (x, y, phi) to joint angles (q1, q2, q3)"""
        px = X[0] - self.l3 * np.cos(X[2])
        py = X[1] - self.l3 * np.sin(X[2])

        D = (px**2 + py**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        D = np.clip(D, -1, 1) 

        q = np.zeros(3)
        q2_1 = np.arctan2(np.sqrt(1 - D**2), D)
        q2_2 = np.arctan2(-np.sqrt(1 - D**2), D)
        
        q[1] = q2_1 if q2_1 > 0 else q2_2

        alpha = np.arctan2(self.l2 * np.sin(q[1]), self.l1 + self.l2 * np.cos(q[1]))
        q[0] = np.arctan2(py, px) - alpha
        q[2] = X[2] - q[0] - q[1]
        return q

    def direct_kinematics(self, q):
        """Calculates end-effector pose from joint angles"""
        x = self.l1 * np.cos(q[0]) + self.l2 * np.cos(q[0] + q[1]) + self.l3 * np.cos(np.sum(q))
        y = self.l1 * np.sin(q[0]) + self.l2 * np.sin(q[0] + q[1]) + self.l3 * np.sin(np.sum(q))
        phi = np.sum(q)
        return np.array([x, y, phi])

    def check_for_collision(self, q):
        """Validates configuration against obstacles, workspace boundaries, and joint limits"""
        x = self.direct_kinematics(q)
        # Obstacle collision check
        if np.linalg.norm(x[:2] - self.obstacle_1) <= self.r or \
           np.linalg.norm(x[:2] - self.obstacle_2) <= self.r:
            return True
        # Workspace Y-axis limits
        if x[1] > 1.2 or x[1] < 0.4: 
            return True
        # Joint angle limits
        if np.any(q < self.lim_min) or np.any(q > self.lim_max):
            return True
        return False

    def get_random_configuration(self):
        """Generates a random joint configuration within limits"""
        return np.random.uniform(self.lim_min, self.lim_max)

    def theta_pol(self, X_nodes, T_segment):
        """Generates a smooth trajectory and returns joint path, Cartesian path, and time vector"""
        num_segments = len(X_nodes) - 1
        points_per_segment = 100
        Q_path = []
        T_path = []
        
        current_time = 0
        for i in range(1, len(X_nodes)):
            q1 = self.inverse_kinematics(X_nodes[i-1])
            q2 = self.inverse_kinematics(X_nodes[i])
            
            # Cubic spline coefficients
            a0 = q1
            a1 = np.zeros(3)
            a2 = 3 * (q2 - q1) / (T_segment**2)
            a3 = -2 * (q2 - q1) / (T_segment**3)

            Tt = np.linspace(0, T_segment, num=points_per_segment)
            for t in Tt:
                q = a3 * t**3 + a2 * t**2 + a1 * t + a0
                Q_path.append(q)
                T_path.append(current_time + t)
            
            current_time += T_segment
        
        return np.array(Q_path), np.array([self.direct_kinematics(q) for q in Q_path]), np.array(T_path)

    def plot_joint_angles(self, Q_path, T_path):
        """Plots joint angles (in degrees) vs time"""
        plt.figure(figsize=(8, 6))
        # Switch to degrees for better interpretability
        Q_deg = np.degrees(Q_path)
        
        plt.plot(T_path, Q_deg[:, 0], label='q_1')
        plt.plot(T_path, Q_deg[:, 1], label='q_2')
        plt.plot(T_path, Q_deg[:, 2], label='q_3')

        plt.title("Joint Angles vs Time", loc='right', fontsize=14)
        plt.xlabel('t (sec)')
        plt.ylabel('q (deg)')
        plt.legend()
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.show()

    def plot(self, q, Xpath=None, Xn=None):
        """Visualizes the robot arm, obstacles, and the calculated path"""
        X_joints = np.array([
            [0, 0],
            [self.l1 * np.cos(q[0]), self.l1 * np.sin(q[0])],
            [self.l1 * np.cos(q[0]) + self.l2 * np.cos(q[0] + q[1]), 
             self.l1 * np.sin(q[0]) + self.l2 * np.sin(q[0] + q[1])],
            [self.l1 * np.cos(q[0]) + self.l2 * np.cos(q[0] + q[1]) + self.l3 * np.cos(np.sum(q)),
             self.l1 * np.sin(q[0]) + self.l2 * np.sin(q[0] + q[1]) + self.l3 * np.sin(np.sum(q))]
        ])

        fig, ax = plt.subplots()
        circle1 = plt.Circle(self.obstacle_1, self.r, color='g', alpha=0.5)
        circle2 = plt.Circle(self.obstacle_2, self.r, color='g', alpha=0.5)
        ax.add_artist(circle1)
        ax.add_artist(circle2)
        

        plt.plot(X_joints[:, 0], X_joints[:, 1], 'o-k', linewidth=3)
        
        if Xpath is not None:
            plt.plot(Xpath[:, 0], Xpath[:, 1], '--b', label='Path')
        if Xn is not None:
            plt.plot(Xn[:, 0], Xn[:, 1], 'or', label='Nodes')

        plt.axis('equal')
        plt.grid(True)
        plt.xlim([-2, 2])
        plt.ylim([-0.5, 2])
        plt.show()

# --- Main Execution ---
R = robot_arm()
x_start = np.array([0.5, 1, 3.4])
x_goal = np.array([-1.3, 0.5, 3.4])

#   define parameters for potential fields
Xn = np.array([x_start, [0.17, 0.63, 3.4], [-0.3, 0.8, 3.4], x_goal])

Q_traj, X_traj, T_traj = R.theta_pol(Xn, 1.0) 

R.plot_joint_angles(Q_traj, T_traj)

q_start = R.inverse_kinematics(x_start)
R.plot(q_start, Xpath=X_traj, Xn=Xn)