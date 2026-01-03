import numpy as np
import matplotlib.pyplot as plt

# =================================================================
# Simulation Parameters
# =================================================================
KP = 2.0     # Attractive potential gain (proportional to goal distance)
ETA = 0.03   # Repulsive potential gain (scaling the avoidance force)

# Environment obstacles and geometry
obstacle_1 = [-0.3, 0.8]  # Center of the circular obstacle
ro = 0.2            # Radius of the obstacle
Qs = 0.1            # Influence distance (threshold where repulsion begins)
alpha = 0.005       # Gradient descent step size (learning rate)

def attractive_force(current_position, goal_position):
    """
    Calculates the attractive force directing the agent toward the goal.
    Formula: F_att = KP * (current_position - goal_position)
    """
    return KP * (current_position - goal_position)

def repulsive_force(current_position):
    """
    Calculates the total repulsive force from obstacles and boundaries.
    Uses the gradient of the repulsive potential field.
    """
    F = np.zeros((2,))

    # --- Repulsion from circular obstacle (obstacle_1) ---
    d = np.linalg.norm(current_position - obstacle_1)
    if d < ro + Qs:
        # Repulsive force increases as distance 'd' decreases
        F += 2 * ETA * (1/Qs - 1/d) * (1/d**2) * current_position
        
    # --- Repulsion from upper wall boundary (y = 1.2) ---
    d_upper = np.abs(current_position[1] - 1.2)
    if d_upper < Qs:
        F += [0, 2 * ETA * (1/Qs - 1/d_upper) * (1/d_upper**2) * current_position[1]]
        
    # --- Repulsion from lower wall boundary (y = 0.4) ---
    d_lower = np.abs(current_position[1] - 0.4)
    if d_lower < Qs:
        F += [0, 2 * ETA * (1/Qs - 1/d_lower) * (1/d_lower**2) * current_position[1]]
        
    return F

# =================================================================
# Visualization Setup
# =================================================================
circle1 = plt.Circle((obstacle_1[0], obstacle_1[1]), ro, color='r', label='Obstacle')

fig, ax = plt.subplots()
ax.add_artist(circle1)

# Start and Goal positions
xs = np.array([0.5, 1])     # Starting point (Red Square)
xg = np.array([-1.3, 0.5])  # Goal point (Blue Square)

plt.plot(xs[0], xs[1], 'sr', label='Start')
plt.plot(xg[0], xg[1], 'sb', label='Goal')

# =================================================================
# Path Planning Loop (Gradient Descent)
# =================================================================
x = np.copy(xs)
X = [] # List to store path coordinates

while True:
    X.append(np.copy(x))
    
    # Total force is the vector sum of attractive and repulsive components
    F = attractive_force(x, xg) + repulsive_force(x)
    
    # Update position: move in the direction opposite to the gradient
    x -= alpha * F
    
    # Convergence criteria: stop if the force magnitude is below a threshold
    if np.linalg.norm(F) < 0.03:
        X.append(np.copy(x))
        break

# Plot the generated path
X = np.array(X)
plt.plot(X[:,0], X[:,1], 'k-', label='Robot Path')

# Formatting the plot
plt.axis('equal')
plt.xlim([-1.5, 1.5])
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Robot Path Planning using Artificial Potential Fields')
plt.legend()
plt.show()