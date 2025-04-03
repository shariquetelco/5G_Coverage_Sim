import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from scipy.linalg import norm

# Define constants
SPEED_OF_SOUND = 343  # m/s (can be changed to RF speed in air if needed)
NOISE_STDDEV = 0.1    # Standard deviation of noise (for simulation)

class TDOATracker:
    def __init__(self, receiver_positions):
        """
        Initialize the tracker with receiver positions.
        
        :param receiver_positions: 2D list or numpy array with receiver positions [(x1, y1), (x2, y2), ...]
        """
        self.receivers = np.array(receiver_positions)
        self.num_receivers = len(self.receivers)
        
    def calculate_true_tdoa(self, object_position):
        """
        Calculate the true TDOA values based on the distance between the object and the receivers.
        
        :param object_position: (x, y) coordinates of the object
        :return: True TDOA values
        """
        distances = np.linalg.norm(self.receivers - object_position, axis=1)
        reference_time = distances[0] / SPEED_OF_SOUND  # Use the first receiver as reference
        tdoa = (distances - distances[0]) / SPEED_OF_SOUND  # TDOA relative to the first receiver
        return tdoa

    def add_noise_to_tdoa(self, true_tdoa):
        """
        Add Gaussian noise to the TDOA values to simulate real-world conditions.
        
        :param true_tdoa: True TDOA values
        :return: Noisy TDOA values
        """
        noise = np.random.normal(0, NOISE_STDDEV, size=true_tdoa.shape)
        noisy_tdoa = true_tdoa + noise
        return noisy_tdoa
    
    def tdoa_position_estimation(self, noisy_tdoa):
        """
        Estimate the position of the object using least squares optimization from noisy TDOA measurements.
        
        :param noisy_tdoa: Noisy TDOA values
        :return: Estimated position of the object
        """
        def residuals(x):
            """
            Residuals function for least squares, calculates the difference between predicted and measured TDOA.
            
            :param x: Position guess (x, y)
            :return: Residuals (difference between predicted and actual TDOA)
            """
            predicted_tdoa = self.calculate_true_tdoa(x)
            return predicted_tdoa - noisy_tdoa
        
        initial_guess = np.mean(self.receivers, axis=0)  # Initial guess as the center of receivers
        result = least_squares(residuals, initial_guess)
        return result.x  # Estimated position
    
# User Input for gNodeB (Receiver) Position

def get_receiver_position():
    """
    Ask the user to input the position of the gNodeB (Receiver).
    
    :return: Position of the gNodeB (Receiver) as (x, y)
    """
    print("Please select the position of the gNodeB (Receiver) within the defined range.")
    print("Range: x: [0, 10], y: [0, 10]")
    x = float(input("Enter the x-coordinate of the gNodeB (Receiver): "))
    y = float(input("Enter the y-coordinate of the gNodeB (Receiver): "))
    
    # Ensure the entered position is within the defined range
    if not (0 <= x <= 10) or not (0 <= y <= 10):
        print("Invalid position! Please enter a position within the defined range.")
        return get_receiver_position()
    
    return (x, y)

# Initialize the Tracker and Setup
print("Welcome to the TDOA-based Object Tracking Simulation!")
receiver_positions = []
for i in range(4):  # Let's assume we have 4 fixed receivers for simplicity
    if i == 0:
        # Allow user to select position of the gNodeB (Receiver)
        gnodeB_pos = get_receiver_position()
        receiver_positions.append(gnodeB_pos)
    else:
        # Randomly place the other receivers within the defined range
        receiver_positions.append((np.random.uniform(0, 10), np.random.uniform(0, 10)))

# Create tracker with user-selected gNodeB and other random receivers
tracker = TDOATracker(receiver_positions)

# Simulate UE (User Equipment) trajectory
# Here, we will assume the user wants to track a moving UE
# Randomly generate a trajectory for the UE within the defined area

ue_trajectory = np.array([(np.random.uniform(0, 10), np.random.uniform(0, 10)) for _ in range(10)])

# Simulate TDOA measurements for each UE position
noisy_tdoas = []
for position in ue_trajectory:
    true_tdoa = tracker.calculate_true_tdoa(position)
    noisy_tdoa = tracker.add_noise_to_tdoa(true_tdoa)
    noisy_tdoas.append(noisy_tdoa)

# Estimate positions of the UE from noisy TDOA
estimated_positions = []
for noisy_tdoa in noisy_tdoas:
    estimated_position = tracker.tdoa_position_estimation(noisy_tdoa)
    estimated_positions.append(estimated_position)

# Visualize the results
true_positions = ue_trajectory
estimated_positions = np.array(estimated_positions)

plt.figure(figsize=(10, 8))
plt.plot(true_positions[:, 0], true_positions[:, 1], 'g-', label="True UE Trajectory")
plt.plot(estimated_positions[:, 0], estimated_positions[:, 1], 'r--', label="Estimated Trajectory")
plt.scatter(np.array(receiver_positions)[:, 0], np.array(receiver_positions)[:, 1], color='blue', marker='x', label="Receivers")
plt.scatter(gnodeB_pos[0], gnodeB_pos[1], color='purple', marker='o', s=100, label="User-selected gNodeB (Receiver)")
plt.legend()
plt.title('Object Tracking using TDOA (User-defined gNodeB)')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.grid(True)
plt.show()

# Performance Metrics: Calculate RMSE (Root Mean Squared Error)
rmse = np.sqrt(np.mean(np.sum((true_positions - estimated_positions) ** 2, axis=1)))
print(f"RMSE: {rmse:.4f}")
