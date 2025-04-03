import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from scipy.linalg import norm
from filterpy.kalman import KalmanFilter

# Define constants
SPEED_OF_SOUND = 343  # m/s (speed of sound in air, adjust if RF speed is required)
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

class VehicleTrackingSystem:
    def __init__(self, receiver_positions):
        self.tracker = TDOATracker(receiver_positions)
        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        self.kf.F = np.array([[1, 0, 1, 0],  # State transition matrix (constant velocity model)
                              [0, 1, 0, 1],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])
        self.kf.H = np.array([[1, 0, 0, 0],  # Measurement matrix
                              [0, 1, 0, 0]])
        self.kf.P *= 1000  # Initial uncertainty
        self.kf.R = np.eye(2) * 0.1  # Measurement noise
        self.kf.Q = np.eye(4) * 0.1  # Process noise

    def estimate_position(self, noisy_tdoa):
        # Use TDOA to estimate the position
        estimated_position = self.tracker.tdoa_position_estimation(noisy_tdoa)
        return estimated_position

    def update_kalman_filter(self, z):
        """
        Update Kalman filter with the new measurement z (estimated position).
        
        :param z: New measurement (x, y)
        :return: Updated state and covariance
        """
        self.kf.predict()
        self.kf.update(z)
        return self.kf.x[:2]

# Simulation setup
receiver_positions = [(0, 0), (10, 0), (0, 10), (10, 10)]  # 4 fixed receivers in a square
true_vehicle_trajectory = np.array([(5, 5), (6, 6), (7, 7), (8, 8), (9, 9), (10, 10), (11, 11), (12, 12), (13, 13)])

# Create vehicle tracking system
vehicle_tracking_system = VehicleTrackingSystem(receiver_positions)

# Simulate TDOA measurements for each vehicle position
noisy_tdoas = []
for position in true_vehicle_trajectory:
    true_tdoa = vehicle_tracking_system.tracker.calculate_true_tdoa(position)
    noisy_tdoa = vehicle_tracking_system.tracker.add_noise_to_tdoa(true_tdoa)
    noisy_tdoas.append(noisy_tdoa)

# Estimate positions of the vehicle using TDOA and Kalman filter
estimated_positions_tdoa = []
estimated_positions_kf = []
for noisy_tdoa in noisy_tdoas:
    # Estimate position using TDOA
    estimated_position_tdoa = vehicle_tracking_system.estimate_position(noisy_tdoa)
    estimated_positions_tdoa.append(estimated_position_tdoa)

    # Update Kalman filter with the new estimated position
    estimated_position_kf = vehicle_tracking_system.update_kalman_filter(estimated_position_tdoa)
    estimated_positions_kf.append(estimated_position_kf)

# Visualize the results
true_positions = true_vehicle_trajectory
estimated_positions_tdoa = np.array(estimated_positions_tdoa)
estimated_positions_kf = np.array(estimated_positions_kf)

plt.figure(figsize=(10, 8))
plt.plot(true_positions[:, 0], true_positions[:, 1], 'g-', label="True Vehicle Trajectory")
plt.plot(estimated_positions_tdoa[:, 0], estimated_positions_tdoa[:, 1], 'r--', label="Estimated Trajectory (TDOA)")
plt.plot(estimated_positions_kf[:, 0], estimated_positions_kf[:, 1], 'b-.', label="Estimated Trajectory (Kalman Filter)")
plt.scatter(np.array(receiver_positions)[:, 0], np.array(receiver_positions)[:, 1], color='blue', marker='x', label="Receivers")
plt.legend()
plt.title('Urban Air Mobility (UAM) Vehicle Tracking with 5G PRS')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.grid(True)
plt.show()

# Performance Metrics: Calculate RMSE (Root Mean Squared Error) for Kalman Filter
estimated_positions_kf = np.squeeze(np.array(estimated_positions_kf))  # Remove unnecessary dimensions
rmse_kf = np.sqrt(np.mean(np.sum((true_positions - estimated_positions_kf) ** 2, axis=1)))
print(f"RMSE (Kalman Filter): {rmse_kf:.4f}")