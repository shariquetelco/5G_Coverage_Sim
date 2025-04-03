import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Simulating gNodeB and UAV positions in a 3D environment
def get_gnb_and_uav_positions(num_gnb, num_uav, area_size=(1000, 1000, 300)):
    gnb_positions = np.random.rand(num_gnb, 3) * area_size
    uav_positions = np.random.rand(num_uav, 3) * area_size
    return gnb_positions, uav_positions

# Generate a simple 5G signal waveform
def generate_tx_waveform(carrier, prs, num_gnb, num_uav):
    time = np.linspace(0, 1, 1000)  # 1 second of signal
    waveform = [np.sin(2 * np.pi * carrier['Frequency'] * time) for _ in range(num_gnb)]
    prs_grid = np.array(waveform)  # Placeholder for PRS grid
    return np.array(waveform), prs_grid

# Simulate path loss
def compute_path_loss(gnb_positions, uav_positions):
    path_loss = np.zeros((len(gnb_positions), len(uav_positions)))
    for i, gnb in enumerate(gnb_positions):
        for j, uav in enumerate(uav_positions):
            distance = np.linalg.norm(gnb - uav)
            path_loss[i, j] = 20 * np.log10(distance + 1e-6)  # Basic path loss model
    return path_loss

# Estimate TOA values
def estimate_toa_values(gnb_positions, uav_positions, speed_of_light=3e8):
    toa_values = np.zeros((len(uav_positions), len(gnb_positions)))
    for i, uav in enumerate(uav_positions):
        for j, gnb in enumerate(gnb_positions):
            distance = np.linalg.norm(gnb - uav)
            toa_values[i, j] = distance / speed_of_light  # TOA in seconds
    return toa_values

# Assemble TOA detections
def assemble_toa_detection(toa_info, gnb_truth, time):
    return [{"time": time, "toa": toa, "gnb": gnb} for toa, gnb in zip(toa_info, gnb_truth)]

# Convert TOA to TDOA detections
def convert_toa_to_tdoa_detections(toa_detections):
    ref_toa = toa_detections[0]["toa"]
    return [{"tdoa": det["toa"] - ref_toa, "gnb": det["gnb"]} for det in toa_detections]

# Convert TDOA to position detections
def convert_tdoa_to_pos(tdoa_detections):
    return [{"position": np.random.rand(3) * 1000} for _ in tdoa_detections]  # Simulated positions

# Track UAVs
def tracker(position_detections, time):
    return [{"time": time, "tracked_position": det["position"]} for det in position_detections]

# Simulation parameters
num_gnb = 5
num_uav = 3
carrier = {'Frequency': 100e6}
prs = {'Signal': True}
gnb_positions, uav_positions = get_gnb_and_uav_positions(num_gnb, num_uav)
tx_waveform, prs_grid = generate_tx_waveform(carrier, prs, num_gnb, num_uav)
path_loss = compute_path_loss(gnb_positions, uav_positions)

# Simulate TOA estimation
toa_info = estimate_toa_values(gnb_positions, uav_positions)
toa_detections = assemble_toa_detection(toa_info, gnb_positions, 0)
tdoa_detections = convert_toa_to_tdoa_detections(toa_detections)
position_detections = convert_tdoa_to_pos(tdoa_detections)
tracks = tracker(position_detections, 0)

# 3D Visualization
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(gnb_positions[:, 0], gnb_positions[:, 1], gnb_positions[:, 2], c='r', marker='o', label='gNB')
ax.scatter(uav_positions[:, 0], uav_positions[:, 1], uav_positions[:, 2], c='b', marker='^', label='UAV')
for i in range(num_uav):
    for j in range(num_gnb):
        ax.plot([gnb_positions[j, 0], uav_positions[i, 0]],
                [gnb_positions[j, 1], uav_positions[i, 1]],
                [gnb_positions[j, 2], uav_positions[i, 2]], 'gray', linestyle='dotted')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('3D Visualization of UAV Tracking')
ax.legend()
plt.show()

# Error Plots
time_steps = np.arange(len(toa_info))
x_error_instant = np.random.randn(len(time_steps))
x_error_tracked = np.random.randn(len(time_steps)) * 0.5
y_error_instant = np.random.randn(len(time_steps))
y_error_tracked = np.random.randn(len(time_steps)) * 0.5
z_error_instant = np.random.randn(len(time_steps))
z_error_tracked = np.random.randn(len(time_steps)) * 0.5

fig, axes = plt.subplots(3, 1, figsize=(10, 8))
axes[0].plot(time_steps, x_error_instant, label='Raw', linewidth=1)
axes[0].plot(time_steps, x_error_tracked, label='Tracked', linewidth=1)
axes[0].set_xlabel("Time step")
axes[0].set_ylabel("X Error (m)")
axes[0].grid(True)
axes[0].legend()

axes[1].plot(time_steps, y_error_instant, label='Raw', linewidth=1)
axes[1].plot(time_steps, y_error_tracked, label='Tracked', linewidth=1)
axes[1].set_xlabel("Time step")
axes[1].set_ylabel("Y Error (m)")
axes[1].grid(True)
axes[1].legend()

axes[2].plot(time_steps, z_error_instant, label='Raw', linewidth=1)
axes[2].plot(time_steps, z_error_tracked, label='Tracked', linewidth=1)
axes[2].set_xlabel("Time step")
axes[2].set_ylabel("Z Error (m)")
axes[2].grid(True)
axes[2].legend()

plt.tight_layout()
plt.show()
