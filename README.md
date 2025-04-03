# 5G Coverage Simulation for UAV Tracking
[![GitHub](https://img.shields.io/github/license/shariquetelco/5G_Coverage_Sim)](https://github.com/shariquetelco/5G_Coverage_Sim)
[![Build Status](https://img.shields.io/github/workflow/status/shariquetelco/5G_Coverage_Sim/CI)](https://github.com/shariquetelco/5G_Coverage_Sim/actions)
[![Version](https://img.shields.io/badge/version-1.0-brightgreen)](https://github.com/shariquetelco/5G_Coverage_Sim/releases)

## Author
**Ahmad Sharique**

## Purpose
This simulation helps in understanding how UAVs (Unmanned Aerial Vehicles) can be tracked using 5G networks by estimating their positions through Time of Arrival (TOA) and Time Difference of Arrival (TDOA) methods. The main focus of the simulation is to estimate the position of UAVs based on gNB (gNodeB) signals in Line-of-Sight (LOS) and Non-Line-of-Sight (NLOS) conditions.

## Simulation Overview
The simulation works by generating random positions for gNBs and UAVs, simulating a 5G signal, and estimating the position of UAVs using TOA/TDOA. The main components of the simulation are described below.

### Key Functions

1. **Generate Positions for gNBs and UAVs**
   ```python
   get_gnb_and_uav_positions(num_gnb, num_uav, area_size)

# Generate a Simple 5G Signal Waveform
generate_tx_waveform(carrier, prs, num_gnb, num_uav)
Creates a simulated waveform using a sine wave to represent the 5G signal.

# Compute Path Loss
compute_path_loss(gnb_positions, uav_positions)
Calculates the signal loss based on the distance between each gNB and UAV using a basic logarithmic model.

# Estimate Time of Arrival (TOA)
estimate_toa_values(gnb_positions, uav_positions)
Computes TOA values for each UAV-gNB pair based on signal travel time.

# Convert TOA to TDOA
assemble_toa_detection(toa_info, gnb_truth, time)
Collects TOA measurements.



# LOS vs. NLOS
Line-of-Sight (LOS): A direct connection between gNB and UAV without any obstruction.
Non-Line-of-Sight (NLOS): A connection is obstructed by buildings or other obstacles.

# Requirements
**Python 3.x**
**numpy, matplotlib, ipywidgets**

# License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

You can install the required dependencies using:
pip install numpy matplotlib ipywidgets

License
This project is licensed under the MIT License - see the LICENSE file for details.
