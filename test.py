import numpy as np
from scipy.optimize import fsolve

# Function to solve the trilateration equations
def equations(vars, landmarks, distances):
    x, y = vars
    eqs = []
    for i, (lx, ly) in enumerate(landmarks):
        distance = distances[i]
        eqs.append((x - lx)**2 + (y - ly)**2 - distance**2)  # Distance formula squared
    return eqs

# Trilateration function using fsolve
def trilateration(landmarks, distances):
    # Ensure there are exactly two landmarks for 2D trilateration
    if len(landmarks) != 2:
        raise ValueError("Exactly two landmarks are required for 2D trilateration.")
    
    # Provide an initial guess for the solver (could be (0, 0) or any other reasonable value)
    initial_guess = (0, 0)
    
    # Use fsolve to find the solution
    position = fsolve(equations, initial_guess, args=(landmarks, distances))
    
    # Return the estimated position
    return position

# Example usage (using only two landmarks)
landmarks = [(0, 0), (10, 0)]  # Landmarks' coordinates
distances = [5, 6]  # Corresponding distances from the mobile device

# Find the position using trilateration
position = trilateration(landmarks, distances)
print(f"Estimated Position: x = {position[0]}, y = {position[1]}")
