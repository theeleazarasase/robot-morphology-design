import numpy as np
import math

class SineWaveController:
    """
    An open-loop Central Pattern Generator (CPG) that produces 
    antiphase sinusoidal trajectories for bipedal locomotion.
    """
    
    def __init__(self, amplitude=0.5, frequency=1.0, phase_offset=0.0):
        """
        Initialize the CPG parameters.
        
        Args:
            amplitude (float): Peak deviation from neutral (radians).
            frequency (float): Oscillation rate (Hz).
            phase_offset (float): Base phase shift (radians).
        """
        self.amplitude = amplitude
        self.frequency = frequency
        self.phase_offset = phase_offset
        self.omega = 2 * np.pi * frequency  # Angular frequency
        
    def get_action(self, time):
        """
        Calculates the target joint angles for the current timestep.
        
        Returns:
            np.array: [left_hip_angle, right_hip_angle]
            
        Math:
            q_left  = A * sin(omega * t)
            q_right = A * sin(omega * t + pi)  <-- Antiphase
        """
        # Left Leg (Phase = 0)
        left_angle = self.amplitude * math.sin(self.omega * time + self.phase_offset)
        
        # Right Leg (Phase = Pi / 180 degrees)
        # We add math.pi to create the "walking" opposition
        right_angle = self.amplitude * math.sin(self.omega * time + self.phase_offset + math.pi)
        
        return np.array([left_angle, right_angle])

# For research robustness, we include a factory function 
# to easily swap controllers later.
def get_controller(name, **kwargs):
    if name == "sine":
        return SineWaveController(**kwargs)
    raise ValueError(f"Unknown controller type: {name}")