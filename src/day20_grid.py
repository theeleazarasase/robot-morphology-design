import os
import glob
import re
import csv
import numpy as np
from experiment_runner import run_episode
from controllers import SineWaveController

def run_grid_search():
    print("Day 20: The Grid (5x10 Parameter Sweep)")
    print("=" * 60)

    # 1. SETUP & DISCOVERY
    script_dir = os.path.dirname(os.path.abspath(__file__))
    generated_dir = os.path.join(script_dir, "../generated")
    output_file = os.path.join(script_dir, "../results/grid_results_day20.csv")
    
    # Create data folder if it doesn't exist
    os.makedirs(os.path.dirname(output_file), exist_ok=True)

    # Find Robots
    search_pattern = os.path.join(generated_dir, "robot_L*.xml")
    xml_files = glob.glob(search_pattern)
    
    # Parse morphologies
    morphologies = []
    regex_pattern = r'robot_L(\d+\.?\d*)'
    for file_path in xml_files:
        match = re.search(regex_pattern, os.path.basename(file_path))
        if match:
            morphologies.append((float(match.group(1)), file_path))
    
    # Sort by Length
    morphologies.sort(key=lambda x: x[0])
    
    # Define Frequencies to test (0.5 Hz to 3.0 Hz, step 0.25)
    # Using numpy to generate the list: [0.5, 0.75, ... 2.75]
    frequencies = np.arange(0.5, 3.0, 0.25)

    print(f" Robots: {len(morphologies)}")
    print(f" Frequencies: {len(frequencies)}")
    print(f" Total Experiments: {len(morphologies) * len(frequencies)}")
    print(f" Saving to: {output_file}")
    print("-" * 60)

    # 2. OPEN CSV & RUN LOOP
    with open(output_file, mode='w', newline='') as f:
        writer = csv.writer(f)
        # Write Header
        writer.writerow(['LegLength', 'Frequency', 'SurvivalTime', 'Distance', 'CoT', 'Speed'])

        count = 0
        total = len(morphologies) * len(frequencies)

        for (leg_length, xml_path) in morphologies:
            for freq in frequencies:
                count += 1
                
                # Feedback to user (Overwriting line to keep console clean)
                print(f"[{count}/{total}] Testing L={leg_length:.2f} @ {freq:.2f}Hz...", end="\r")
                
                # Run Experiment
                try:
                    # Configure Brain
                    brain = SineWaveController(amplitude=0.5, frequency=freq)
                    
                    # Run Sim
                    data = run_episode(model_path=xml_path, controller=brain, duration=5.0)
                    metrics = data['metrics']
                    
                    # Safe CoT string for CSV
                    cot_val = metrics['cot']
                    if cot_val == float('inf'):
                        cot_val = 999.0  # Use a large number instead of 'inf' for easier plotting later
                    
                    # Calculate Average Speed (Distance / Time)
                    # Use ABS distance because we care about magnitude of motion
                    speed = abs(metrics['total_distance']) / metrics['survival_time']

                    # Save Row
                    writer.writerow([
                        leg_length, 
                        freq, 
                        f"{metrics['survival_time']:.4f}", 
                        f"{metrics['total_distance']:.4f}", 
                        f"{cot_val:.4f}",
                        f"{speed:.4f}"
                    ])
                    
                except Exception as e:
                    print(f"\n Error on L={leg_length}, f={freq}: {e}")
                    # Log failure as zeros
                    writer.writerow([leg_length, freq, 0, 0, 999, 0])

    print(f"\n\n Grid Search Complete. Data saved to data/grid_results_day20.csv")

if __name__ == "__main__":
    run_grid_search()