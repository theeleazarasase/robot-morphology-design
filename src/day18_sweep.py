import os
import glob
import re
import numpy as np
from experiment_runner import run_episode
from controllers import SineWaveController

def run_sweep():
    print("Day 18: Morphological Sweep (Leg Length vs Stability)")
    print("=" * 60)

    # 1. SETUP & DISCOVERY
    # Use dirname to ensure we find the folder regardless of where we run the script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    generated_dir = os.path.join(script_dir, "../generated")
    search_pattern = os.path.join(generated_dir, "robot_L*.xml")
    
    xml_files = glob.glob(search_pattern)
    morphologies = []

    # Regex to extract "0.10" from "robot_L0.10.xml"
    regex_pattern = r'robot_L(\d+\.?\d*)'

    for file_path in xml_files:
        filename = os.path.basename(file_path)
        match = re.search(regex_pattern, filename)
        if match:
            leg_length = float(match.group(1))
            morphologies.append((leg_length, file_path))

    # Sort by leg length (Small -> Large)
    morphologies.sort(key=lambda x: x[0])

    if not morphologies:
        print(f" Error: No robot files found in {generated_dir}")
        print("   Did you run make_robot.py?")
        return

    # 2. EXPERIMENT LOOP
    results = []
    # Standardize the Brain: Everyone gets the exact same signal
    brain = SineWaveController(amplitude=0.5, frequency=1.0)
    
    print(f"found {len(morphologies)} robots. Starting sweep...")

    for (leg_length, xml_path) in morphologies:
        print(f"   Testing L={leg_length:.2f}m...", end=" ", flush=True)
        
        try:
            # Run the episode
            experiment_data = run_episode(
                model_path=xml_path,
                controller=brain,
                duration=5.0
            )
            
            # Extract Metrics
            survival = experiment_data['metrics']['survival_time']
            dist = experiment_data['metrics']['total_distance']
            
            # Store Result
            trial_result = {
                'L': leg_length,
                'Time': survival,
                'Dist': dist
            }
            results.append(trial_result)
            
            print(f"Done ({survival:.2f}s)")
            
        except Exception as e:
            print(f"FAILED: {e}")
            results.append({'L': leg_length, 'Time': 0.0, 'Dist': 0.0})

    # 3. REPORTING (The Table)
    print("\n" + "="*60)
    print("EXPERIMENTAL RESULTS")
    print("="*60)
    print(f"| {'Leg Length':<10} | {'Survival (s)':<12} | {'Distance (m)':<12} |")
    print(f"|{'-'*12}|{'-'*14}|{'-'*14}|")

    for r in results:
        print(f"| {r['L']:<10.2f} | {r['Time']:<12.2f} | {r['Dist']:<12.2f} |")
    print("="*60)

    # 4. WINNER ANALYSIS
    winner = max(results, key=lambda x: x['Time'])
    print(f"\n🏆 WINNER: L={winner['L']:.2f}m")
    print(f"   Survived: {winner['Time']:.2f}s")

    # 5. HYPOTHESIS TEST
    # Split data into "Short" (first half) and "Tall" (second half)
    mid = len(results) // 2
    short_legs = results[:mid]
    tall_legs = results[mid:]
    
    avg_short = np.mean([r['Time'] for r in short_legs])
    avg_tall = np.mean([r['Time'] for r in tall_legs])

    print(f"\n Hypothesis Test:")
    print(f"   Avg Survival (Short): {avg_short:.2f}s")
    print(f"   Avg Survival (Tall):  {avg_tall:.2f}s")
    
    if avg_short > avg_tall:
        print(" CONCLUSION: H1 Supported (Short legs are more stable)")
    else:
        print(" CONCLUSION: H0 Supported (No stability advantage for short legs)")

if __name__ == "__main__":
    run_sweep()