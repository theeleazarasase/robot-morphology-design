import os
import glob
import re
import numpy as np
from experiment_runner import run_episode
from controllers import SineWaveController

def run_efficiency_sweep():
    print("Day 19: Energy Efficiency Sweep (Cost of Transport)")
    print("=" * 80)

    # 1. SETUP & DISCOVERY
    script_dir = os.path.dirname(os.path.abspath(__file__))
    generated_dir = os.path.join(script_dir, "../generated")
    search_pattern = os.path.join(generated_dir, "robot_L*.xml")
    
    xml_files = glob.glob(search_pattern)
    morphologies = []

    # Regex to extract leg length
    regex_pattern = r'robot_L(\d+\.?\d*)'

    for file_path in xml_files:
        filename = os.path.basename(file_path)
        match = re.search(regex_pattern, filename)
        if match:
            leg_length = float(match.group(1))
            morphologies.append((leg_length, file_path))

    # Sort Small -> Large
    morphologies.sort(key=lambda x: x[0])

    if not morphologies:
        print(f" Error: No robot files found in {generated_dir}")
        return

    # 2. EXPERIMENT LOOP
    results = []
    brain = SineWaveController(amplitude=0.5, frequency=1.0)
    
    print(f"Found {len(morphologies)} robots. Measuring Energy...")

    for (leg_length, xml_path) in morphologies:
        try:
            # Run the episode
            experiment_data = run_episode(
                model_path=xml_path,
                controller=brain,
                duration=5.0
            )
            
            # Extract Metrics
            metrics = experiment_data['metrics']
            survival = metrics['survival_time']
            dist = metrics['total_distance']
            energy = metrics['total_energy']
            cot = metrics['cot']
            
            # Store Result
            trial_result = {
                'L': leg_length,
                'Time': survival,
                'Dist': dist,
                'Energy': energy,
                'CoT': cot
            }
            results.append(trial_result)
            
            # Live Print (Showing CoT)
            cot_display = f"{cot:.2f}" if cot != float('inf') else "inf"
            print(f"   L={leg_length:.2f}m -> Time: {survival:.2f}s | Dist: {dist:.2f}m | CoT: {cot_display}")
            
        except Exception as e:
            print(f"   L={leg_length:.2f}m -> FAILED: {e}")
            results.append({'L': leg_length, 'Time': 0.0, 'Dist': 0.0, 'Energy': 0.0, 'CoT': float('inf')})

    # 3. REPORTING
    print("\n" + "="*85)
    print("EXPERIMENTAL RESULTS: EFFICIENCY (Lower CoT is Better)")
    print("="*85)
    
    print(f"| {'Length':<8} | {'Survival':<10} | {'Distance':<10} | {'Energy (J)':<12} | {'CoT':<10} |")
    print(f"|{'-'*10}|{'-'*12}|{'-'*12}|{'-'*14}|{'-'*12}|")

    for r in results:
        cot_str = f"{r['CoT']:.2f}" if r['CoT'] != float('inf') else "inf"
        print(f"| {r['L']:<8.2f} | {r['Time']:<10.2f} | {r['Dist']:<10.2f} | {r['Energy']:<12.2f} | {cot_str:<10} |")
    print("="*85)

    # 4. WINNER ANALYSIS
    # Filter for robots that actually moved (> 10cm)
    moving_robots = [r for r in results if abs(r['Dist']) > 0.10]
    
    print(f"\n ANALYSIS:")
    
    if moving_robots:
        # Sort by CoT (Lowest is best)
        best_efficiency = min(moving_robots, key=lambda x: x['CoT'])
        print(f"   Most Efficient: L={best_efficiency['L']:.2f}m (CoT = {best_efficiency['CoT']:.2f})")
        print("   (Note: This robot used the least energy per meter traveled)")
    else:
        print("   Most Efficient: N/A (No robot walked significantly)")

if __name__ == "__main__":
    run_efficiency_sweep()