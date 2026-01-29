import experiment_runner
from controllers import SineWaveController
import os

# CONFIGURATION
# We pick the "Medium" robot from our factory
ROBOT_FILE = "../generated/robot_L0.20.xml"

def test_integration():
    print("🔌 Day 17: System Integration Test")
    print("=" * 40)
    
    # 1. Check if Robot Exists
    if not os.path.exists(ROBOT_FILE):
        print(f"❌ Error: Robot file not found: {ROBOT_FILE}")
        print("   Did you run make_robot.py in Day 15?")
        return

    # 2. Setup The Brain
    # Frequency 1.0 Hz, Amplitude 0.5 rad
    my_brain = SineWaveController(amplitude=0.5, frequency=1.0)
    print("🧠 Brain: SineWaveController (A=0.5, f=1.0)")

    # 3. Run The Experiment
    print(f"🤖 Robot: {ROBOT_FILE}")
    print("🚀 Running simulation...")
    
    result = experiment_runner.run_episode(
        model_path=ROBOT_FILE,
        duration=5.0,       # Give it 5 seconds to try walking
        controller=my_brain
    )
    
    # 4. Analyze Results
    dist = result['metrics']['total_distance']
    time = result['metrics']['survival_time']
    reason = result['status']['reason']
    
    print("-" * 40)
    print(f"📊 Result: {result['status']['state']}")
    print(f"📝 Reason: {reason}")
    print(f"⏱️  Time Survived: {time:.3f}s")
    print(f"📏 Distance Moved: {dist:.3f}m")
    
    # 5. Success Criteria
    # It counts as "Moving" if it moved more than 5cm (0.05m)
    if abs(dist) > 0.05:
        print("✅ SUCCESS: Robot moved successfully!")
    else:
        print("⚠️  WARNING: Robot barely moved. Check actuators/friction.")

if __name__ == "__main__":
    test_integration()