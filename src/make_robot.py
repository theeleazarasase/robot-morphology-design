import os

# CONFIGURATION
TEMPLATE_PATH = "../assets/robot_template.xml"
OUTPUT_DIR = "../generated"
LEG_LENGTHS = [0.10, 0.15, 0.20, 0.25, 0.30] # The Sweep
FOOT_HEIGHT = 0.01 # Half-thickness of the foot box
DROP_BUFFER = 0.05 # Drop from 5cm above ground to prove stability

def generate_robots():
    print(f"🏭 Factory: Initializing...")
    
    # 1. Read Template
    if not os.path.exists(TEMPLATE_PATH):
        print(f"❌ Error: Template not found at {TEMPLATE_PATH}")
        return
        
    with open(TEMPLATE_PATH, 'r') as f:
        template_str = f.read()
    
    # 2. Ensure Output Directory
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
        print(f"📁 Created directory: {OUTPUT_DIR}")

    # 3. The Generation Loop
    for L in LEG_LENGTHS:
        # Calculate Start Z to prevent floor clipping
        # Z = Leg Length + Foot Thickness + Buffer
        start_z = L + FOOT_HEIGHT + DROP_BUFFER
        
        # Create the filename
        # We use .2f to ensure "0.10" not "0.1" for sorting consistency
        filename = f"robot_L{L:.2f}.xml"
        filepath = os.path.join(OUTPUT_DIR, filename)
        
        # Fill the Template
        # We replace {LEG_LENGTH} and {START_Z}
        xml_content = template_str.format(
            LEG_LENGTH=L,
            START_Z=f"{start_z:.3f}"
        )
        
        # Save File
        with open(filepath, 'w') as f:
            f.write(xml_content)
            
        print(f"✅ Generated: {filename} (L={L:.2f}m, StartZ={start_z:.3f}m)")

if __name__ == "__main__":
    generate_robots()