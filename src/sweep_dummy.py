import itertools
import random
import pandas as pd
from pathlib import Path
from datetime import datetime

log_path = Path("results/logs.csv")
experiment_name = "flat_ground_param_sweep"
success_threshold = 0.5

#parameter sweep definition
leg_lengths = [0.08, 0.12, 0.16, 0.20]
body_clearances = [0.03, 0.05, 0.07]
n_seeds = 5

rows = []
design_id = 0

for leg_length, body_clearance in itertools.product(leg_lengths,body_clearances):
    for seed in range(n_seeds):
        random.seed(seed)

        #dummy eval
        forward_dist = random.uniform(0.0, 2.0)
        sucess = forward_dist > success_threshold

        row = {            
            "timestamp": datetime.utcnow().isoformat(),
            "experiment_name": EXPERIMENT_NAME,
            "design_id": design_id,
            "leg_length_m": leg_length,
            "body_clearance_m": body_clearance,
            "metric_forward_distance_m": forward_distance,
            "success": success,
            "success_threshold_m": SUCCESS_THRESHOLD,
            "seed": seed,
    
        }
        rows.append(row)
    design_id +=1
df = pd.DataFrame(rows)
log_path.parent.mkdire(parents=True, exist_ok =True)

if log_path.exists():
    df.to_csv(log_path, mode="a", header=False, index =False)
else:
    df.to_csv(log_path, index = False)

print(f"Logged{len(rows)} experiment rows")