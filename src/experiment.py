import pandas as pd
import random
from pathlib import Path

LOG_PATH = Path("results/logss.csv")

experiment_name = "flat_ground_dummy_v1"

designs = [
    {"design_id": 0, "leg_length": 0.10, "body_clearance": 0.04},
    {"design_id": 1, "leg_length": 0.18, "body_clearance": 0.06},
    {"design_id": 2, "leg_length": 0.22, "body_clearance": 0.08},
]

rows = []

for seed, design in enumerate(designs):
    random.seed(seed)

    forward_distance = random.uniform(0.0, 2.0)
    success = forward_distance > 0.5

    row = {
        "experiment_name": experiment_name,
        "design_id": design["design_id"],
        "leg_length": design["leg_length"],
        "body_clearance": design["body_clearance"],
        "forward_distance": forward_distance,
        "success": success,
        "seed": seed,
    }

    rows.append(row)

df = pd.DataFrame(rows)

LOG_PATH.parent.mkdir(parents=True, exist_ok=True)

if LOG_PATH.exists():
    df.to_csv(LOG_PATH, mode="a", header=False, index=False)
else:
    df.to_csv(LOG_PATH, index=False)
