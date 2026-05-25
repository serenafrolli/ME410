import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

# ── config ────────────────────────────────────────────────────────────────────
MODE = "pitch"        # "pitch", "roll", or "yaw"
SHOW_MOTORS = True  # only works for yaw (or if you add motors to pitch/roll CSVs)

# CSV layouts — must match the fprintf in main.cpp
CONFIGS = {
    "pitch": {
        "file":  "~/VSCode/ME410/0007.csv",
        "cols":  ["actual", "desired"],
        "label": "Pitch",
        "has_motors": False,
    },
    "roll": {
        "file":  "~/VSCode/ME410/0004_roll.csv",
        "cols":  ["actual", "desired"],
        "label": "Roll",
        "has_motors": False,
    },
    "yaw": {
        "file":  "~/VSCode/ME410/0004_yaw copia.csv",
        "cols":  ["actual", "desired", "motor1", "motor2", "motor3", "motor4"],
        "label": "Yaw rate",
        "has_motors": True,
    },
}

cfg = CONFIGS[MODE]
CSV_FILE = os.path.expanduser(cfg["file"])

# ── load ──────────────────────────────────────────────────────────────────────
if not os.path.exists(CSV_FILE):
    print(f"ERROR: '{CSV_FILE}' not found.\n"
          f"Copy it from the Pi, then re-run.")
    sys.exit(1)

df = pd.read_csv(CSV_FILE, header=None, names=cfg["cols"])
df = df.dropna()
df["sample"] = range(len(df))

# ── plot ──────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(14, 6))

ax.plot(df["sample"], df["actual"],  label=f"{cfg['label']} actual",  linewidth=2)
ax.plot(df["sample"], df["desired"], label=f"{cfg['label']} desired", linewidth=2, linestyle="-.")

if SHOW_MOTORS and cfg["has_motors"]:
    for i in range(1, 5):
        ax.plot(df["sample"], df[f"motor{i}"], label=f"Motor {i}", linewidth=1.0, alpha=0.7)

ax.set_xlabel("Sample", fontsize=12)
ax.set_ylabel(f"{cfg['label']} / Motor speed", fontsize=12)
ax.set_title(f"W5 Milestone — {cfg['label']} PID tracking", fontsize=13)
ax.legend(fontsize=10)
ax.grid(True, alpha=0.35)

plt.tight_layout()
plt.show()

