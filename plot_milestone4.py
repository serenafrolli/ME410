import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

# ── config ────────────────────────────────────────────────────────────────────
CSV_FILE = os.path.expanduser("~/VSCode/ME410/0004_yaw.csv")
SCALE    = 1                  # pitch * 10 per slides

# ── load ──────────────────────────────────────────────────────────────────────
if not os.path.exists(CSV_FILE):
    print(f"ERROR: '{CSV_FILE}' not found.\n"
          f"Copy it from the Pi to ~/Desktop/, then re-run.")
    sys.exit(1)

cols = ["pitch_t", "pitch_desired"]
#cols = ["pitch_t", "acc_pitch"]

df = pd.read_csv(CSV_FILE, header=None, names=cols)
df = df.dropna()

# ── derived columns ───────────────────────────────────────────────────────────
df["pitch_x10"] = df["pitch_t"] * SCALE
df["sample"]    = range(len(df))

# ── plot ──────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(14, 6))

ax.plot(df["sample"], df["motor1"],        label="Motor 1",              linewidth=1.2)
ax.plot(df["sample"], df["motor2"],        label="Motor 2",              linewidth=1.2)
ax.plot(df["sample"], df["motor3"],        label="Motor 3",              linewidth=1.2)
ax.plot(df["sample"], df["motor4"],        label="Motor 4",              linewidth=1.2)
# ax.plot(df["sample"], df["thrust"],        label="Thrust",               linewidth=2, linestyle="--")
#ax.plot(df["sample"], df["pitch_x10"], label="Pitch × 10 (comp.)",  linewidth=2)
#ax.plot(df["sample"], df["pitch_desired"], label="Pitch desired",  linewidth=2)
#ax.plot(df["sample"], df["pitch_desired"], label="Pitch desired", linewidth=2, linestyle="-.")
#ax.plot(df["sample"], df["acc_pitch"], label="acc_pitch", linewidth=2, linestyle="-.")
ax.plot(df["sample"], df["yaw_desired"], label="yaw desired",  linewidth=2)
ax.plot(df["sample"], df["pitch_desired"], label="Pitch desired",  linewidth=2)


ax.set_xlabel("Sample", fontsize=12)
ax.set_ylabel("Motor speed / Angle (deg×10) / Integrated pitch", fontsize=12)
ax.set_title("Milestone 1 W5: PID Control",
             fontsize=13)
ax.legend(fontsize=10)
ax.grid(True, alpha=0.35)

plt.tight_layout()
plt.show()

