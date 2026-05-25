import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

# ── config ────────────────────────────────────────────────────────────────────
CSV_FILE = os.path.expanduser("~/VSCode/ME410/0008.csv")
SCALE    = 10                      # pitch * 10 per slides

# ── load ──────────────────────────────────────────────────────────────────────
if not os.path.exists(CSV_FILE):
    print(f"ERROR: '{CSV_FILE}' not found.\n"
          f"Copy it from the Pi to ~/Desktop/, then re-run.")
    sys.exit(1)

cols = ["temp[7]", "pitch_gyro_delta", "pitch_desired", "pitch_t"]

df = pd.read_csv(CSV_FILE, header=None, names=cols)
df = df.dropna()

# ── derived columns ───────────────────────────────────────────────────────────
df["pitch_x10"] = df["pitch_t"] * SCALE
df["sample"]    = range(len(df))

# ── plot ──────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(14, 6))

ax.plot(df["sample"], df["temp[7]"],        label="acc angle ",              linewidth=1.2)
ax.plot(df["sample"], df["pitch_gyro_delta"],        label="gyro angle",              linewidth=1.2)
ax.plot(df["sample"], df["pitch_desired"],        label="pitch_d",              linewidth=1.2)
ax.plot(df["sample"], df["pitch_t"],        label="pitch_filter",              linewidth=1.2)
#ax.plot(df["sample"], df["thrust"],        label="Thrust",               linewidth=2, linestyle="--")
#ax.plot(df["sample"], df["pitch_x10"],     label="Pitch × 10 (comp.)",  linewidth=2)
#ax.plot(df["sample"], df["pitch_desired"], label="Pitch Velocity (intl_pitch)", linewidth=2, linestyle="-.")

ax.set_xlabel("Sample", fontsize=12)
ax.set_ylabel("Motor speed / Angle (deg×10) / Integrated pitch", fontsize=12)
ax.set_title("Problem",
             fontsize=13)
ax.legend(fontsize=10)
ax.grid(True, alpha=0.35)

plt.tight_layout()
plt.savefig(os.path.expanduser("~/Desktop/0008.png"), dpi=150)
print("Saved")
plt.show()

