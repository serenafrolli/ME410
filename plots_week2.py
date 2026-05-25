import pandas as pd
import matplotlib.pyplot as plt

# --- ROLL ---
df_roll = pd.read_csv("~/Desktop/roll_data.csv", header=None, names=["accel_roll", "intl_roll", "filter_roll"])
plt.figure(figsize=(12, 5))
plt.plot(df_roll["accel_roll"],  label="accel_roll",  color="blue",   linewidth=0.8)
plt.plot(df_roll["intl_roll"],   label="intl_roll",   color="red",    linewidth=1.2)
plt.plot(df_roll["filter_roll"], label="filter_roll", color="orange", linewidth=1.5)
plt.axhline(0, color='black', linewidth=0.8)
plt.title("Complementary Filter - Roll")
plt.legend()
plt.tight_layout()

plt.show()

# --- PITCH ---
df_pitch = pd.read_csv("~/Desktop/pitch_data.csv", header=None, names=["accel_pitch", "intl_pitch", "filter_pitch"])
plt.figure(figsize=(12, 5))
plt.plot(df_pitch["accel_pitch"],  label="accel_pitch",  color="blue",   linewidth=0.8)
plt.plot(df_pitch["intl_pitch"],   label="intl_pitch",   color="red",    linewidth=1.2)
plt.plot(df_pitch["filter_pitch"], label="filter_pitch", color="orange", linewidth=1.5)
plt.axhline(0, color='black', linewidth=0.8)
plt.title("Complementary Filter - Pitch")
plt.legend()
plt.tight_layout()

plt.show()