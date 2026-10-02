import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv("results/noise.csv")

fig, ax = plt.subplots(figsize=(11, 5))

ax.plot(
    data["time_s"],
    data["pf_std_x_m"],
    linewidth=2.2,
    label="Std X"
)

ax.plot(
    data["time_s"],
    data["pf_std_y_m"],
    linewidth=2.2,
    label="Std Y"
)

ax.set_xlabel("Time [s]", fontsize=13)
ax.set_ylabel("Position standard deviation [m]", fontsize=13)

ax.set_title(
    "Particle Filter position uncertainty",
    fontsize=16
)

ax.grid(True, alpha=0.3)
ax.legend(fontsize=11)

plt.tight_layout()
plt.show()


import numpy as np

theta_std_deg = np.degrees(
    data["pf_theta_std_rad"]
)

fig, ax = plt.subplots(figsize=(11, 5))

ax.plot(
    data["time_s"],
    theta_std_deg,
    linewidth=2.2
)

ax.set_xlabel("Time [s]", fontsize=13)
ax.set_ylabel("Orientation standard deviation [°]", fontsize=13)

ax.set_title(
    "Particle Filter orientation uncertainty",
    fontsize=16
)

ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

fig, ax = plt.subplots(figsize=(11, 5))

# Effective Sample Size
ax.plot(
    data["time_s"],
    data["n_eff"],
    linewidth=2.2,
    label="Effective sample size"
)


# ============================================================
# RESAMPLING EVENTS
# ============================================================

resampled = data[data["resampled"] == True]

ax.scatter(
    resampled["time_s"],
    resampled["n_eff"],
    marker="X",
    s=100,
    zorder=5,
    label="Resampling"
)


ax.set_xlabel("Time [s]", fontsize=13)
ax.set_ylabel("Effective particles", fontsize=13)

ax.set_title(
    "Effective Sample Size and resampling events",
    fontsize=16
)

ax.grid(True, alpha=0.3)
ax.legend(fontsize=11)

plt.tight_layout()
plt.show()

fig, ax = plt.subplots(figsize=(11, 5))

ax.plot(
    data["time_s"],
    data["pf_best_weight"],
    linewidth=2.2
)

ax.set_xlabel("Time [s]", fontsize=13)
ax.set_ylabel("Best particle weight", fontsize=13)

ax.set_title(
    "Maximum particle weight",
    fontsize=16
)

ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()