import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


# ============================================================
# LOAD DATA
# ============================================================

data = pd.read_csv("results/noise.csv")


# ============================================================
# UNWRAP ORIENTATION
# ============================================================

true_theta = np.unwrap(data["true_theta_rad"])
odom_theta = np.unwrap(data["odom_theta_rad"])
pf_mean_theta = np.unwrap(data["pf_mean_theta_rad"])
pf_best_theta = np.unwrap(data["pf_best_theta_rad"])


# ============================================================
# PLOT
# ============================================================

fig, ax = plt.subplots(figsize=(11, 5))


ax.plot(
    data["time_s"],
    true_theta,
    linewidth=3,
    label="Ground truth"
)

ax.plot(
    data["time_s"],
    odom_theta,
    linestyle="--",
    linewidth=2.2,
    label="Odometry"
)

ax.plot(
    data["time_s"],
    pf_mean_theta,
    linewidth=2.4,
    label="PF mean"
)

ax.plot(
    data["time_s"],
    pf_best_theta,
    linestyle=":",
    linewidth=2.6,
    label="Best particle"
)


# ============================================================
# FORMAT
# ============================================================

ax.set_xlabel("Time [s]", fontsize=13)
ax.set_ylabel("Orientation θ [rad]", fontsize=13)

ax.set_title(
    "Robot orientation estimation",
    fontsize=16
)

ax.grid(True, alpha=0.3)

ax.legend(
    fontsize=11
)

plt.tight_layout()
plt.show()