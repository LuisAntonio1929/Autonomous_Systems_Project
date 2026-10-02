import pandas as pd
import matplotlib.pyplot as plt


# ============================================================
# LOAD CSV FILES
# ============================================================

no_noise_path = (
    "results/"
    "no_noise.csv"
)

noise_path = (
    "results/"
    "noise.csv"
)


no_noise_data = pd.read_csv(
    no_noise_path
)

noise_data = pd.read_csv(
    noise_path
)


# ============================================================
# EXTRACT GROUND-TRUTH TRAJECTORIES
# ============================================================

# ------------------------------------------------------------
# No motor noise
# ------------------------------------------------------------

no_noise_x = no_noise_data[
    "true_x_m"
]

no_noise_y = no_noise_data[
    "true_y_m"
]


# ------------------------------------------------------------
# With motor noise
# ------------------------------------------------------------

noise_x = noise_data[
    "true_x_m"
]

noise_y = noise_data[
    "true_y_m"
]


# ============================================================
# CREATE FIGURE
# ============================================================

fig, ax = plt.subplots(
    figsize=(11, 8)
)


# ============================================================
# PLOT TRAJECTORIES
# ============================================================

# ------------------------------------------------------------
# Ground truth without motor noise
# ------------------------------------------------------------

ax.plot(
    no_noise_x,
    no_noise_y,
    linewidth=2.5,
    label="Ground truth - No motor noise"
)


# ------------------------------------------------------------
# Ground truth with motor noise
# ------------------------------------------------------------

ax.plot(
    noise_x,
    noise_y,
    linewidth=2.5,
    linestyle="--",
    label="Ground truth - Motor noise"
)


# ============================================================
# STARTING POSITIONS
# ============================================================

ax.scatter(
    no_noise_x.iloc[0],
    no_noise_y.iloc[0],
    s=90,
    marker="o",
    zorder=4
)

ax.scatter(
    noise_x.iloc[0],
    noise_y.iloc[0],
    s=90,
    marker="o",
    zorder=4
)


# ============================================================
# FINAL POSITIONS
# ============================================================

ax.scatter(
    no_noise_x.iloc[-1],
    no_noise_y.iloc[-1],
    s=110,
    marker="X",
    zorder=4
)

ax.scatter(
    noise_x.iloc[-1],
    noise_y.iloc[-1],
    s=110,
    marker="X",
    zorder=4
)


# ============================================================
# ANNOTATIONS
# ============================================================

ax.annotate(
    "Start",
    (
        no_noise_x.iloc[0],
        no_noise_y.iloc[0]
    ),
    xytext=(8, 8),
    textcoords="offset points"
)

ax.annotate(
    "End - No noise",
    (
        no_noise_x.iloc[-1],
        no_noise_y.iloc[-1]
    ),
    xytext=(8, 8),
    textcoords="offset points"
)

ax.annotate(
    "End - Noise",
    (
        noise_x.iloc[-1],
        noise_y.iloc[-1]
    ),
    xytext=(8, -15),
    textcoords="offset points"
)


# ============================================================
# GRAPH CONFIGURATION
# ============================================================

ax.set_title(
    "Ground-Truth Robot Trajectory: "
    "No Motor Noise vs Motor Noise"
)

ax.set_xlabel(
    "X position [m]"
)

ax.set_ylabel(
    "Y position [m]"
)


# Preserve real spatial proportions.
ax.set_aspect(
    "equal",
    adjustable="box"
)


ax.grid(
    True,
    alpha=0.3
)

ax.legend()

fig.tight_layout()


# ============================================================
# SAVE FIGURE
# ============================================================

output_path = (
    "results/"
    "ground_truth_noise_comparison.png"
)

fig.savefig(
    output_path,
    dpi=300,
    bbox_inches="tight"
)


# ============================================================
# SHOW FIGURE
# ============================================================

plt.show()


print(
    f"Figure saved to:\n"
    f"{output_path}"
)