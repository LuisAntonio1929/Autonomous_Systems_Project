import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1.inset_locator import inset_axes, mark_inset
from matplotlib.patches import Ellipse


# ============================================================
# CARGAR CSV
# ============================================================

csv_path = "results/noise.csv"
data = pd.read_csv(csv_path)


# ============================================================
# FUNCIÓN PARA DIBUJAR ELIPSE DE COVARIANZA
# ============================================================

def add_covariance_ellipse(
    ax,
    mean_x,
    mean_y,
    cov_xx,
    cov_xy,
    cov_yy,
    n_std=2.0,
    alpha=0.18
):

    # Matriz de covarianza
    covariance = np.array([
        [cov_xx, cov_xy],
        [cov_xy, cov_yy]
    ])

    # Autovalores y autovectores
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)

    # Ordenar de mayor a menor
    order = eigenvalues.argsort()[::-1]

    eigenvalues = eigenvalues[order]
    eigenvectors = eigenvectors[:, order]


    # Evitar problemas numéricos
    eigenvalues = np.maximum(
        eigenvalues,
        0
    )


    # ========================================================
    # ORIENTACIÓN DE LA ELIPSE
    # ========================================================

    principal_vector = eigenvectors[:, 0]

    angle = np.degrees(
        np.arctan2(
            principal_vector[1],
            principal_vector[0]
        )
    )


    # ========================================================
    # TAMAÑO DE LA ELIPSE
    # ========================================================

    width = (
        2
        * n_std
        * np.sqrt(eigenvalues[0])
    )

    height = (
        2
        * n_std
        * np.sqrt(eigenvalues[1])
    )


    # ========================================================
    # CREAR ELIPSE
    # ========================================================

    ellipse = Ellipse(
        xy=(mean_x, mean_y),
        width=width,
        height=height,
        angle=angle,
        fill=True,
        alpha=alpha,
        zorder=1
    )

    ax.add_patch(ellipse)


# ============================================================
# ELEGIR EL TRAMO QUE QUIERES AMPLIAR
# ============================================================

start_idx = 10
end_idx = 16

segment = data.iloc[
    start_idx:end_idx + 1
]


# ============================================================
# CALCULAR LÍMITES DEL ZOOM
# ============================================================

x_all = np.concatenate([
    segment["true_x_m"].values,
    segment["odom_x_m"].values,
    segment["pf_mean_x_m"].values,
    segment["pf_best_x_m"].values
])

y_all = np.concatenate([
    segment["true_y_m"].values,
    segment["odom_y_m"].values,
    segment["pf_mean_y_m"].values,
    segment["pf_best_y_m"].values
])

x_margin = 0.35
y_margin = 0.035

x1 = x_all.min() - x_margin
x2 = x_all.max() + x_margin

y1 = y_all.min() - y_margin
y2 = y_all.max() + y_margin


# ============================================================
# FIGURA PRINCIPAL
# ============================================================

fig, ax = plt.subplots(
    figsize=(12, 8)
)


# ============================================================
# ELIPSES DE INCERTIDUMBRE
# ============================================================

# Cuántos puntos saltamos entre cada elipse
ellipse_skip = 6

for i in range(
    0,
    len(data),
    ellipse_skip
):

    add_covariance_ellipse(
        ax=ax,

        mean_x=data.loc[
            i,
            "pf_mean_x_m"
        ],

        mean_y=data.loc[
            i,
            "pf_mean_y_m"
        ],

        cov_xx=data.loc[
            i,
            "pf_cov_xx_m2"
        ],

        cov_xy=data.loc[
            i,
            "pf_cov_xy_m2"
        ],

        cov_yy=data.loc[
            i,
            "pf_cov_yy_m2"
        ],

        n_std=2.0,
        alpha=0.18
    )


# ============================================================
# TRAYECTORIAS PRINCIPALES
# ============================================================

ax.plot(
    data["true_x_m"],
    data["true_y_m"],
    linewidth=3,
    label="Ground truth",
    zorder=3
)

ax.plot(
    data["odom_x_m"],
    data["odom_y_m"],
    linestyle="--",
    linewidth=2.2,
    label="Odometry",
    zorder=3
)

ax.plot(
    data["pf_mean_x_m"],
    data["pf_mean_y_m"],
    linewidth=2.4,
    label="PF mean",
    zorder=4
)

ax.plot(
    data["pf_best_x_m"],
    data["pf_best_y_m"],
    linestyle=":",
    linewidth=2.6,
    label="Best particle",
    zorder=4
)


# ============================================================
# MARCADORES DE INICIO Y FIN
# ============================================================

ax.scatter(
    data["true_x_m"].iloc[0],
    data["true_y_m"].iloc[0],
    s=90,
    marker="o",
    zorder=5,
    label="Start"
)

ax.scatter(
    data["true_x_m"].iloc[-1],
    data["true_y_m"].iloc[-1],
    s=110,
    marker="X",
    zorder=5,
    label="End"
)


# ============================================================
# FORMATO DEL GRÁFICO PRINCIPAL
# ============================================================

ax.set_xlabel(
    "X position [m]",
    fontsize=13
)

ax.set_ylabel(
    "Y position [m]",
    fontsize=13
)

ax.set_title(
    "Particle Filter Localization: trajectory and uncertainty",
    fontsize=16
)

ax.axis("auto")

ax.grid(
    True,
    alpha=0.3
)


# ============================================================
# VENTANA DE ZOOM
# ============================================================

axins = inset_axes(
    ax,
    width="36%",
    height="36%",
    bbox_to_anchor=(
        0.08,
        0.0,
        1,
        1
    ),
    bbox_transform=ax.transAxes,
    loc="upper left",
    borderpad=1.6
)


# ============================================================
# ELIPSES DENTRO DEL ZOOM
# ============================================================

for i in range(
    start_idx,
    end_idx + 1
):

    add_covariance_ellipse(
        ax=axins,

        mean_x=data.loc[
            i,
            "pf_mean_x_m"
        ],

        mean_y=data.loc[
            i,
            "pf_mean_y_m"
        ],

        cov_xx=data.loc[
            i,
            "pf_cov_xx_m2"
        ],

        cov_xy=data.loc[
            i,
            "pf_cov_xy_m2"
        ],

        cov_yy=data.loc[
            i,
            "pf_cov_yy_m2"
        ],

        n_std=2.0,
        alpha=0.12
    )


# ============================================================
# TRAYECTORIAS EN EL ZOOM
# ============================================================

axins.plot(
    data["true_x_m"],
    data["true_y_m"],
    linewidth=2.2,
    zorder=3
)

axins.plot(
    data["odom_x_m"],
    data["odom_y_m"],
    linestyle="--",
    linewidth=1.7,
    zorder=3
)

axins.plot(
    data["pf_mean_x_m"],
    data["pf_mean_y_m"],
    linewidth=1.9,
    zorder=4
)

axins.plot(
    data["pf_best_x_m"],
    data["pf_best_y_m"],
    linestyle=":",
    linewidth=2.2,
    zorder=4
)


# ============================================================
# LÍMITES DEL ZOOM
# ============================================================

axins.set_xlim(
    x1,
    x2
)

axins.set_ylim(
    y1,
    y2
)


# ============================================================
# FORMATO DEL ZOOM
# ============================================================

axins.set_title(
    "Zoom window",
    fontsize=10,
    pad=8
)

axins.set_xlabel(
    "X position [m]",
    fontsize=9
)

axins.set_ylabel(
    "Y position [m]",
    fontsize=9
)

axins.grid(
    True,
    alpha=0.3
)

axins.tick_params(
    labelsize=9
)


# ============================================================
# RECTÁNGULO Y LÍNEAS DE CONEXIÓN
# ============================================================

mark_inset(
    ax,
    axins,
    loc1=2,
    loc2=4,
    fc="none",
    ec="0.35",
    lw=1.2
)


# ============================================================
# LEYENDA
# ============================================================

ax.legend(
    loc="upper right",
    fontsize=11,
    frameon=True
)


plt.tight_layout()
plt.show()