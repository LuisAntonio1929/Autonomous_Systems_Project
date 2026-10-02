import math
import numpy as np
import pybullet as p
import utils.world as W
import utils.demo_robot as R
import utils.demo_person as P
import utils.demo_odometry as O
import utils.ScannableMap as S
import utils.particle_filter as PF
import csv
import os
from datetime import datetime
import random

RANDOM_SEED = 42

random.seed(
    RANDOM_SEED
)

np.random.seed(
    RANDOM_SEED
)

# create the world object, which will set up most of pybullet
wld = W.BaseWorld()

# Disable rendering while the environment is being constructed.
# This significantly speeds up the creation of maps with many objects.
p.configureDebugVisualizer(
    p.COV_ENABLE_RENDERING,
    0
)

# load a map
wld.map_from_file("maps/Lab3Map")
# ============================================================
# SCANNABLE MAP
# ============================================================

# The map uses an internal scale of 10:
#
#   1 PyBullet meter = 10 ScanableMap units
#
# Robot LiDAR configuration:
#   61 rays
#   ±60 degrees -> 120 degree total FOV
#   10 meter maximum range

scan_map = S.ScanableMap(
    map_dimensions=(22, 31),
    fov=120,
    max_distance=100,
    num_rays=61,
    scale=10
)

scan_map.load_map(
    "maps/Lab3Map"
)

# ============================================================
# PARTICLE FILTER
# ============================================================

particle_filter = PF.ParticleFilter(
    scan_map=scan_map,
    num_particles=500
)

particle_filter.initialize_gaussian(
    x=3.0,
    y=5.0,
    theta=0.0,
    position_std=0.40,
    angle_std=math.radians(10.0)
)

# you do not have to do this in this way, but I created a method to add textures to the walls
# be warned it interacts with the way I did the map and the storage of the map, so it only
# impacts walls of type 1
wld.texture_walls()


# ------------------------------------------------------------
# Create the robot
# ------------------------------------------------------------

# This position works correctly with the new Lab 3 map.
r = R.DemoRobot(x=3, y=5)
r.set_control_mode("autonomous")
# make the camera follow the robot
wld.follow_camera(r.agv_id)

# ============================================================
# ODOMETRY
# ============================================================

# Differential-drive odometry based on the rotation
# of the two powered wheels.
odom = O.DifferentialDriveOdometry(
    wheel_radius=0.15,
    wheel_base=0.90,
    left_wheel_joints=[0],
    right_wheel_joints=[1]
)

# The robot starts from a known position.
odom.reset(
    x=3.0,
    y=5.0,
    theta=0.0
)

INCLUDE_PEOPLE = True
PRINT_DATA = False

if INCLUDE_PEOPLE:
    # ============================================================
    # PERSON 1
    # ============================================================

    person1 = P.DemoPerson()

    # ------------------------------------------------------------
    # Person 1 movement graph
    # ------------------------------------------------------------
    # Person 1 moves vertically.
    #
    #       (9,1)
    #         |
    #         |
    #         |
    # ------ (9,5) ------ Robot path
    #         |
    #         |
    #       (9,8)
    #
    # The movement is a simple straight back-and-forth motion.

    person1_path = P.ValidPathGraph().add_positions([
        (9.5, 2),
        (9.5, 8)
    ])

    person1_path.add_connection(
        (9.5, 2),
        (9.5, 8)
    )

    person1.set_movement_graph(
        person1_path
    )

    # Set a known initial position so the simulation is repeatable.
    person1.set_position_on_path(
        start=(9.5, 2),
        end=(9.5, 8),
        progress=0.54,
        toward_end=True
    )

    person1.set_yaw(90)

    # ============================================================
    # PERSON 2
    # ============================================================

    person2 = P.DemoPerson()

    # ------------------------------------------------------------
    # Person 2 movement graph
    # ------------------------------------------------------------
    # Person 2 moves horizontally.
    #
    # (22,8) -------- (27,8) -------- (29,8)
    #                     |
    #                     |
    #                  Robot path
    #
    # The robot will later travel vertically through this crossing.

    person2_path = P.ValidPathGraph().add_positions([
        (22, 8.5),
        (29, 8.5)
    ])

    person2_path.add_connection(
        (22, 8.5),
        (29, 8.5)
    )

    person2.set_movement_graph(
        person2_path
    )

    # Set a known initial position.
    person2.set_position_on_path(
        start=(22, 8.5),
        end=(29, 8.5),
        progress=0.78,
        toward_end=True
    )


    # ============================================================
    # PERSON 3
    # ============================================================

    person3 = P.DemoPerson()

    # ------------------------------------------------------------
    # Person 3 movement graph
    # ------------------------------------------------------------
    # Person 3 moves vertically.
    #
    #       (20,13)
    #          |
    #          |
    # ------ (20,16) ------ Robot path
    #          |
    #          |
    #       (20,20)
    #
    # Again, the person only moves in a straight line.

    person3_path = P.ValidPathGraph().add_positions([
        (20.5, 14),
        (20.5, 20)
    ])

    person3_path.add_connection(
        (20.5, 14),
        (20.5, 20)
    )

    person3.set_movement_graph(
        person3_path
    )

    # Set a known initial position.
    person3.set_position_on_path(
        start=(20.5, 14),
        end=(20.5, 20),
        progress=0.18,
        toward_end=False
    )

    person3.set_yaw(90)

    # ------------------------------------------------------------
    # Robot-person physical interaction
    # ------------------------------------------------------------

    # Allow every person object to detect physical collisions
    # with the robot.
    person1.add_collidable(
        r.agv_id
    )

    person2.add_collidable(
        r.agv_id
    )

    person3.add_collidable(
        r.agv_id
    )

    # Update all people before updating the robot.
    # This means the LiDAR sees their most recent positions
    # when the robot performs its next step.
    wld.add_step_callback(
        person1.step_action
    )

    wld.add_step_callback(
        person2.step_action
    )

    wld.add_step_callback(
        person3.step_action
    )

    r.set_person_targets([
        person1.objectId,
        person2.objectId,
        person3.objectId
    ])


# ------------------------------------------------------------
# Simulation callbacks
# ------------------------------------------------------------

# Then update the robot.
wld.add_step_callback(
    r.step_action
)

# Keep the keyboard callback available for manual control/testing.
wld.add_keyboard_callback(
    r.user_control
)

# The complete environment has now been created.
# Enable rendering so everything appears at once.
p.configureDebugVisualizer(
    p.COV_ENABLE_RENDERING,
    1
)

# Counter used only for displaying information.
step_counter = 0
# Store the most recent LiDAR measurement.
#
# The LiDAR will run more frequently than the particle filter.
latest_lidar_distances = None
latest_lidar_hit_ids = None
latest_lidar_angles = None
# ============================================================
# PARTICLE FILTER ODOMETRY REFERENCE
# ============================================================
#
# Store the current odometry pose.
# This will be used as the reference pose for calculating
# the robot motion during the first particle-filter update.

previous_pf_odom = odom.get_pose()

# ============================================================
# DATA LOGGING
# ============================================================
#
# Create one CSV file for the current localisation experiment.
#
# A timestamp prevents previous experiments from being
# overwritten.

os.makedirs(
    "results",
    exist_ok=True
)

timestamp = datetime.now().strftime(
    "%Y%m%d_%H%M%S"
)

csv_path = (
    f"results/"
    f"lab4_localisation_{timestamp}.csv"
)

csv_file = open(
    csv_path,
    mode="w",
    newline="",
    encoding="utf-8"
)

csv_writer = csv.writer(
    csv_file
)


# ------------------------------------------------------------
# CSV header
# ------------------------------------------------------------

csv_writer.writerow([

    # Simulation
    "step",
    "time_s",

    # Ground truth
    "true_x_m",
    "true_y_m",
    "true_theta_rad",

    # Odometry
    "odom_x_m",
    "odom_y_m",
    "odom_theta_rad",

    # Particle-filter weighted mean
    "pf_mean_x_m",
    "pf_mean_y_m",
    "pf_mean_theta_rad",

    # Best particle
    "pf_best_x_m",
    "pf_best_y_m",
    "pf_best_theta_rad",
    "pf_best_weight",

    # Position uncertainty
    "pf_std_x_m",
    "pf_std_y_m",

    "pf_cov_xx_m2",
    "pf_cov_xy_m2",
    "pf_cov_yy_m2",

    # Orientation uncertainty
    "pf_theta_std_rad",
    "pf_theta_variance",
    "pf_resultant_length",

    # Position errors
    "odom_position_error_m",
    "pf_mean_position_error_m",
    "pf_best_position_error_m",

    # Orientation errors
    "odom_angle_error_rad",
    "pf_mean_angle_error_rad",
    "pf_best_angle_error_rad",

    # Particle-filter state
    "n_eff",
    "resampled",

    # Motion used by prediction
    "delta_forward_m",
    "delta_lateral_m",
    "delta_theta_rad"
])

csv_file.flush()

print(
    f"\nLogging localisation data to:\n"
    f"{csv_path}\n"
)
# ============================================================
# RUN SIMULATION
# ============================================================

while True:

    # ========================================================
    # EXIT WITH Q
    # ========================================================
    #
    # Press Q to finish the experiment safely.
    #
    # The loop is stopped first. The CSV file and PyBullet
    # simulation are closed after leaving the loop.

    keys = p.getKeyboardEvents()

    if (
        ord('q') in keys
        and keys[ord('q')] & p.KEY_WAS_TRIGGERED
    ):

        print(
            "\n"
            "==========================================\n"
            "Q pressed - finishing experiment...\n"
            "==========================================\n"
        )

        break


    # --------------------------------------------------------
    # Advance PyBullet simulation
    # --------------------------------------------------------

    wld.simStep()


    # --------------------------------------------------------
    # Update odometry
    # --------------------------------------------------------

    odom.update(
        r.agv_id
    )

    step_counter += 1


    # ========================================================
    # LiDAR UPDATE
    # ========================================================
    #
    # The simulation runs at approximately 240 Hz.
    #
    # 240 / 24 = 10 Hz
    #
    # The LiDAR is refreshed approximately 10 times
    # per second.
    #
    # This update is independent from the particle filter.

    if step_counter % 24 == 0:

        (
            latest_lidar_distances,
            latest_lidar_hit_ids,
            latest_lidar_angles
        ) = r.ray_cast_lidar()

        # Update LiDAR visualisation.
        r.draw_lidar_debug(
            distances=latest_lidar_distances,
            hit_ids=latest_lidar_hit_ids,
            angles=latest_lidar_angles
        )


    # ========================================================
    # PARTICLE FILTER UPDATE
    # ========================================================
    #
    # Run localisation approximately once per second.

    if step_counter % 240 == 0:

        # ====================================================
        # ODOMETRY
        # ====================================================

        (
            odom_x,
            odom_y,
            odom_theta
        ) = odom.get_pose()


        # ====================================================
        # PARTICLE FILTER MOTION PREDICTION
        # ====================================================

        (
            previous_x,
            previous_y,
            previous_theta
        ) = previous_pf_odom


        # ----------------------------------------------------
        # Odometry displacement in world coordinates
        # ----------------------------------------------------

        delta_x = (
            odom_x
            - previous_x
        )

        delta_y = (
            odom_y
            - previous_y
        )

        delta_theta = (
            odom_theta
            - previous_theta
        )


        # ----------------------------------------------------
        # Normalize angular displacement to [-pi, pi]
        # ----------------------------------------------------

        delta_theta = (
            delta_theta
            + math.pi
        ) % (
            2.0 * math.pi
        ) - math.pi


        # ----------------------------------------------------
        # Convert displacement to robot local coordinates
        # ----------------------------------------------------

        delta_forward = (
            math.cos(
                previous_theta
            )
            * delta_x

            + math.sin(
                previous_theta
            )
            * delta_y
        )

        delta_lateral = (
            -math.sin(
                previous_theta
            )
            * delta_x

            + math.cos(
                previous_theta
            )
            * delta_y
        )


        # ----------------------------------------------------
        # Predict new particle poses
        # ----------------------------------------------------

        particle_filter.predict(
            delta_forward=delta_forward,
            delta_lateral=delta_lateral,
            delta_theta=delta_theta
        )


        # ----------------------------------------------------
        # Store odometry pose for the next PF cycle
        # ----------------------------------------------------

        previous_pf_odom = (
            odom_x,
            odom_y,
            odom_theta
        )


        # ====================================================
        # GROUND TRUTH
        # ====================================================
        #
        # Ground truth is used only for validation.
        # It is not used by the particle filter.

        (
            true_position,
            true_orientation
        ) = p.getBasePositionAndOrientation(
            r.agv_id
        )

        (
            _,
            _,
            true_theta
        ) = p.getEulerFromQuaternion(
            true_orientation
        )


        # ====================================================
        # USE MOST RECENT LiDAR SCAN
        # ====================================================

        lidar_distances = (
            latest_lidar_distances
        )


        # ====================================================
        # PARTICLE FILTER SENSOR UPDATE
        # ====================================================

        particle_filter.update_weights(
            lidar_distances
        )


        # ====================================================
        # PARTICLE FILTER STATISTICS
        # ====================================================
        #
        # Calculate statistics before resampling because
        # particle weights still represent the current
        # posterior distribution.

        pf_stats = (
            particle_filter.estimate_statistics()
        )

        pf_x = (
            pf_stats["mean_x"]
        )

        pf_y = (
            pf_stats["mean_y"]
        )

        pf_theta = (
            pf_stats["mean_theta"]
        )

        best_x = (
            pf_stats["best_x"]
        )

        best_y = (
            pf_stats["best_y"]
        )

        best_theta = (
            pf_stats["best_theta"]
        )

        best_weight = (
            pf_stats["best_weight"]
        )

        # ----------------------------------------------------
        # Best-particle position error
        # ----------------------------------------------------

        best_position_error = math.hypot(
            best_x
            - true_position[0],

            best_y
            - true_position[1]
        )


        # ----------------------------------------------------
        # Best-particle orientation error
        # ----------------------------------------------------

        best_angle_error = (
            best_theta
            - true_theta
        )

        best_angle_error = (
            best_angle_error
            + math.pi
        ) % (
            2.0 * math.pi
        ) - math.pi


        # ====================================================
        # PARTICLE VISUALISATION
        # ====================================================

        particle_filter.draw_debug()


        # ====================================================
        # LOCALISATION ERRORS
        # ====================================================

        # ----------------------------------------------------
        # Odometry position error
        # ----------------------------------------------------

        odom_position_error = math.hypot(
            odom_x
            - true_position[0],

            odom_y
            - true_position[1]
        )


        # ----------------------------------------------------
        # Particle-filter position error
        # ----------------------------------------------------

        pf_position_error = math.hypot(
            pf_x
            - true_position[0],

            pf_y
            - true_position[1]
        )


        # ----------------------------------------------------
        # Odometry orientation error
        # ----------------------------------------------------

        odom_angle_error = (
            odom_theta
            - true_theta
        )

        odom_angle_error = (
            odom_angle_error
            + math.pi
        ) % (
            2.0 * math.pi
        ) - math.pi


        # ----------------------------------------------------
        # Particle-filter orientation error
        # ----------------------------------------------------

        pf_angle_error = (
            pf_theta
            - true_theta
        )

        pf_angle_error = (
            pf_angle_error
            + math.pi
        ) % (
            2.0 * math.pi
        ) - math.pi


        # ====================================================
        # PARTICLE FILTER RESAMPLING
        # ====================================================

        (
            resampled,
            n_eff
        ) = particle_filter.resample()

        # ====================================================
        # SAVE LOCALISATION DATA
        # ====================================================

        simulation_time = (
            step_counter
            / 240.0
        )

        csv_writer.writerow([

            # Simulation
            step_counter,
            simulation_time,

            # Ground truth
            true_position[0],
            true_position[1],
            true_theta,

            # Odometry
            odom_x,
            odom_y,
            odom_theta,

            # Particle-filter weighted mean
            pf_x,
            pf_y,
            pf_theta,

            # Best particle
            best_x,
            best_y,
            best_theta,
            best_weight,

            # Position uncertainty
            pf_stats["std_x"],
            pf_stats["std_y"],

            pf_stats["cov_xx"],
            pf_stats["cov_xy"],
            pf_stats["cov_yy"],

            # Orientation uncertainty
            pf_stats["theta_std"],
            pf_stats["theta_variance"],
            pf_stats["resultant_length"],

            # Position errors
            odom_position_error,
            pf_position_error,
            best_position_error,

            # Orientation errors
            abs(
                odom_angle_error
            ),

            abs(
                pf_angle_error
            ),

            abs(
                best_angle_error
            ),

            # Particle-filter state
            n_eff,
            resampled,

            # Motion prediction
            delta_forward,
            delta_lateral,
            delta_theta
        ])


        # Write the row immediately to disk.
        #
        # This is useful during development because closing the
        # simulation unexpectedly will not lose the latest records.

        csv_file.flush()


        # ====================================================
        # LOCALISATION DIAGNOSTICS
        # ====================================================
        if PRINT_DATA:
            print(
                "\n"
                "==================================================\n"
                "               LOCALISATION STATUS\n"
                "==================================================\n"
                "\n"
                "GROUND TRUTH\n"
                f"  x     : {true_position[0]:.3f} m\n"
                f"  y     : {true_position[1]:.3f} m\n"
                f"  theta : "
                f"{math.degrees(true_theta):.2f}°\n"
                "\n"
                "ODOMETRY\n"
                f"  x     : {odom_x:.3f} m\n"
                f"  y     : {odom_y:.3f} m\n"
                f"  theta : "
                f"{math.degrees(odom_theta):.2f}°\n"
                "\n"
                "PARTICLE FILTER - WEIGHTED MEAN\n"
                f"  x     : {pf_x:.3f} m\n"
                f"  y     : {pf_y:.3f} m\n"
                f"  theta : "
                f"{math.degrees(pf_theta):.2f}°\n"
                "\n"
                "BEST PARTICLE\n"
                f"  x      : {best_x:.3f} m\n"
                f"  y      : {best_y:.3f} m\n"
                f"  theta  : "
                f"{math.degrees(best_theta):.2f}°\n"
                f"  weight : "
                f"{best_weight:.6f}\n"
                "\n"
                "---------------- POSITION UNCERTAINTY -------------\n"
                "\n"
                f"std x                : "
                f"{pf_stats['std_x']:.3f} m\n"
                f"std y                : "
                f"{pf_stats['std_y']:.3f} m\n"
                f"covariance xy        : "
                f"{pf_stats['cov_xy']:.5f} m²\n"
                "\n"
                "--------------- ORIENTATION UNCERTAINTY -----------\n"
                "\n"
                f"circular std         : "
                f"{math.degrees(pf_stats['theta_std']):.2f}°\n"
                f"circular variance    : "
                f"{pf_stats['theta_variance']:.5f}\n"
                f"resultant length     : "
                f"{pf_stats['resultant_length']:.5f}\n"
                "\n"
                "-------------------- ERROR -----------------------\n"
                "\n"
                f"ODOM position error  : "
                f"{odom_position_error:.3f} m\n"
                f"PF mean pos. error   : "
                f"{pf_position_error:.3f} m\n"
                f"BEST pos. error      : "
                f"{best_position_error:.3f} m\n"
                "\n"
                f"ODOM angle error     : "
                f"{abs(math.degrees(odom_angle_error)):.2f}°\n"
                f"PF mean angle error  : "
                f"{abs(math.degrees(pf_angle_error)):.2f}°\n"
                f"BEST angle error     : "
                f"{abs(math.degrees(best_angle_error)):.2f}°\n"
                "\n"
                "---------------- PARTICLE FILTER -----------------\n"
                "\n"
                f"N_eff                : "
                f"{n_eff:.2f} / "
                f"{particle_filter.num_particles}\n"
                f"Resampled            : "
                f"{resampled}\n"
                "\n"
                "-------------------- MOTION ----------------------\n"
                "\n"
                f"Forward              : "
                f"{delta_forward:.3f} m\n"
                f"Lateral              : "
                f"{delta_lateral:.3f} m\n"
                f"Rotation             : "
                f"{math.degrees(delta_theta):.2f}°\n"
                "\n"
                "==================================================\n"
            )
# Disconnect
# ============================================================
# SAFE SHUTDOWN
# ============================================================
#
# This code runs after leaving the simulation loop with Q.

print(
    "\nSaving localisation data..."
)

# Make sure all buffered data is written to disk.
csv_file.flush()

# Close the CSV file.
csv_file.close()

print(
    f"CSV saved successfully:\n"
    f"{csv_path}"
)


# ------------------------------------------------------------
# Close PyBullet simulation
# ------------------------------------------------------------

wld.end()


print(
    "\n"
    "==========================================\n"
    "Experiment finished successfully.\n"
    "==========================================\n"
)