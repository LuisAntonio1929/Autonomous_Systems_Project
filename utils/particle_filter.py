import math
import numpy as np
import pybullet as p

class ParticleFilter:

    def __init__(self,scan_map,num_particles=500):

        self.scan_map = scan_map
        self.num_particles = num_particles

        # Each particle stores:
        #
        # [x, y, theta, weight]
        #
        # x and y are expressed in PyBullet meters.
        self.particles = np.zeros((num_particles, 4))

        # ------------------------------------------------------------
        # Particle visualisation
        # ------------------------------------------------------------

        self.max_visual_particles = 75

        # Persistent heading lines.
        self.particle_heading_ids = [
            -1
        ] * self.max_visual_particles

        # Persistent point cloud.
        self.particle_points_id = -1

        # Separate persistent point for the best particle.
        self.best_particle_point_id = -1

    # ------------------------------------------------------------
    # Particle initialisation around a known pose
    # ------------------------------------------------------------

    def initialize_gaussian(self,x,y,theta,position_std=0.40,angle_std=math.radians(10.0)):
        """
        Initialise particles around a known starting pose.

        The robot is assumed to start approximately at (x, y, theta),
        but uncertainty is represented by spreading the particles
        around that pose.
        """

        particles = []

        while len(particles) < self.num_particles:

            particle_x = np.random.normal(
                x,
                position_std
            )

            particle_y = np.random.normal(
                y,
                position_std
            )

            particle_theta = np.random.normal(
                theta,
                angle_std
            )

            # Normalize orientation to [-pi, pi].
            particle_theta = (
                particle_theta
                + math.pi
            ) % (
                2.0 * math.pi
            ) - math.pi

            # Only accept particles in free space.
            if not self.is_free(
                particle_x,
                particle_y
            ):
                continue

            particles.append([
                particle_x,
                particle_y,
                particle_theta,
                1.0 / self.num_particles
            ])

        self.particles = np.array(
            particles,
            dtype=float
        )

    # ------------------------------------------------------------
    # Particle initialisation
    # ------------------------------------------------------------

    def initialize_uniform(self):
        """
        Distribute particles randomly over free areas of the map
        with random orientations.
        """

        particles = []

        while len(particles) < self.num_particles:

            # Random position in meters.
            x = np.random.uniform(
                0,
                self.scan_map.cols / self.scan_map.scale
            )

            y = np.random.uniform(
                0,
                self.scan_map.rows / self.scan_map.scale
            )

            # Convert to internal map coordinates.
            map_x = int(
                x * self.scan_map.scale
            )

            map_y = int(
                y * self.scan_map.scale
            )

            # Ignore positions outside the map.
            if (
                map_x < 0
                or map_x >= self.scan_map.cols
                or map_y < 0
                or map_y >= self.scan_map.rows
            ):
                continue

            # Only create particles in free space.
            if self.scan_map.grid[
                map_y,
                map_x
            ] != 0:
                continue

            theta = np.random.uniform(
                -math.pi,
                math.pi
            )

            particles.append([
                x,
                y,
                theta,
                1.0 / self.num_particles
            ])

        self.particles = np.array(
            particles
        )

    # ------------------------------------------------------------
    # Sensor update
    # ------------------------------------------------------------

    def update_weights(self,lidar_distances,sigma=0.50):
        """
        Update particle probabilities using the LiDAR measurement.

        The new sensor likelihood is combined with the previous
        particle weight so information can accumulate across
        consecutive measurements when resampling is not performed.

        Logarithms are used to avoid numerical underflow.
        """

        log_weights = np.full(
            self.num_particles,
            -np.inf
        )

        for i in range(
            self.num_particles
        ):

            x, y, theta, previous_weight = (
                self.particles[i]
            )

            # ----------------------------------------------------
            # Reject impossible particle poses
            # ----------------------------------------------------

            if not self.is_free(
                x,
                y
            ):
                continue

            # ----------------------------------------------------
            # Expected LiDAR scan
            # ----------------------------------------------------

            expected_scaled, _ = (
                self.scan_map.scan(
                    start_x=(
                        x
                        * self.scan_map.scale
                    ),
                    start_y=(
                        y
                        * self.scan_map.scale
                    ),
                    start_a=theta
                )
            )

            expected = (
                expected_scaled
                / self.scan_map.scale
            )

            # ----------------------------------------------------
            # Measurement error
            # ----------------------------------------------------

            error = (
                lidar_distances
                - expected
            )

            mean_squared_error = np.mean(
                error ** 2
            )

            # ----------------------------------------------------
            # Sensor likelihood
            # ----------------------------------------------------

            log_likelihood = (
                -mean_squared_error
                / (
                    2.0
                    * sigma ** 2
                )
            )

            # ----------------------------------------------------
            # Combine previous belief with new measurement
            # ----------------------------------------------------

            safe_previous_weight = max(
                previous_weight,
                1e-300
            )

            log_weights[i] = (
                math.log(
                    safe_previous_weight
                )
                + log_likelihood
            )

        # --------------------------------------------------------
        # Normalize weights safely
        # --------------------------------------------------------

        finite_mask = np.isfinite(
            log_weights
        )

        if not np.any(
            finite_mask
        ):

            # Complete filter failure:
            # restore uniform weights.
            self.particles[:, 3] = (
                1.0
                / self.num_particles
            )

            return

        max_log_weight = np.max(
            log_weights[
                finite_mask
            ]
        )

        weights = np.zeros(
            self.num_particles
        )

        weights[
            finite_mask
        ] = np.exp(
            log_weights[
                finite_mask
            ]
            - max_log_weight
        )

        total = np.sum(
            weights
        )

        if total <= 1e-12:

            self.particles[:, 3] = (
                1.0
                / self.num_particles
            )

        else:

            self.particles[:, 3] = (
                weights
                / total
            )

    # ------------------------------------------------------------
    # Check whether a position is free in the map
    # ------------------------------------------------------------

    def is_free(self, x, y):
        """
        Return True if the position (x, y) is inside the map
        and does not correspond to an occupied map cell.

        Particle coordinates are stored in meters.
        ScanableMap coordinates use the internal map scale.
        """

        map_x = int(
            x * self.scan_map.scale
        )

        map_y = int(
            y * self.scan_map.scale
        )

        # Outside map limits.
        if (
            map_x < 0
            or map_x >= self.scan_map.cols
            or map_y < 0
            or map_y >= self.scan_map.rows
        ):
            return False

        # Grid value 0 means free space.
        return (
            self.scan_map.grid[
                map_y,
                map_x
            ] == 0
        )


    # ------------------------------------------------------------
    # Generate one random free particle
    # ------------------------------------------------------------

    def random_free_particle(self):
        """
        Generate a random particle located in free space.

        Returns:
            [x, y, theta, weight]
        """

        map_width = (
            self.scan_map.cols
            / self.scan_map.scale
        )

        map_height = (
            self.scan_map.rows
            / self.scan_map.scale
        )

        while True:

            x = np.random.uniform(
                0.0,
                map_width
            )

            y = np.random.uniform(
                0.0,
                map_height
            )

            if self.is_free(x, y):

                theta = np.random.uniform(
                    -math.pi,
                    math.pi
                )

                return np.array([
                    x,
                    y,
                    theta,
                    1.0 / self.num_particles
                ])


    # ------------------------------------------------------------
    # Effective sample size
    # ------------------------------------------------------------

    def effective_sample_size(self):
        """
        Calculate the effective number of particles.

        N_eff becomes small when most of the probability mass
        is concentrated in only a few particles.
        """

        weights = (
            self.particles[:, 3]
        )

        total = np.sum(
            weights
        )

        if total <= 0.0:
            return 0.0

        weights = (
            weights
            / total
        )

        return (
            1.0
            / np.sum(
                weights ** 2
            )
        )


    # ------------------------------------------------------------
    # Systematic resampling
    # ------------------------------------------------------------

    def resample(
        self,
        threshold_ratio=0.5,
        random_fraction=0.0,
        position_noise=0.05,
        angle_noise=math.radians(2.0)
    ):
        """
        Perform systematic resampling when the particle weights
        have become sufficiently concentrated.

        A small fraction of new random particles is introduced
        to preserve global diversity and help the filter recover
        from an incorrect hypothesis.

        Returns:
            resampled:
                True if resampling was performed.

            n_eff:
                Effective sample size before resampling.
        """

        # --------------------------------------------------------
        # Calculate effective sample size
        # --------------------------------------------------------

        n_eff = (
            self.effective_sample_size()
        )

        threshold = (
            threshold_ratio
            * self.num_particles
        )

        # --------------------------------------------------------
        # Do not resample if particle diversity is still good
        # --------------------------------------------------------

        if n_eff >= threshold:

            return (
                False,
                n_eff
            )

        # --------------------------------------------------------
        # Normalize particle weights
        # --------------------------------------------------------

        weights = (
            self.particles[:, 3]
            .copy()
        )

        total = np.sum(
            weights
        )

        if total <= 0.0:

            weights[:] = (
                1.0
                / self.num_particles
            )

        else:

            weights /= total


        # --------------------------------------------------------
        # Number of particles coming from resampling
        # and number of completely new random particles
        # --------------------------------------------------------

        num_random = max(
            0,
            int(
                self.num_particles
                * random_fraction
            )
        )

        num_resampled = (
            self.num_particles
            - num_random
        )


        # --------------------------------------------------------
        # Systematic resampling positions
        # --------------------------------------------------------

        start = np.random.uniform(
            0.0,
            1.0 / num_resampled
        )

        positions = (
            start
            + np.arange(
                num_resampled
            ) / num_resampled
        )


        # --------------------------------------------------------
        # Cumulative weight distribution
        # --------------------------------------------------------

        cumulative_weights = (
            np.cumsum(
                weights
            )
        )

        # Avoid numerical problems at the end.
        cumulative_weights[-1] = 1.0


        # --------------------------------------------------------
        # Select particle indices
        # --------------------------------------------------------

        selected_indices = (
            np.searchsorted(
                cumulative_weights,
                positions
            )
        )


        # --------------------------------------------------------
        # Create new particle population
        # --------------------------------------------------------

        new_particles = np.zeros_like(
            self.particles
        )


        # ========================================================
        # RESAMPLED PARTICLES
        # ========================================================

        for new_index, old_index in enumerate(
            selected_indices
        ):

            old_particle = (
                self.particles[
                    old_index
                ]
            )

            old_x = (
                old_particle[0]
            )

            old_y = (
                old_particle[1]
            )

            old_theta = (
                old_particle[2]
            )


            # ----------------------------------------------------
            # Add a small amount of roughening
            # ----------------------------------------------------
            #
            # This prevents all copies of one particle from
            # becoming mathematically identical.

            valid_particle = False

            for _ in range(10):

                new_x = (
                    old_x
                    + np.random.normal(
                        0.0,
                        position_noise
                    )
                )

                new_y = (
                    old_y
                    + np.random.normal(
                        0.0,
                        position_noise
                    )
                )

                new_theta = (
                    old_theta
                    + np.random.normal(
                        0.0,
                        angle_noise
                    )
                )


                # Normalize orientation to [-pi, pi].
                new_theta = (
                    new_theta
                    + math.pi
                ) % (
                    2.0 * math.pi
                ) - math.pi


                # Only accept particles located in free space.
                if self.is_free(
                    new_x,
                    new_y
                ):

                    valid_particle = True

                    break


            # ----------------------------------------------------
            # Fallback if roughening produced invalid positions
            # ----------------------------------------------------

            if not valid_particle:

                # Keep original particle if it was valid.
                if self.is_free(
                    old_x,
                    old_y
                ):

                    new_x = old_x
                    new_y = old_y
                    new_theta = old_theta

                else:

                    random_particle = (
                        self.random_free_particle()
                    )

                    new_x = (
                        random_particle[0]
                    )

                    new_y = (
                        random_particle[1]
                    )

                    new_theta = (
                        random_particle[2]
                    )


            new_particles[
                new_index
            ] = [
                new_x,
                new_y,
                new_theta,
                1.0 / self.num_particles
            ]


        # ========================================================
        # RANDOM PARTICLE INJECTION
        # ========================================================
        #
        # These particles maintain global localisation diversity.
        #
        # If the filter initially converges to a wrong corridor,
        # new particles still have an opportunity to discover the
        # correct region later.

        for i in range(
            num_resampled,
            self.num_particles
        ):

            new_particles[i] = (
                self.random_free_particle()
            )


        # --------------------------------------------------------
        # Replace old particle set
        # --------------------------------------------------------

        self.particles = (
            new_particles
        )


        return (
            True,
            n_eff
        )
    
    # ------------------------------------------------------------
    # Motion prediction
    # ------------------------------------------------------------

    def predict(
        self,
        delta_forward,
        delta_lateral,
        delta_theta,
        translation_noise=0.02,
        rotation_noise=math.radians(1.0),
        slip_probability=0.15
    ):
        """
        Move particles according to wheel odometry.

        The motion model includes uncertainty caused by wheel slip
        or wheel stall.

        Most particles follow the odometry normally.

        A small proportion of particles assume that the robot moved
        less than the wheel encoders indicate. This allows the
        particle filter to recover when the wheels rotate while the
        robot is blocked by an obstacle.
        """

        # --------------------------------------------------------
        # Do nothing if there is effectively no odometry motion
        # --------------------------------------------------------

        motion_magnitude = math.hypot(
            delta_forward,
            delta_lateral
        )

        if (
            motion_magnitude < 1e-4
            and abs(delta_theta) < 1e-4
        ):
            return


        # --------------------------------------------------------
        # Motion-dependent uncertainty
        # --------------------------------------------------------
        #
        # Greater odometry motion creates greater uncertainty.

        translation_std = (
            translation_noise
            + 0.05
            * motion_magnitude
        )

        rotation_std = (
            rotation_noise
            + 0.05
            * abs(delta_theta)
        )


        # --------------------------------------------------------
        # Predict every particle
        # --------------------------------------------------------

        for i in range(
            self.num_particles
        ):

            x, y, theta, weight = (
                self.particles[i]
            )


            # ====================================================
            # SLIP / STALL HYPOTHESIS
            # ====================================================
            #
            # Most particles trust odometry.
            #
            # Some particles represent the possibility that the
            # wheels rotated but the chassis moved less or did not
            # move at all.

            if np.random.random() < slip_probability:

                # Movement can range from zero to approximately
                # one third of the encoder-estimated displacement.
                motion_scale = np.random.uniform(
                    0.0,
                    0.35
                )

            else:

                # Normal odometry hypothesis.
                #
                # Small variation around 1.0 represents ordinary
                # wheel-slip uncertainty.
                motion_scale = np.random.normal(
                    1.0,
                    0.05
                )


            # ----------------------------------------------------
            # Apply motion scale
            # ----------------------------------------------------

            scaled_forward = (
                delta_forward
                * motion_scale
            )

            scaled_lateral = (
                delta_lateral
                * motion_scale
            )


            # ----------------------------------------------------
            # Add random translation uncertainty
            # ----------------------------------------------------

            noisy_forward = (
                scaled_forward
                + np.random.normal(
                    0.0,
                    translation_std
                )
            )

            noisy_lateral = (
                scaled_lateral
                + np.random.normal(
                    0.0,
                    translation_std
                )
            )


            # ----------------------------------------------------
            # Angular motion
            # ----------------------------------------------------

            noisy_theta = (
                delta_theta
                + np.random.normal(
                    0.0,
                    rotation_std
                )
            )


            # ----------------------------------------------------
            # Transform local robot movement into world movement
            # ----------------------------------------------------

            new_x = (
                x
                + math.cos(theta)
                * noisy_forward
                - math.sin(theta)
                * noisy_lateral
            )

            new_y = (
                y
                + math.sin(theta)
                * noisy_forward
                + math.cos(theta)
                * noisy_lateral
            )

            new_theta = (
                theta
                + noisy_theta
            )


            # ----------------------------------------------------
            # Normalize orientation to [-pi, pi]
            # ----------------------------------------------------

            new_theta = (
                new_theta
                + math.pi
            ) % (
                2.0 * math.pi
            ) - math.pi


            # ----------------------------------------------------
            # Update particle only if new position is valid
            # ----------------------------------------------------

            if self.is_free(
                new_x,
                new_y
            ):

                self.particles[i, 0] = (
                    new_x
                )

                self.particles[i, 1] = (
                    new_y
                )

                self.particles[i, 2] = (
                    new_theta
                )

            else:

                # If motion would place the particle inside a wall,
                # keep its previous position but still allow a small
                # orientation change.

                self.particles[i, 2] = (
                    new_theta
                )

    # ------------------------------------------------------------
    # Particle-filter pose estimation
    # ------------------------------------------------------------

    def estimate_pose(self):
        """
        Estimate the robot pose from the complete particle
        distribution.

        Position (x, y) is calculated using the weighted mean.

        Orientation theta is calculated using a weighted circular
        mean so angles close to +pi and -pi are handled correctly.

        Returns:
            estimated_x
            estimated_y
            estimated_theta
        """

        if len(self.particles) == 0:

            return (
                None,
                None,
                None
            )

        # --------------------------------------------------------
        # Particle weights
        # --------------------------------------------------------

        weights = (
            self.particles[:, 3]
            .copy()
        )

        total_weight = np.sum(
            weights
        )

        # --------------------------------------------------------
        # Safety fallback
        # --------------------------------------------------------

        if total_weight <= 0.0:

            weights[:] = (
                1.0
                / self.num_particles
            )

        else:

            weights /= total_weight


        # --------------------------------------------------------
        # Estimate X position
        # --------------------------------------------------------

        estimated_x = np.sum(
            self.particles[:, 0]
            * weights
        )


        # --------------------------------------------------------
        # Estimate Y position
        # --------------------------------------------------------

        estimated_y = np.sum(
            self.particles[:, 1]
            * weights
        )


        # --------------------------------------------------------
        # Estimate orientation
        # --------------------------------------------------------
        #
        # A normal arithmetic mean cannot be used for angles.
        #
        # For example:
        #
        #     +179 degrees
        #     -179 degrees
        #
        # represent almost the same orientation.
        #
        # Their arithmetic mean would incorrectly produce 0.
        #
        # We therefore use a circular mean.

        sin_sum = np.sum(
            np.sin(
                self.particles[:, 2]
            )
            * weights
        )

        cos_sum = np.sum(
            np.cos(
                self.particles[:, 2]
            )
            * weights
        )

        estimated_theta = math.atan2(
            sin_sum,
            cos_sum
        )


        return (
            estimated_x,
            estimated_y,
            estimated_theta
        )

    # ------------------------------------------------------------
    # Particle-filter statistics
    # ------------------------------------------------------------

    def estimate_statistics(self):
        """
        Calculate statistics describing the particle distribution.

        The particle cloud approximates the probability
        distribution of the robot pose.

        Position:
            - weighted mean x
            - weighted mean y
            - weighted standard deviation x
            - weighted standard deviation y
            - weighted covariance xy

        Orientation:
            - weighted circular mean
            - circular standard deviation
            - circular variance

        Best particle:
            - particle with the highest weight

        Returns:
            Dictionary containing all statistics.
        """

        if len(self.particles) == 0:
            return None

        # --------------------------------------------------------
        # Normalize particle weights
        # --------------------------------------------------------

        weights = (
            self.particles[:, 3]
            .copy()
        )

        total_weight = np.sum(
            weights
        )

        if total_weight <= 0.0:

            weights[:] = (
                1.0
                / self.num_particles
            )

        else:

            weights /= total_weight


        # --------------------------------------------------------
        # Particle states
        # --------------------------------------------------------

        x_values = (
            self.particles[:, 0]
        )

        y_values = (
            self.particles[:, 1]
        )

        theta_values = (
            self.particles[:, 2]
        )


        # ========================================================
        # POSITION MEAN
        # ========================================================

        mean_x = np.sum(
            weights
            * x_values
        )

        mean_y = np.sum(
            weights
            * y_values
        )


        # ========================================================
        # POSITION COVARIANCE
        # ========================================================

        dx = (
            x_values
            - mean_x
        )

        dy = (
            y_values
            - mean_y
        )

        covariance_xx = np.sum(
            weights
            * dx
            * dx
        )

        covariance_yy = np.sum(
            weights
            * dy
            * dy
        )

        covariance_xy = np.sum(
            weights
            * dx
            * dy
        )


        # --------------------------------------------------------
        # Position standard deviations
        # --------------------------------------------------------

        std_x = math.sqrt(
            max(
                covariance_xx,
                0.0
            )
        )

        std_y = math.sqrt(
            max(
                covariance_yy,
                0.0
            )
        )


        # ========================================================
        # ORIENTATION STATISTICS
        # ========================================================
        #
        # Orientation requires circular statistics because
        # +179 degrees and -179 degrees are nearly identical.

        mean_sin = np.sum(
            weights
            * np.sin(
                theta_values
            )
        )

        mean_cos = np.sum(
            weights
            * np.cos(
                theta_values
            )
        )


        # --------------------------------------------------------
        # Circular mean
        # --------------------------------------------------------

        mean_theta = math.atan2(
            mean_sin,
            mean_cos
        )


        # --------------------------------------------------------
        # Mean resultant length
        # --------------------------------------------------------
        #
        # R close to 1:
        #     orientations are strongly concentrated.
        #
        # R close to 0:
        #     orientations are highly dispersed.

        resultant_length = math.hypot(
            mean_cos,
            mean_sin
        )

        resultant_length = np.clip(
            resultant_length,
            1e-12,
            1.0
        )


        # --------------------------------------------------------
        # Circular variance
        # --------------------------------------------------------

        circular_variance = (
            1.0
            - resultant_length
        )


        # --------------------------------------------------------
        # Circular standard deviation
        # --------------------------------------------------------

        circular_std = math.sqrt(
            -2.0
            * math.log(
                resultant_length
            )
        )


        # ========================================================
        # BEST PARTICLE
        # ========================================================

        best_index = np.argmax(
            weights
        )

        best_particle = (
            self.particles[
                best_index
            ]
        )


        return {

            # Mean pose
            "mean_x": mean_x,
            "mean_y": mean_y,
            "mean_theta": mean_theta,

            # Position uncertainty
            "std_x": std_x,
            "std_y": std_y,
            "cov_xx": covariance_xx,
            "cov_xy": covariance_xy,
            "cov_yy": covariance_yy,

            # Orientation uncertainty
            "theta_std": circular_std,
            "theta_variance": circular_variance,
            "resultant_length": resultant_length,

            # Best particle
            "best_x": best_particle[0],
            "best_y": best_particle[1],
            "best_theta": best_particle[2],
            "best_weight": best_particle[3]
        }

    # ------------------------------------------------------------
    # Particle debug visualisation
    # ------------------------------------------------------------
    def draw_debug(self,line_length=0.40):
        """
        Visualise the highest-weight particles.

        Normal particles:
            - dark blue position point
            - lighter blue orientation line

        Best particle:
            - large dark-red position point
            - longer, thicker light-red orientation line
            - slightly higher than the normal particles

        Only the highest-weight particles are displayed.
        """

        if len(self.particles) == 0:
            return


        # --------------------------------------------------------
        # Select particles with highest weight
        # --------------------------------------------------------

        num_to_draw = min(
            self.max_visual_particles,
            self.num_particles
        )

        top_indices = np.argsort(
            self.particles[:, 3]
        )[-num_to_draw:]

        best_index = (
            top_indices[-1]
        )


        # --------------------------------------------------------
        # Visual parameters
        # --------------------------------------------------------

        normal_z = 0.55

        # Put the best particle slightly above the others
        # so it remains visible when particles overlap.
        best_z = 0.62


        # --------------------------------------------------------
        # Normal particle points
        # --------------------------------------------------------

        normal_point_positions = []
        normal_point_colors = []


        # --------------------------------------------------------
        # Draw orientation lines
        # --------------------------------------------------------

        for visual_slot, particle_index in enumerate(
            top_indices
        ):

            x, y, theta, weight = (
                self.particles[
                    particle_index
                ]
            )


            # ====================================================
            # BEST PARTICLE
            # ====================================================

            if particle_index == best_index:

                start = [
                    x,
                    y,
                    best_z
                ]

                # Make the best orientation line longer.
                best_line_length = 0.65

                end = [
                    x
                    + best_line_length
                    * math.cos(theta),

                    y
                    + best_line_length
                    * math.sin(theta),

                    best_z
                ]

                # Light red orientation line.
                line_color = [
                    1.00,
                    0.15,
                    0.15
                ]

                # Considerably thicker than normal particles.
                width = 6.0


            # ====================================================
            # NORMAL PARTICLE
            # ====================================================

            else:

                start = [
                    x,
                    y,
                    normal_z
                ]

                end = [
                    x
                    + line_length
                    * math.cos(theta),

                    y
                    + line_length
                    * math.sin(theta),

                    normal_z
                ]

                # Lighter blue orientation line.
                line_color = [
                    0.25,
                    0.60,
                    1.00
                ]

                width = 1.5


                # Store only normal particles in this point batch.
                normal_point_positions.append(
                    start
                )

                # Dark blue position point.
                normal_point_colors.append([
                    0.00,
                    0.10,
                    0.50
                ])


            # ----------------------------------------------------
            # Update persistent orientation line
            # ----------------------------------------------------

            self.particle_heading_ids[
                visual_slot
            ] = p.addUserDebugLine(
                lineFromXYZ=start,
                lineToXYZ=end,
                lineColorRGB=line_color,
                lineWidth=width,
                lifeTime=0,
                replaceItemUniqueId=(
                    self.particle_heading_ids[
                        visual_slot
                    ]
                )
            )


        # ========================================================
        # DRAW NORMAL PARTICLE POINTS
        # ========================================================
        #
        # All normal points are rendered together efficiently.

        if len(
            normal_point_positions
        ) > 0:

            self.particle_points_id = (
                p.addUserDebugPoints(
                    pointPositions=(
                        normal_point_positions
                    ),
                    pointColorsRGB=(
                        normal_point_colors
                    ),
                    pointSize=5,
                    lifeTime=0,
                    replaceItemUniqueId=(
                        self.particle_points_id
                    )
                )
            )


        # ========================================================
        # DRAW BEST PARTICLE POINT
        # ========================================================
        #
        # Draw it separately so it can have a much larger
        # point size than the normal particles.

        best_particle = (
            self.particles[
                best_index
            ]
        )

        best_x = (
            best_particle[0]
        )

        best_y = (
            best_particle[1]
        )

        self.best_particle_point_id = (
            p.addUserDebugPoints(
                pointPositions=[
                    [
                        best_x,
                        best_y,
                        best_z
                    ]
                ],
                pointColorsRGB=[
                    [
                        0.65,
                        0.00,
                        0.00
                    ]
                ],
                pointSize=12,
                lifeTime=0,
                replaceItemUniqueId=(
                    self.best_particle_point_id
                )
            )
        )