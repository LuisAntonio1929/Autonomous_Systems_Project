import math
import random
import numpy as np
import pybullet as p


class DemoRobot:
    def __init__(self, x=0, y=0):

        # ------------------------------------------------------------
        # Robot dimensions
        # ------------------------------------------------------------

        body_length = 1.2
        body_width = 0.8
        body_height = 0.3

        wheel_radius = 0.15
        wheel_width = 0.10

        caster_radius = 0.06

        # ------------------------------------------------------------
        # Chassis
        # ------------------------------------------------------------

        chassis_collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[
                body_length / 2,
                body_width / 2,
                body_height / 2
            ]
        )

        chassis_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[
                body_length / 2,
                body_width / 2,
                body_height / 2
            ],
            rgbaColor=[0.2, 0.4, 0.8, 1.0]
        )

        # ------------------------------------------------------------
        # Drive wheels
        # ------------------------------------------------------------
        #
        # PyBullet cylinders are created along the Z axis.
        # Rotate their geometry 90 degrees around X so that
        # the wheel axle lies along the Y axis.
        # ------------------------------------------------------------

        wheel_orientation = p.getQuaternionFromEuler(
            [math.pi / 2, 0, 0]
        )

        wheel_collision = p.createCollisionShape(
            p.GEOM_CYLINDER,
            radius=wheel_radius,
            height=wheel_width,
            collisionFrameOrientation=wheel_orientation
        )

        wheel_visual = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=wheel_radius,
            length=wheel_width,
            visualFrameOrientation=wheel_orientation,
            rgbaColor=[0.12, 0.12, 0.12, 1.0]
        )

        # ------------------------------------------------------------
        # Passive caster supports
        # ------------------------------------------------------------

        caster_collision = p.createCollisionShape(
            p.GEOM_SPHERE,
            radius=caster_radius
        )

        caster_visual = p.createVisualShape(
            p.GEOM_SPHERE,
            radius=caster_radius,
            rgbaColor=[0.25, 0.25, 0.25, 1.0]
        )

        # ------------------------------------------------------------
        # Link positions
        # ------------------------------------------------------------
        #
        # Robot coordinate convention:
        #
        #     +X = forward
        #     +Y = left
        #     -Y = right
        #
        # Drive wheels are located at x = 0.
        # ------------------------------------------------------------

        left_wheel_position = [
            0,
            body_width / 2 + wheel_width / 2,
            -body_height / 2
        ]

        right_wheel_position = [
            0,
            -body_width / 2 - wheel_width / 2,
            -body_height / 2
        ]

        # Caster centers must be lower than the chassis so that
        # they touch the ground.
        caster_z = (
            caster_radius
            - (wheel_radius + body_height / 2)
        )

        front_caster_position = [
            body_length * 0.40,
            0,
            caster_z
        ]

        rear_caster_position = [
            -body_length * 0.40,
            0,
            caster_z
        ]

        # Link order:
        #
        # 0 -> left drive wheel
        # 1 -> right drive wheel
        # 2 -> front caster
        # 3 -> rear caster

        link_positions = [
            left_wheel_position,
            right_wheel_position,
            front_caster_position,
            rear_caster_position
        ]

        # ------------------------------------------------------------
        # Multi-body configuration
        # ------------------------------------------------------------

        link_masses = [
            2.0,   # left wheel
            2.0,   # right wheel
            0.2,   # front caster
            0.2    # rear caster
        ]

        link_collision_shapes = [
            wheel_collision,
            wheel_collision,
            caster_collision,
            caster_collision
        ]

        link_visual_shapes = [
            wheel_visual,
            wheel_visual,
            caster_visual,
            caster_visual
        ]

        link_orientations = [
            [0, 0, 0, 1],
            [0, 0, 0, 1],
            [0, 0, 0, 1],
            [0, 0, 0, 1]
        ]

        link_inertial_frame_positions = [
            [0, 0, 0]
        ] * 4

        link_inertial_frame_orientations = [
            [0, 0, 0, 1]
        ] * 4

        link_parent_indices = [0] * 4

        # Two powered revolute wheels.
        # Casters are passive fixed supports.
        link_joint_types = [
            p.JOINT_REVOLUTE,
            p.JOINT_REVOLUTE,
            p.JOINT_FIXED,
            p.JOINT_FIXED
        ]

        # Drive-wheel rotation axis = Y
        # Fixed joints ignore their axis.
        link_joint_axis = [
            [0, 1, 0],
            [0, 1, 0],
            [0, 0, 1],
            [0, 0, 1]
        ]

        # ------------------------------------------------------------
        # Create robot
        # ------------------------------------------------------------

        self.agv_id = p.createMultiBody(
            baseMass=50,

            baseCollisionShapeIndex=chassis_collision,
            baseVisualShapeIndex=chassis_visual,

            basePosition=[
                x,
                y,
                wheel_radius + body_height / 2
            ],

            linkMasses=link_masses,

            linkCollisionShapeIndices=link_collision_shapes,
            linkVisualShapeIndices=link_visual_shapes,

            linkPositions=link_positions,
            linkOrientations=link_orientations,

            linkInertialFramePositions=link_inertial_frame_positions,
            linkInertialFrameOrientations=link_inertial_frame_orientations,

            linkParentIndices=link_parent_indices,

            linkJointTypes=link_joint_types,
            linkJointAxis=link_joint_axis
        )

        # ------------------------------------------------------------
        # Joint IDs
        # ------------------------------------------------------------

        self.left_wheel = 0
        self.right_wheel = 1

        self.drive_wheels = [
            self.left_wheel,
            self.right_wheel
        ]

        self.casters = [2, 3]

        # ------------------------------------------------------------
        # Drive wheel dynamics
        # ------------------------------------------------------------

        for joint_id in self.drive_wheels:

            # Disable default joint motor.
            p.setJointMotorControl2(
                bodyUniqueId=self.agv_id,
                jointIndex=joint_id,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=0,
                force=0
            )

            p.changeDynamics(
                self.agv_id,
                joint_id,
                lateralFriction=1.0,
                rollingFriction=0.001,
                spinningFriction=0.001
            )

        # ------------------------------------------------------------
        # Caster dynamics
        # ------------------------------------------------------------
        #
        # The casters are simplified passive supports.
        # Very low friction allows them to slide when the
        # differential drive rotates the robot.
        # ------------------------------------------------------------

        for caster_id in self.casters:

            p.changeDynamics(
                self.agv_id,
                caster_id,
                lateralFriction=0.05,
                rollingFriction=0.0,
                spinningFriction=0.0
            )

        # ------------------------------------------------------------
        # Control parameters
        # ------------------------------------------------------------

        self.left_speed = 0.0
        self.right_speed = 0.0

        self.drive_speed = 6.0
        self.turn_speed = 3.0

        self.motor_force = 20.0

        # Motor noise level
        # 0.05 means ±5 %
        self.motor_noise_level = 0.05

        # ------------------------------------------------------------
        # Autonomous route
        # ------------------------------------------------------------

        self.route = [
            ("forward", 240*13.5),
            ("stop", 240 * 0.5),
            ("left", 240*1.84),
            ("stop", 240 * 0.5),
            ("forward", 240*3.5),
            ("stop", 240 * 0.5),
            ("left", 240*1.84),
            ("stop", 240 * 0.5),
            ("forward", 240*6.5),
            ("stop", 240 * 0.5),
            ("right", 240*1.84),
            ("stop", 240 * 0.5),
            ("forward", 240*3.5)
        ]

        self.current_action_index = 0
        self.action_step = 0

        # ------------------------------------------------------------
        # LiDAR parameters
        # ------------------------------------------------------------

        self.num_lidar_beams = 61

        # Front field of view: ±60 degrees
        self.lidar_fov = math.radians(60)

        self.lidar_max_range = 10.0

        # Distance at which the robot should stop
        self.safety_distance = 1.5

        self.person_id = None

    # ----------------------------------------------------------------
    # Manual control
    # ----------------------------------------------------------------

    def user_control(self, keys):
        """
        Manual control used to test/calibrate the Lab 2 robot.

        W = forward
        S = backward
        A = counter-clockwise / left
        D = clockwise / right
        """

        linear = 0.0
        angular = 0.0

        # Forward
        if (
            ord('w') in keys
            and keys[ord('w')] & p.KEY_IS_DOWN
        ):
            linear += 1.0

        # Backward
        if (
            ord('s') in keys
            and keys[ord('s')] & p.KEY_IS_DOWN
        ):
            linear -= 1.0

        # Left / counter-clockwise
        if (
            ord('a') in keys
            and keys[ord('a')] & p.KEY_IS_DOWN
        ):
            angular += 1.0

        # Right / clockwise
        if (
            ord('d') in keys
            and keys[ord('d')] & p.KEY_IS_DOWN
        ):
            angular -= 1.0

        # Differential-drive equations
        self.left_speed = (
            linear * self.drive_speed
            - angular * self.turn_speed
        )

        self.right_speed = (
            linear * self.drive_speed
            + angular * self.turn_speed
        )

    # ----------------------------------------------------------------
    # Apply motor commands
    # ----------------------------------------------------------------

    def step_action(self):

        if self.person_detected():

            self.left_speed = 0.0
            self.right_speed = 0.0

            left_command = 0.0
            right_command = 0.0

            p.setJointMotorControl2(
                bodyUniqueId=self.agv_id,
                jointIndex=self.left_wheel,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=left_command,
                force=self.motor_force
            )

            p.setJointMotorControl2(
                bodyUniqueId=self.agv_id,
                jointIndex=self.right_wheel,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=right_command,
                force=self.motor_force
            )

            return

        if self.current_action_index < len(self.route):

            action, duration = self.route[self.current_action_index]

            if action == "forward":
                self.left_speed = self.drive_speed
                self.right_speed = self.drive_speed

            elif action == "left":
                self.left_speed = -self.turn_speed
                self.right_speed = self.turn_speed

            elif action == "right":
                self.left_speed = self.turn_speed
                self.right_speed = -self.turn_speed

            elif action == "stop":
                self.left_speed = 0.0
                self.right_speed = 0.0

            self.action_step += 1

            if self.action_step >= duration:
                self.current_action_index += 1
                self.action_step = 0

        else:
            self.left_speed = 0.0
            self.right_speed = 0.0

        left_command = self.motor_noise(self.left_speed)
        right_command = self.motor_noise(self.right_speed)

        p.setJointMotorControl2(
            bodyUniqueId=self.agv_id,
            jointIndex=self.left_wheel,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=left_command,
            force=self.motor_force
        )

        p.setJointMotorControl2(
            bodyUniqueId=self.agv_id,
            jointIndex=self.right_wheel,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=right_command,
            force=self.motor_force
        )

    def motor_noise(self, speed):
        """Apply independent multiplicative noise to a motor command."""

        noise_factor = random.uniform(
            1.0 - self.motor_noise_level,
            1.0 + self.motor_noise_level
        )

        return speed * noise_factor

    def ray_cast_lidar(self):
        """
        Simulate a 2D LiDAR using PyBullet ray casting.

        Returns:
            distances: measured distance for each beam
            hit_ids: PyBullet body ID hit by each beam
            angles: beam angles
        """

        robot_pos, robot_orn = p.getBasePositionAndOrientation(
            self.agv_id
        )

        # Robot yaw
        _, _, robot_yaw = p.getEulerFromQuaternion(robot_orn)

        angles = np.linspace(
            robot_yaw - self.lidar_fov,
            robot_yaw + self.lidar_fov,
            self.num_lidar_beams
        )

        ray_from = []
        ray_to = []

        # Sensor position slightly above the chassis
        sensor_origin = [
            robot_pos[0],
            robot_pos[1],
            robot_pos[2] + 0.2
        ]

        for angle in angles:

            direction = np.array([
                np.cos(angle),
                np.sin(angle),
                0.0
            ])

            ray_from.append(sensor_origin)

            ray_to.append([
                sensor_origin[0] + direction[0] * self.lidar_max_range,
                sensor_origin[1] + direction[1] * self.lidar_max_range,
                sensor_origin[2]
            ])

        results = p.rayTestBatch(
            ray_from,
            ray_to
        )

        distances = []
        hit_ids = []

        for result in results:

            hit_object_id = result[0]
            hit_fraction = result[2]

            if hit_object_id >= 0:
                distance = hit_fraction * self.lidar_max_range
            else:
                distance = self.lidar_max_range

            distances.append(distance)
            hit_ids.append(hit_object_id)

        return (
            np.array(distances),
            np.array(hit_ids),
            angles
        )

    def set_person_target(self, person_id):
        """Store the PyBullet body ID of the person to detect."""
        self.person_id = person_id

    def person_detected(self):
        """Return True when the person is detected inside the safety distance."""

        if self.person_id is None:
            return False

        distances, hit_ids, angles = self.ray_cast_lidar()

        person_hits = (
            (hit_ids == self.person_id)
            & (distances <= self.safety_distance)
        )

        return np.any(person_hits)