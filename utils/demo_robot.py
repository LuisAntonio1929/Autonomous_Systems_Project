import pybullet as p


class DemoRobot:
    def __init__(self, x=0, y=0):

        body_length = 1.2
        body_width = 0.8
        body_height = 0.3
        wheel_radius = 0.15
        wheel_width = 0.1

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
        # Wheels
        # ------------------------------------------------------------

        wheel_collision = p.createCollisionShape(
            p.GEOM_CYLINDER,
            radius=wheel_radius,
            height=wheel_width
        )

        wheel_visual = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=wheel_radius,
            length=wheel_width,
            rgbaColor=[0.3, 0.3, 0.3, 1.0]
        )

        wheel_positions = [
            [
                body_length / 3,
                body_width / 2 + wheel_width / 2,
                -wheel_width / 2
            ],  # Front right

            [
                body_length / 3,
                -body_width / 2 - wheel_width / 2,
                -wheel_width / 2
            ],  # Front left

            [
                -body_length / 3,
                body_width / 2 + wheel_width / 2,
                -wheel_width / 2
            ],  # Rear right

            [
                -body_length / 3,
                -body_width / 2 - wheel_width / 2,
                -wheel_width / 2
            ]   # Rear left
        ]

        # ------------------------------------------------------------
        # Multi-body configuration
        # ------------------------------------------------------------

        link_masses = [2.0] * 4

        link_collision_shapes = [wheel_collision] * 4
        link_visual_shapes = [wheel_visual] * 4

        link_positions = wheel_positions

        # Original orientation from the provided template
        link_orientations = [
            p.getQuaternionFromEuler([1.57, 0, 0])
        ] * 4

        link_inertial_frame_positions = [
            [0, 0, 0]
        ] * 4

        link_inertial_frame_orientations = [
            [0, 0, 0, 1]
        ] * 4

        link_parent_indices = [0] * 4

        link_joint_types = [
            p.JOINT_REVOLUTE
        ] * 4

        # Original joint axis from the provided template
        link_joint_axis = [
            [0, 0, 1]
        ] * 4

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
        # Wheel joints
        # ------------------------------------------------------------

        self.num_joints = p.getNumJoints(self.agv_id)

        self.wheel_joints = list(
            range(self.num_joints)
        )

        # Original wheel grouping
        self.right_wheels = [0, 2]
        self.left_wheels = [1, 3]

        # ------------------------------------------------------------
        # Wheel dynamics
        # ------------------------------------------------------------

        for joint_id in self.wheel_joints:

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
                lateralFriction=0.5,
                rollingFriction=0.001,
                spinningFriction=0.001
            )

        # ------------------------------------------------------------
        # Control parameters
        # ------------------------------------------------------------

        self.left_speed = 0.0
        self.right_speed = 0.0

        self.drive_speed = 6.0
        self.turn_speed = 3.0
        self.motor_force = 20.0


    def user_control(self, keys):
        """
        Manual control.

        W = forward
        S = backward
        A = turn counter-clockwise / left
        D = turn clockwise / right
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

        # Turn left / counter-clockwise
        if (
            ord('a') in keys
            and keys[ord('a')] & p.KEY_IS_DOWN
        ):
            angular -= 1.0

        # Turn right / clockwise
        if (
            ord('d') in keys
            and keys[ord('d')] & p.KEY_IS_DOWN
        ):
            angular += 1.0

        # Negative linear velocity is necessary because of the
        # wheel orientation used in the original template.
        self.left_speed = (
            -linear * self.drive_speed
            + angular * self.turn_speed
        )

        self.right_speed = (
            -linear * self.drive_speed
            - angular * self.turn_speed
        )


    def step_action(self):
        """Apply desired wheel speeds."""

        for joint_id in self.left_wheels:

            p.setJointMotorControl2(
                bodyUniqueId=self.agv_id,
                jointIndex=joint_id,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=self.left_speed,
                force=self.motor_force
            )

        for joint_id in self.right_wheels:

            p.setJointMotorControl2(
                bodyUniqueId=self.agv_id,
                jointIndex=joint_id,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=self.right_speed,
                force=self.motor_force
            )