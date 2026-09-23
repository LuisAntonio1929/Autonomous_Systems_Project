import pybullet as p

class DemoRobot:
    def __init__(self,x=0,y=0):
        body_length = 1.2
        body_width = 0.8
        body_height = 0.3
        wheel_radius = 0.15
        wheel_width = 0.1

        chassis_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[body_length/2, body_width/2, body_height/2])
        chassis_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[body_length/2, body_width/2, body_height/2],
                                        rgbaColor=[0.2, 0.4, 0.8, 1.0])
        
        wheel_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=wheel_radius, height=wheel_width)
        wheel_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=wheel_radius, length=wheel_width,
                                         rgbaColor=[0.3, 0.3, 0.3, 1.0])

        wheel_positions = [
            [body_length/3, body_width/2 + wheel_width/2, -wheel_width/2],    # Front right
            [body_length/3, -body_width/2 - wheel_width/2, -wheel_width/2],   # Front left
            [-body_length/3, body_width/2 + wheel_width/2, -wheel_width/2],   # Rear right
            [-body_length/3, -body_width/2 - wheel_width/2, -wheel_width/2],  # Rear left
        ]
# Create multi-body with wheels
        link_masses = [2.0] * 4  # 2kg per wheel
        link_collision_shapes = [wheel_collision] * 4
        link_visual_shapes = [wheel_visual] * 4
        link_positions = wheel_positions
        link_orientations = [p.getQuaternionFromEuler([1.57, 0, 0])] *4    # [[0, 0, 0, 1]] * 4
        link_inertial_frame_positions = [[0, 0, 0]] * 4
        link_inertial_frame_orientations = [[0, 0, 0, 1]] * 4
        link_parent_indices = [0] * 4  # All wheels attached to base
        link_joint_types = [p.JOINT_REVOLUTE] * 4
        link_joint_axis = [[0, 0, 1]] * 4  # Rotate around Z axis - though not quite sure why, trial and error
        
        self.agv_id = p.createMultiBody(
            baseMass=50,  # 50kg body
            baseCollisionShapeIndex=chassis_collision,
            baseVisualShapeIndex=chassis_visual,
            basePosition=[x, y, wheel_radius + body_height/2],
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


        # Get number of joints
        self.num_joints = p.getNumJoints(self.agv_id)
        self.wheel_joints = list(range(self.num_joints))  # All joints are wheels
        
        # Set wheel friction
        for joint_id in self.wheel_joints:
            p.setJointMotorControl2(
                bodyUniqueId=self.agv_id,
                jointIndex=joint_id,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=0,
                force=0  # Adjust based on your needs
            )
            p.changeDynamics(self.agv_id, joint_id, lateralFriction=1.0,rollingFriction=0.01)
        
        self.left_speed=0
        self.right_speed=0

    

    def user_control(self,keys):
        """Overwrite this to catch keyboard"""
        pass

    def step_action(self):
        """Overwrite this to change behaviour"""
        pass
