import pybullet as p
import os
import math
import random

class ValidPathGraph():
    def __init__(self):
        self.nodes=set()
        self.connections={}
    
    def add_position(self,x,y):
        self.nodes.add((x,y))

    def add_positions(self,xs):
        for x in xs:
            self.add_position(x[0],x[1])
        return self
    
    def add_connection(self,p1,p2):
        if p1 in self.nodes and p2 in self.nodes:
            if p1 not in self.connections:
                self.connections[p1]=[]
            if p2 not in self.connections:
                self.connections[p2]=[]
            self.connections[p1].append(p2)
            self.connections[p2].append(p1)    

    def random_target_from(self,x):
        print(self.connections)
        return random.sample(self.connections[x],1)[0]

class DemoPerson:
    def __init__(self,x=0,y=0):
        urdf_path = os.path.abspath("models/bubbleperson.urdf")
        self.objectId = p.loadURDF(urdf_path,[25,0,0.82])
        # ------------------------------------------------------------
        # Lock articulated joints
        # ------------------------------------------------------------
        # For Lab 3 the person is used only as a moving obstacle.
        # All movable joints are kept in their neutral position.

        for joint_id in range(
            p.getNumJoints(self.objectId)
        ):

            joint_info = p.getJointInfo(
                self.objectId,
                joint_id
            )

            joint_type = joint_info[2]

            # Ignore joints that are already fixed in the URDF.
            if joint_type != p.JOINT_FIXED:

                # Start the joint at its neutral position.
                p.resetJointState(
                    self.objectId,
                    joint_id,
                    targetValue=0.0
                )

                # Keep the joint fixed at zero while the simulation runs.
                p.setJointMotorControl2(
                    bodyUniqueId=self.objectId,
                    jointIndex=joint_id,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=0.0,
                    force=200.0
                )
        self.new_orn = p.getQuaternionFromEuler([0, 0, 0])

        self.collidable=set()

        self.collision_callback=None
        self.movements=None
        self.speed = 0.8
        self.sim_hz = 240

        self.current_location=(25,0)
        self.active_steps=[]

    def set_movement_graph(self,g):
        self.movements=g
        x=random.sample(list(self.movements.nodes),1)[0]
        self.set_position(x[0],x[1])
        # pick an initial location?

    def set_position(self,x,y):
        p.resetBasePositionAndOrientation(self.objectId, [x,y,0.82], self.new_orn)
        self.current_location=(x,y)

    def set_collision_callback(self,f):
        self.collision_callback=f

    def add_collidable(self,new_id):
        self.collidable.add(new_id)
        p.setCollisionFilterPair(new_id, self.objectId, -1, -1, enableCollision=0)

    def pick_new_path(self):
        targ = self.movements.random_target_from(
            self.current_location
        )

        dx = targ[0] - self.current_location[0]
        dy = targ[1] - self.current_location[1]

        dist = (dx * dx + dy * dy) ** 0.5

        # Distance travelled during one simulation step
        step_distance = self.speed / self.sim_hz

        # Number of simulation steps needed to reach the target
        steps = max(
            1,
            int(dist / step_distance)
        )

        self.active_steps = [
            (
                self.current_location[0] + (float(i) / steps) * dx,
                self.current_location[1] + (float(i) / steps) * dy
            )
            for i in range(1, steps)
        ]

        self.active_steps.append(targ)

        self.active_steps.reverse()
        
    def update_position(self):
        if len(self.active_steps)==0:
            self.pick_new_path()
        else:
            (x,y)=self.active_steps.pop()
            self.set_position(x,y)

    def step_action(self):

        for o in self.collidable:

            contact_points = p.getContactPoints(
                bodyA=o,
                bodyB=self.objectId
            )

            if len(contact_points) > 0:

                print("Collision detected!")

                if self.collision_callback is not None:
                    self.collision_callback()

        if self.movements is not None:

            self.update_position()

        # Keep the person's articulated body completely rigid.
        self.lock_joints()
         
    def lock_joints(self):
        """
        Keep all movable joints in their neutral position.
        """

        for joint_id in range(
            p.getNumJoints(self.objectId)
        ):

            joint_info = p.getJointInfo(
                self.objectId,
                joint_id
            )

            joint_type = joint_info[2]

            if joint_type != p.JOINT_FIXED:

                p.resetJointState(
                    self.objectId,
                    joint_id,
                    targetValue=0.0
                )

    def set_yaw(self, degrees):
        """
        Rotate the person around the Z axis.

        Args:
            degrees: Desired yaw angle in degrees.
        """

        yaw = math.radians(degrees)

        self.new_orn = p.getQuaternionFromEuler([
            0,
            0,
            yaw
        ])

        # Apply the new orientation immediately while
        # preserving the current position.
        position, _ = p.getBasePositionAndOrientation(
            self.objectId
        )

        p.resetBasePositionAndOrientation(
            self.objectId,
            position,
            self.new_orn
        )

    def set_position_on_path(self,start,end,progress=0.5,toward_end=True):
        """
        Start the person at an intermediate point of a straight path.

        Args:
            start:
                First endpoint of the path, e.g. (9.5, 2).

            end:
                Second endpoint of the path, e.g. (9.5, 8).

            progress:
                Position between the endpoints.

                0.0 = start
                0.5 = middle
                1.0 = end

            toward_end:
                True  -> initially walk toward end
                False -> initially walk toward start
        """

        # Clamp progress to the valid range.
        progress = max(
            0.0,
            min(1.0, progress)
        )

        # ------------------------------------------------------------
        # Calculate intermediate position
        # ------------------------------------------------------------

        x = (
            start[0]
            + progress * (end[0] - start[0])
        )

        y = (
            start[1]
            + progress * (end[1] - start[1])
        )

        # Place the person at this intermediate position.
        self.set_position(
            x,
            y
        )

        # ------------------------------------------------------------
        # Select the first destination
        # ------------------------------------------------------------

        if toward_end:
            target = end
        else:
            target = start

        dx = target[0] - x
        dy = target[1] - y

        distance = (
            dx * dx + dy * dy
        ) ** 0.5

        step_distance = (
            self.speed / self.sim_hz
        )

        steps = max(
            1,
            int(distance / step_distance)
        )

        # ------------------------------------------------------------
        # Generate the first partial path
        # ------------------------------------------------------------
        #
        # Once the person reaches the endpoint, current_location
        # becomes an actual graph node. From then on, the normal
        # A -> B -> A -> B movement works automatically.
        # ------------------------------------------------------------

        self.active_steps = [
            (
                x + (float(i) / steps) * dx,
                y + (float(i) / steps) * dy
            )
            for i in range(1, steps)
        ]

        # Make sure the final position is exactly the graph node,
        # avoiding floating-point equality problems.
        self.active_steps.append(
            target
        )

        # update_position() uses pop(), so reverse the list.
        self.active_steps.reverse()