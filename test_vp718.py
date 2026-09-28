import pybullet as p
import utils.world as W
import utils.demo_robot as R
import utils.demo_person as P


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

# you do not have to do this in this way, but I created a method to add textures to the walls
# be warned it interacts with the way I did the map and the storage of the map, so it only
# impacts walls of type 1
wld.texture_walls()


# ------------------------------------------------------------
# Create the robot
# ------------------------------------------------------------

# This position works correctly with the new Lab 3 map.
r = R.DemoRobot(x=3, y=5)

# make the camera follow the robot
wld.follow_camera(r.agv_id)


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


# ------------------------------------------------------------
# Simulation callbacks
# ------------------------------------------------------------

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

# Then update the robot.
wld.add_step_callback(
    r.step_action
)

# Keep the keyboard callback available for manual control/testing.
wld.add_keyboard_callback(
    r.user_control
)

r.set_person_targets([
    person1.objectId,
    person2.objectId,
    person3.objectId
])

# The complete environment has now been created.
# Enable rendering so everything appears at once.
p.configureDebugVisualizer(
    p.COV_ENABLE_RENDERING,
    1
)

# Run simulation
while True:
    wld.simStep()


# Disconnect
wld.end()