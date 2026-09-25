import utils.world as W
import utils.demo_robot as R
import utils.demo_person as P


# create the world object, which will set up most of pybullet
wld = W.BaseWorld()

# load a map
wld.map_from_file("maps/Lab1Map")

# you do not have to do this in this way, but I created a method to add textures to the walls
# be warned it interacts with the way I did the map and the storage of the map, so it only
# impacts walls of type 1
wld.texture_walls()


# create a robot, positioned at 0,0
r = R.DemoRobot(x=2, y=2)     # 9,9

# make the camera follow the robot
wld.follow_camera(r.agv_id)


# create a person
person = P.DemoPerson()


# ------------------------------------------------------------
# Person movement graph
# ------------------------------------------------------------
# The person will randomly move between these connected nodes.
active_worker_paths = P.ValidPathGraph().add_positions([
    (6, 2),
    (8, 2),
    (10, 2)
])

active_worker_paths.add_connection(
    (6, 2),
    (8, 2)
)

active_worker_paths.add_connection(
    (8, 2),
    (10, 2)
)

# Give the person the movement graph.
# set_movement_graph() will automatically choose one of the nodes
# as the initial position.
person.set_movement_graph(
    active_worker_paths
)


# ------------------------------------------------------------
# Robot-person interaction
# ------------------------------------------------------------

# Tell the robot which PyBullet object corresponds to the person.
# The LiDAR can then identify the person specifically.
r.set_person_target(
    person.objectId
)

# Allow the person object to detect physical collisions with the robot.
person.add_collidable(
    r.agv_id
)


# my world object has call backs for the step function and the keyboard
# I add only the robots step function here, but you could have 2 robots,
# or other special objects

# Update the person's position first.
# This means the LiDAR sees the person's most recent position
# when the robot performs its next step.
wld.add_step_callback(
    person.step_action
)

# Then update the robot.
wld.add_step_callback(
    r.step_action
)

# Keep the keyboard callback available for manual control/testing.
wld.add_keyboard_callback(
    r.user_control
)


# Run simulation
while True:
    wld.simStep()


# Disconnect
wld.end()