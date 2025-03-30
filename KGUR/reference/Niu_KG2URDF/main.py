from urdf import BaseDoor, BaseDrawer, BaseWall, BaseObject
from yourdfpy import URDF, Link, Robot
from math import pi
from numpy.linalg import inv

ROBOT_URDF = URDF.load(
    "/home/yida/projects/vkc_exp/src/hello_moveit/urdf/env_output.xacro")


def create_drawer_env(inverse=False):
    robot = BaseObject("scene")

    drawer0 = BaseDrawer("drawer0")
    drawer0.create_world_joint([4, 2.5, 0.9], pi)
    if inverse:
        drawer0.inverse_root_tip("world", drawer0.name + "_handle_link")
    table0 = BaseWall("table0", 2.25, 3.25, 0.7)
    table0.create_world_joint([1.125, 2.5, 0])
    table1 = BaseWall("table1", 1, 6, 0.7)
    table1.create_world_joint([4.0, 2.5, 0])
    table2 = BaseWall("table2", 4.5, 1, 0.7)
    table2.create_world_joint([2.25, 6, 0])
    robot._merge_robot(drawer0, table0, table1, table2)

    joint = robot._joint_map["drawer0_world_joint"]
    joint.parent = "robotiq_arg2f_base_link"
    joint.origin = inv(drawer0._attach_location._make_joint().origin)

    return robot


def create_door_env(inverse=False):
    robot = BaseObject("scene")
    arena_x = 10.0
    arena_y = 10.0
    arena_z = 2.06

    wall_thickness = 0.1
    door_width = 1.1

    wall_north_left = BaseWall("wall_north_left", wall_thickness,
                               (arena_y - door_width) / 2.0, arena_z)
    wall_north_left.create_world_joint([
        arena_x / 2.0 - 2., (arena_y - door_width) / 4.0 + door_width / 2.0, 0
    ], 0)
    wall_north_right = BaseWall("wall_north_right", wall_thickness,
                                (arena_y - door_width - 0.1) / 2.0, arena_z)
    wall_north_right.create_world_joint([
        arena_x / 2.0 - 2., -((arena_y - door_width) / 4.0 + door_width / 2.0),
        0
    ], 0)
    wall_west = BaseWall("wall_west", arena_x / 2. + wall_thickness - 2., 4.,
                         arena_z)
    wall_west.create_world_joint([arena_x / 4. - 1., arena_y / 2.0 - 2., 0], 0)

    wall_east = BaseWall("wall_east", arena_x / 2. + wall_thickness - 2., 4.,
                         arena_z)
    wall_east.create_world_joint([arena_x / 4. - 1., -arena_y / 2.0 + 2., 0],
                                 0)
    wall_south = BaseWall("wall_south", wall_thickness, arena_y, arena_z)

    wall_south.create_world_joint([-arena_x / 2.0, 0, 0], 0)

    # inverse door first
    door0 = BaseDoor("door_north", door_width=door_width, dir="right")
    door0.create_world_joint([arena_x / 2.0 - 2., door_width / 2, 0.0], 0)
    if inverse:
        door0.inverse_root_tip("world", door0.name + "_handle_link")

    # attach door to robot
    robot._merge_robot(door0, ROBOT_URDF.robot)
    if inverse:
        joint = robot._joint_map["door_north_world_joint"]
        joint.parent = "robotiq_arg2f_base_link"
        joint.origin = inv(door0._attach_location._make_joint().origin)

    # inverse door & robot
    robot.inverse_root_tip("world", "door_north_base_link")

    robot._joint_map[
        "world"].origin = robot._make_transformation(
            [arena_x / 2.0 - 2., door_width / 2, 0.0])

    # fix door base joint
    robot._merge_robot(wall_north_left, wall_north_right, wall_west, wall_east,
                       wall_south)
    make_world_link(robot)
    return robot


def make_world_link(robot: Robot):
    for l in robot.links:
        if l.name == "world":
            return
    world_link = Link("world")
    robot.links.append(world_link)


def main():
    drawer_env = create_door_env(True)

    urdf = URDF(drawer_env)

    urdf.write_xml_file("door_robot_inverse_2.urdf")
    print("urdf saved")


if __name__ == "__main__":
    main()
