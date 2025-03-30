from math import pi
from typing import Dict

import numpy as np
import trimesh.transformations as tra
from base import BaseObject, AttachLocation
from yourdfpy.urdf import (URDF, Box, Collision, Color, Cylinder, Dynamics,
                           Geometry, Inertial, Joint, Limit, Link, Material,
                           Robot, Visual)


class BaseDoor(BaseObject):
    mir = 0

    def __init__(self,
                 name,
                 door_height=2.06,
                 door_width=1.3,
                 handle_length=0.1,
                 handle_height=0.95,
                 dir="right"):
        super().__init__(name)
        if dir == "right":
            self.mir = 1
        elif dir == "left":
            self.mir = -1
        else:
            raise ValueError("dir must be 'right' or 'left'")

        base_link = Link(self.name + "_base_link")
        base_link_visual = Visual(self.name + "_visual")

        base_link_visual.origin = tra.compose_matrix(
            translate=np.array([0, 0, door_height / 2]),
            angles=np.array([0, 0, 0]),
        )

        base_link_visual.material = Material(self.name + "_color",
                                             color=Color(
                                                 np.array([0.4, 0.2, 0.0,
                                                           1.0])))
        base_link.visuals.append(base_link_visual)

        door_link = Link(self.name + "_door_link")
        door_link.inertial = Inertial(
            origin=tra.compose_matrix(translate=np.array(
                [0, -self.mir * door_width / 2.0, door_height / 2.0])),
            mass=3.0,
            inertia=np.array(
                [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 3.0],
                ],
                dtype=np.float64,
            ))
        door_link.visuals.append(
            Visual(self.name + "_door_visual",
                   origin=tra.compose_matrix(translate=np.array(
                       [0, -self.mir * door_width / 2.0, door_height / 2]),
                                             angles=np.array([0, 0, pi])),
                   geometry=Geometry(
                       box=Box(np.array([0.05, door_width, door_height]))),
                   material=Material(self.name + "_color",
                                     color=Color(np.array([0.4, 0.2, 0.0,
                                                           1.0])))))
        door_link.collisions.append(self._make_collision(door_link.visuals))

        handle_link = Link(self.name + "_handle_link")
        handle_link.inertial = Inertial(origin=self._make_transformation(
            [0, self.mir * (handle_length / 2.0), 0.0]),
                                        mass=0.3,
                                        inertia=self._make_inertia(
                                            0.01, 0.01, 0.01))
        handle_link.visuals.append(
            Visual(self.name + "_handle_visual",
                   origin=self._make_transformation(
                       [0, self.mir * (handle_length / 2.0), 0],
                       [pi/2, 0, 0]),
                   geometry=Geometry(cylinder=Cylinder(0.013, handle_length)),
                   material=Material(self.name + "_color",
                                     color=Color(np.array([0.9, 0.9, 0.9,
                                                           1.0])))))
        handle_link.collisions.append(self._make_collision(
            handle_link.visuals))

        door_joint = Joint(self.name + "_door_joint", type="revolute")
        door_joint.parent = base_link.name
        door_joint.child = door_link.name
        door_joint.origin = self._make_transformation()
        door_joint.axis = np.array([0, 0, 1])
        door_joint.limit = Limit(effort=1000,
                                 velocity=10,
                                 lower=-1.57,
                                 upper=1.57)
        door_joint.dynamics = Dynamics(damping=100, friction=0.0)

        handle_joint = Joint(
            self.name + "_handle_joint",
            type="fixed",
            parent=door_link.name,
            child=handle_link.name,
            origin=self._make_transformation(
                xyz=[-0.125, self.mir * (-door_width + 0.08), handle_height]))

        self.links = [base_link, door_link, handle_link]
        self.joints = [door_joint, handle_joint]

        self._attach_location = AttachLocation(handle_link.name,
                                               [-0.15, self.mir * 0.08, 0.0],
                                               [0.5, 0.5, 0.5, 0.5])


class BaseWall(BaseObject):

    def __init__(self, name, x, y, z, color=[0.5, 0.5, 0.5, 1.0]):
        super().__init__(name)
        base_link = Link(self.name + "_base_link")

        wall_link = Link(self.name + "_wall_link")
        wall_link.visuals.append(
            Visual(self.name + "_wall_visual",
                   origin=self._make_transformation(),
                   geometry=Geometry(box=Box(np.array([x, y, z]))),
                   material=Material(self.name + "_color",
                                     color=Color(np.array(color)))))
        wall_link.collisions.append(self._make_collision(wall_link.visuals))

        wall_joint = Joint(self.name + "_wall_joint",
                           type="fixed",
                           parent=base_link.name,
                           child=wall_link.name,
                           origin=self._make_transformation([0, 0, z / 2.0]))
        self.links = [base_link, wall_link]
        self.joints = [wall_joint]


class BaseDrawer(BaseObject):

    def __init__(self,
                 name,
                 drawer_height=0.3,
                 drawer_length=1.0,
                 drawer_width=0.6,
                 drawer_mass=0.1,
                 wall_thickness=0.02,
                 wall_mass=0.05,
                 gap=0.01):
        super().__init__(name)

        side_wall_height = drawer_height + 2 * gap
        side_wall_length = drawer_length + gap
        base_wall_width = drawer_width + 2 * gap + 2 * wall_thickness
        base_wall_length = drawer_length + gap

        back_wall_height = side_wall_height + 2 * wall_thickness
        back_wall_length = base_wall_width

        handle_gap = wall_thickness * 3
        handle_thickness = wall_thickness
        handle_span = drawer_width / 4.0

        base_link = Link(self.name + "_base_link")

        drawer_link = Link(self.name + "_drawer_link",
                           inertial=Inertial(
                               origin=self._make_transformation(),
                               mass=0.1,
                               inertia=self._make_inertia(
                                   1.0 / 2.0 * drawer_mass * drawer_width**2 +
                                   drawer_height**2, 0.01, 0.01)))  # TODO!!!
        drawer_link.visuals.append(
            Visual(
                self.name + "_drawer_visual",
                origin=self._make_transformation(),
                geometry=Geometry(box=Box(
                    np.array([drawer_length, drawer_width, drawer_height]))),
                material=Material(self.name + "_color",
                                  color=Color(np.array([0.6, 0.6, 0.6,
                                                        1.0])))))
        drawer_link.collisions.append(self._make_collision(
            drawer_link.visuals))

        left_wall = Link(self.name + "_left_wall_link",
                         inertial=Inertial(
                             origin=self._make_transformation(),
                             mass=wall_mass,
                             inertia=self._make_inertia(
                                 1.0 / 2.0 * wall_mass * wall_thickness**2 +
                                 side_wall_height**2,
                                 1.0 / 2.0 * wall_mass * side_wall_length**2 +
                                 side_wall_height**2,
                                 1.0 / 2.0 * wall_mass * side_wall_length**2 +
                                 wall_thickness**2)))
        left_wall.visuals.append(
            Visual(self.name + "_left_wall_visual",
                   geometry=Geometry(box=Box(
                       np.array([
                           side_wall_length, wall_thickness, side_wall_height
                       ]))),
                   material=Material(self.name + "_color",
                                     color=Color(np.array([1.0, 1.0, 1.0,
                                                           1.0])))))

        right_wall = Link(self.name + "_right_wall_link",
                          inertial=Inertial(
                              mass=wall_mass,
                              inertia=self._make_inertia(
                                  1.0 / 2.0 * wall_mass * wall_thickness**2 +
                                  side_wall_height**2,
                                  1.0 / 2.0 * wall_mass * side_wall_length**2 +
                                  side_wall_height**2,
                                  1.0 / 2.0 * wall_mass * side_wall_length**2 +
                                  wall_thickness**2)))
        right_wall.visuals.append(
            Visual(self.name + "_right_wall_visual",
                   geometry=Geometry(box=Box(
                       np.array([
                           side_wall_length, wall_thickness, side_wall_height
                       ]))),
                   material=Material(self.name + "_color",
                                     color=Color(np.array([1, 1, 1, 1.0])))))

        bottom_wall = Link(
            self.name + "_bottom_wall_link",
            inertial=Inertial(mass=wall_mass,
                              inertia=self._make_inertia(
                                  1.0 / 2.0 * wall_mass * wall_thickness**2 +
                                  base_wall_length**2,
                                  1.0 / 2.0 * wall_mass * base_wall_width**2 +
                                  wall_thickness**2,
                                  1.0 / 2.0 * wall_mass * base_wall_length**2 +
                                  base_wall_width**2)))
        bottom_wall.visuals.append(
            Visual(self.name + "_bottom_wall_visual",
                   geometry=Geometry(box=Box(
                       np.array([
                           base_wall_length, base_wall_width, wall_thickness
                       ])))))
        bottom_wall.collisions.append(self._make_collision(
            bottom_wall.visuals))

        top_wall = Link(self.name + "_top_wall_link",
                        inertial=Inertial(
                            mass=wall_mass,
                            inertia=self._make_inertia(
                                1.0 / 2.0 * wall_mass * base_wall_width**2 +
                                base_wall_length**2,
                                1.0 / 2.0 * wall_mass * base_wall_width**2 +
                                wall_thickness**2,
                                1.0 / 2.0 * wall_mass * wall_thickness**2 +
                                base_wall_length**2)))
        top_wall.visuals.append(
            Visual(self.name + "_top_wall_visual",
                   geometry=Geometry(box=Box(
                       np.array([
                           base_wall_length, base_wall_width, wall_thickness
                       ])))))

        back_wall = Link(self.name + "_back_wall_link",
                         inertial=Inertial(
                             mass=wall_mass,
                             inertia=self._make_inertia(
                                 1.0 / 2.0 * wall_mass * back_wall_length**2 +
                                 back_wall_height**2,
                                 1.0 / 2.0 * wall_mass * back_wall_height**2 +
                                 wall_thickness**2,
                                 1.0 / 2.0 * wall_mass * back_wall_length**2 +
                                 back_wall_length**2)))
        back_wall.visuals.append(
            Visual(self.name + "_back_wall_visual",
                   geometry=Geometry(box=Box(
                       np.array([
                           wall_thickness, back_wall_length, back_wall_height
                       ]))),
                   material=Material(self.name + "_white")))

        handle_left = Link(
            self.name + "_handle_left_link",
            inertial=Inertial(mass=wall_mass,
                              inertia=self._make_inertia(
                                  1.0 / 2.0 * wall_mass * handle_thickness**2 +
                                  handle_thickness**2,
                                  1.0 / 2.0 * wall_mass * handle_thickness**2 +
                                  handle_gap**2,
                                  1.0 / 2.0 * wall_mass * handle_thickness**2 +
                                  handle_gap**2)))
        handle_left.visuals.append(
            Visual(self.name + "_handle_left_visual",
                   geometry=Geometry(box=Box(
                       np.array(
                           [handle_gap, handle_thickness, handle_thickness]))),
                   material=Material(self.name + "_white")))

        handle_right = Link(
            self.name + "_handle_right_link",
            inertial=Inertial(mass=wall_mass,
                              inertia=self._make_inertia(
                                  1.0 / 2.0 * wall_mass * handle_thickness**2 +
                                  handle_thickness**2,
                                  1.0 / 2.0 * wall_mass * handle_thickness**2 +
                                  handle_gap**2,
                                  1.0 / 2.0 * wall_mass * handle_thickness**2 +
                                  handle_gap**2)))
        handle_right.visuals.append(
            Visual(self.name + "_handle_right_visual",
                   geometry=Geometry(box=Box(
                       np.array(
                           [handle_gap, handle_thickness, handle_thickness]))),
                   material=Material(self.name + "_white")))

        handle_link = Link(
            self.name + "_handle_link",
            inertial=Inertial(mass=wall_mass,
                              inertia=self._make_inertia(
                                  1.0 / 2.0 * wall_mass * handle_thickness**2 +
                                  (handle_span + 2 * handle_thickness)**2,
                                  1.0 / 2.0 * wall_mass * handle_thickness**2 +
                                  handle_thickness**2,
                                  1.0 / 2.0 * wall_mass * handle_thickness**2 +
                                  handle_thickness**2)))
        handle_link.visuals.append(
            Visual(self.name + "_handle_link_visual",
                   geometry=Geometry(box=Box(
                       np.array([
                           handle_thickness, handle_span +
                           2 * handle_thickness, handle_thickness
                       ]))),
                   material=Material(self.name + "_white")))

        base_drawer_joint = Joint(self.name + "_base_drawer_joint",
                                  parent=base_link.name,
                                  child=drawer_link.name,
                                  origin=self._make_transformation(),
                                  limit=Limit(upper=0.8,
                                              lower=-0.8,
                                              effort=1000,
                                              velocity=10.0),
                                  dynamics=Dynamics(damping=100.0,
                                                    friction=0.0),
                                  axis=np.array([1, 0, 0]),
                                  type="prismatic")
        base_left_wall_joint = Joint(
            self.name + "_base_left_wall_joint",
            parent=base_link.name,
            child=left_wall.name,
            origin=self._make_transformation(
                [-gap, -(drawer_width / 2 + wall_thickness / 2) - gap, 0]),
            type="fixed")
        base_right_wall_joint = Joint(
            self.name + "_base_right_wall_joint",
            parent=base_link.name,
            child=right_wall.name,
            origin=self._make_transformation(
                [gap, drawer_width / 2 + wall_thickness / 2 + gap, 0]),
            type="fixed")
        base_bottom_wall_joint = Joint(
            self.name + "_base_bottom_wall_joint",
            parent=base_link.name,
            child=bottom_wall.name,
            origin=self._make_transformation(
                [-gap, 0, -(drawer_height / 2 - wall_thickness / 2) - gap]),
            type="fixed")
        base_top_wall_joint = Joint(
            self.name + "_base_top_wall_joint",
            parent=base_link.name,
            child=top_wall.name,
            origin=self._make_transformation(
                [-gap, 0, drawer_height / 2 + wall_thickness / 2 + gap]),
            type="fixed")
        base_back_wall_joint = Joint(
            self.name + "_base_back_wall_joint",
            parent=base_link.name,
            child=back_wall.name,
            origin=self._make_transformation(
                [-(drawer_length / 2 + gap + wall_thickness / 2), 0, 0]),
            type="fixed")
        drawer_handle_left_joint = Joint(
            self.name + "_drawer_handle_left_joint",
            parent=drawer_link.name,
            child=handle_left.name,
            origin=self._make_transformation([
                drawer_length / 2 + handle_thickness / 2,
                -handle_span / 2 - handle_thickness / 2, 0
            ]),
            type="fixed")
        drawer_handle_right_joint = Joint(
            self.name + "_drawer_handle_right_joint",
            parent=drawer_link.name,
            child=handle_right.name,
            origin=self._make_transformation([
                drawer_length / 2 + handle_gap / 2,
                handle_span / 2 + handle_thickness / 2, 0
            ]),
            type="fixed")
        drawer_handle_joint = Joint(
            self.name + "_drawer_handle_joint",
            parent=drawer_link.name,
            child=handle_link.name,
            origin=self._make_transformation(
                [drawer_length / 2 + handle_thickness / 2, 0, 0]),
            type="fixed")
        self.links = [
            base_link, drawer_link, left_wall, right_wall, bottom_wall,
            top_wall, back_wall, handle_link, handle_left, handle_right
        ]
        self.joints = [
            base_drawer_joint, base_left_wall_joint, base_right_wall_joint,
            base_bottom_wall_joint, base_top_wall_joint, base_back_wall_joint,
            drawer_handle_left_joint, drawer_handle_right_joint,
            drawer_handle_joint
        ]

        self._attach_location = AttachLocation(handle_link.name,
                                               [0.2, 0.0, 0.0],
                                               [0.5, 0.5, -0.5, -0.5])
