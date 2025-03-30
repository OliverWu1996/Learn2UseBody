from __future__ import annotations

import logging
from copy import deepcopy
from typing import Dict, List

import networkx as nx
import numpy as np
import trimesh.transformations as tra
from numpy.linalg import inv
from yourdfpy.urdf import (URDF, Collision, Color, Inertial, Joint, Link,
                           Material, Robot, Visual)


class AttachLocation():
    joint: Joint

    def __init__(self, link_name, xyz, quat):
        self.link_name = link_name
        self.xyz = xyz
        self.quat = quat

    def _make_joint(self):
        joint = Joint(self.link_name + "_attach_joint",
                      type="fixed",
                      parent="NULL",
                      child=self.link_name,
                      origin=tra.compose_matrix(
                          translate=self.xyz,
                          angles=tra.euler_from_quaternion(self.quat)))
        return joint


class BaseObject(Robot):
    _link_map: Dict[str, Link] = {}
    _joint_map: Dict[str, Joint] = {}
    _attach_location: AttachLocation | None = None

    def __init__(self, name):
        super().__init__(name)
        self.logger = logging.getLogger(self.name)
        self.logger.setLevel(logging.DEBUG)
        ch = logging.StreamHandler()
        ch.setLevel(logging.DEBUG)
        self.logger.addHandler(ch)

        self.materials.append(
            Material(self.name + "_white",
                     color=Color(np.array([1.0, 1.0, 1.0, 1.0]))))

    @property
    def world_joint(self):
        for j in self.joints:
            if j.parent == "world":
                return j
        raise ValueError("No world joint found")

    def create_world_joint(
        self,
        xyz=[0, 0, 0],
        yaw=0.0,
        rpy: List[float] | None = None,
    ):
        if rpy is None:
            rpy = [0, 0, yaw]
        world_joint = Joint(self.name + "_world_joint",
                            type="fixed",
                            parent="world",
                            child=self.name + "_base_link",
                            origin=self._make_transformation(xyz, rpy))
        self.joints.append(world_joint)

    def inverse_root_tip(self, root_link: str, tip_link: str):
        """
        inverse root and tip link
        """
        if not self.world_joint:
            raise ValueError("No world joint found")
        self._update_maps()
        self._build_graph()
        path = self._get_path(root_link, tip_link)

        tip_root_transform = tra.compose_matrix()
        prev_old_joint_tf = tra.compose_matrix()
        prev_old_joint_tf_inv = tra.compose_matrix()
        for joint in path[::-1]:
            current_joint_origin = deepcopy(joint.origin)
            if joint.parent == "world":  # if world joint
                self.logger.debug("inverting world joint: %s", joint.name)
                tip_root_transform = self.world_joint.origin @ tip_root_transform
                childs = self.G.out_edges(joint.child)
                if not len(childs):
                    continue
                # reverse branch
                for child in childs:
                    joint_name = self.G[child[0]][child[1]][
                        "name"]  # get joint name from edge property
                    if joint_name not in map(lambda j: j.name, path):
                        self._joint_map[joint_name].origin = inv(
                            prev_old_joint_tf
                        ) @ self._joint_map[joint_name].origin
                continue
            self.logger.info("inverting: %s", joint.name)
            joint.parent, joint.child = joint.child, joint.parent
            tip_root_transform = joint.origin @ tip_root_transform

            if (joint.parent == tip_link):
                joint.origin = tra.compose_matrix()
            else:
                joint.origin = prev_old_joint_tf_inv

            # reverse branch
            childs = self.G.out_edges(joint.parent)
            if len(childs):
                for child in childs:
                    joint_name = self.G[child[0]][child[1]]["name"]
                    if joint_name not in map(lambda j: j.name, path):
                        print(joint_name)
                        self._joint_map[joint_name].origin = inv(
                            prev_old_joint_tf) @ (
                                self._joint_map[joint_name].origin
                                if self._joint_map[joint_name].origin
                                is not None else tra.compose_matrix())

            prev_old_joint_tf = current_joint_origin
            prev_old_joint_tf_inv = inv(prev_old_joint_tf)

            # get the link to be modified
            link = self._link_map[joint.child]

            if link.inertial:
                link.inertial.origin = prev_old_joint_tf_inv @ link.inertial.origin
            if len(link.visuals):
                for visual in link.visuals:
                    visual.origin = prev_old_joint_tf_inv @ (
                        visual.origin
                        if visual.origin is not None else tra.compose_matrix())
            if len(link.collisions):
                for collision in link.collisions:
                    collision.origin = prev_old_joint_tf_inv @ (
                        collision.origin if collision.origin is not None else
                        tra.compose_matrix())
            if joint.type in ["revolute", "prismatic"]:
                if joint.limit:
                    joint.limit.lower, joint.limit.upper = -joint.limit.upper, -joint.limit.lower
        self.world_joint.parent = "world"
        self.world_joint.child = tip_link
        self.world_joint.type = "fixed"
        self.world_joint.origin = tip_root_transform

    def _update_maps(self):
        self._joint_map = {}
        self._link_map = {}
        for j in self.joints:
            self._joint_map[j.name] = j
        for l in self.links:
            self._link_map[l.name] = l

    def _get_path(self, root_link: str, tip_link: str) -> List[Joint]:
        """
        use bfs to find joint path from root link to tip link
        """
        queue = [(root_link, [])]
        while queue:
            link, path = queue.pop(0)
            if link == tip_link:
                return path
            for joint in self.joints:
                if joint.parent == link:
                    queue.append((joint.child, path + [joint]))
        raise ValueError("No path found")

    def _make_inertia(self, ixx, iyy, izz):
        inertia = np.array(
            [
                [ixx, 0.0, 0.0],
                [0.0, iyy, 0.0],
                [0.0, 0.0, izz],
            ],
            dtype=np.float64,
        )
        return inertia

    def _make_transformation(self, xyz=[0, 0, 0], rpy=[0, 0, 0]):
        return tra.compose_matrix(translate=np.array(xyz),
                                  angles=np.array(rpy))

    def _make_collision(self, visuals: List[Visual]):
        v = visuals[0]
        if not v.geometry:
            raise ValueError("Visual must have geometry")

        collision = Collision(v.name if v.name else self.name + "_collision",
                              origin=v.origin,
                              geometry=v.geometry)
        return collision

    def _merge_robot(self, *others: Robot):
        for other in others:
            self.joints.extend(other.joints)
            self.links.extend(other.links)
            self.materials.extend(other.materials)
        self._update_maps()
        self._build_graph()

    def _build_graph(self):
        self.G = nx.DiGraph()
        for j in self.joints:
            self.G.add_edge(j.parent, j.child, name=j.name)

#help me write a main fucltion
if __name__ == "__main__":
    URDF.load('/home/y/Documents/WYY_META/effectit/VKC/src/vkc-planner-ros/vkc_deps/scene_builder/output/env_description/singleobj3_env/main.urdf')
    