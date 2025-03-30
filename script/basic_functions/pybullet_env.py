import os
import math 
import numpy as np
import time
import pybullet 
import random
from datetime import datetime
import pybullet_data

from collections import namedtuple
from attrdict import AttrDict

# ROBOT_URDF_PATH = "./ur_e_description/urdf/ur5e.urdf"
ROBOT_URDF_PATH = "/home/y/Documents/WYY_META/effectit/ros_kortex/kortex_description/arms/gen3/7dof/urdf/GEN3_URDF_V12_absPath.urdf"
ENV_URDF_PATH = "/home/y/Documents/WYY_META/effectit/VKC/src/vkc-planner-ros/vkc_deps/scene_builder/output/env_description/singleobj3_env/main.urdf"
TABLE_URDF_PATH = os.path.join(pybullet_data.getDataPath(), "table/table.urdf")

class KinovaPybulletSim():
    def __init__(self):
        pybullet.connect(pybullet.GUI)
        pybullet.setRealTimeSimulation(True)
        pybullet.setGravity(0,0,-9.8)
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.robot_urdf_path = ROBOT_URDF_PATH
        self.table_urdf_path = TABLE_URDF_PATH
        # self.cube_urdf_path = os.path.join(pybullet_data.getDataPath(),"toys/concave_box.urdf")
        self.cube_urdf_path = os.path.join(pybullet_data.getDataPath(),"lego/lego.urdf")
        # plane = pybullet.createCollisionShape(pybullet.GEOM_PLANE)
        planeID = pybullet.loadURDF("plane.urdf")
        self.end_effector_index = 8 #7DOF arm + 1
        self.robot = self.load_scene()
        self.num_joints = pybullet.getNumJoints(self.robot)
    
    def load_scene(self):
        flags = pybullet.URDF_USE_SELF_COLLISION
        table = pybullet.loadURDF(self.table_urdf_path, [0.5, 0, 0], [0, 0, 0, 1])
        concave_box = pybullet.loadURDF(self.cube_urdf_path, [0.5,0,0.64], [0,0,0,1])
        robot = pybullet.loadURDF(ROBOT_URDF_PATH, [0, 0, 0.63], [0, 0, 0, 1], flags=flags, useFixedBase=1)
        return robot

    def check_collisions(self):
        collisions = pybullet.getContactPoints()
        if len(collisions) > 0:
            print("[Collision detected!] {}".format(datetime.now()))
            # print(collisions)
            return True
        return False
    
    def test(self):
        print(pybullet.getJointInfo(self.robot, 0))


def launch():
    sim = KinovaPybulletSim()
    while True:
        sim.check_collisions()
        # sim.test()


if __name__ == "__main__":
    launch()
