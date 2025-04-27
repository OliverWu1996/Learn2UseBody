import yourdfpy 
import networkx as nx
from networkx import DiGraph
import matplotlib.pyplot as plt
#help me import OrderedDict
from collections import OrderedDict
from yourdfpy.urdf import (URDF, Collision, Color, Inertial, Joint, Link,
                            Material, Robot, Visual, Limit, SafetyController)   
import numpy as np
import urdf_validation as UValid
import logging
import os
#help me import deepcpoy
from copy import deepcopy
import trimesh.transformations as tra
from numpy.linalg import inv
#import list
from typing import Dict, List
import ikpy.chain
import ikpy.utils.plot as plot_utils



ORI_SCENE_URDF_PATH = "/home/oliver/Documents/Projects/NonPrehensile/Planning-on-VKC-feature-household-env/src/env_description/singleobj3_env/main.urdf"
ORI_ROBOT_URDF_PATH = "/home/oliver/Documents/Projects/NonPrehensile/Planning-on-VKC-feature-household-env/src/robot_description/ur_description/urdf/ur5_generated.urdf"
AESTRUCTURE_URDF_PATH = os.path.join(os.getcwd(),"resource", "urdf", "AEstructure_test01.urdf")
MR_URDF_PATH = os.path.join(os.getcwd(),"resource", "urdf", "MainRobot_test01.urdf")
URDF_RESOURCE_PATH = os.path.join(os.getcwd(),"resource", "urdf")

# ORI_NIU_ROBOT = "/home/y/Documents/WYY_META/Planning-on-VKC/src/vkc_example/env/vkc_demo.urdf"
# ORI_NIU_SCENE = "/home/y/Documents/WYY_META/SR_exp/effectit/VKC/src/env_description/household_env/main.urdf"

DEFAULT_EFFORT = 1
DEFAULT_VELOCITY = 1


#initialize a class named UR2KG
#input: scene_urdf_path, robot_urdf_path
#input: vkc(robot,push,object_goal)
#output: new_scene_urdf, new_robot_urdf, robot_goal

#example:(kinova_gen3, push, ("closet_bottom_right_door_joint", 1.57))

def yourdfpy_init_URDF(urdf_name):
        Robot_init = Robot(urdf_name)
        Robot_init.materials = []
        Robot_init.joints = []
        Robot_init.links = []
        Robot_init.actuated_joint_indices = []
        Robot_init.actuated_joints = []
        urdf = yourdfpy.URDF(Robot_init)
        return urdf

class UR2KG():
    def __init__(self, scene_urdf_path, robot_urdf_path):
        ValidateURDFs = UValid.URDFValidation(ORI_SCENE_URDF_PATH, ORI_ROBOT_URDF_PATH)
        ValidateURDFs.validate_robot_urdf()
        ValidateURDFs.validate_env_urdf()

        self.scene_urdf_path = scene_urdf_path
        self.robot_urdf_path = robot_urdf_path
        self.scene_urdf = yourdfpy.URDF.load(self.scene_urdf_path)
        self.robot_urdf = yourdfpy.URDF.load(self.robot_urdf_path)
        # self.scene_urdf.show()
        # self.robot_urdf.show()



class KinematicsGraph():
    #basic structure of a kinematics graph:
        #world_frame
        #nodes
            #node_id
            #node_type
            #node_name
            #node_urdf
        #edges
            #edge_id 
            #edge_type
            #edge_name
            #edge_from
            #edge_to
            #relationship(transformation matrix)


    def __init__(self, world_frame='world') -> None:
        self.world = world_frame
        # self.G = nx.DiGraph()
        # self.G.add_node(self.world, node_type='frame', node_name=self.world, node_urdf=None) #urdf required?

class VKChain():
    def __init__(self, urdf, link_names=[], joint_names=[]):
        self.urdf = urdf
        self.link_names = link_names
        if joint_names == []:
            self.fill_in_joint_names()
        else:
            self.joint_names = joint_names
        self.joints = []
        self.links = []
        for joint_ in self.joint_names:
            for joint__ in self.urdf.robot.joints:
                if joint_ == joint__.name:
                    self.joints.append(deepcopy(joint__))
                    break
        for link_ in self.link_names:
            for link__ in self.urdf.robot.links:
                if link_ == link__.name:
                    self.links.append(deepcopy(link__))
                    break
        
        print("VKC links:", self.link_names)
        print("VKC joints:", self.joint_names)
        # if not self.validate(self._build_graph()):
        #     raise Exception("VKChain validation failed!")
    

    def get_VKChain_urdf(self, name="VKChain"):
        reference_urdf = self.urdf
        self.VKChain_urdf = yourdfpy_init_URDF(name)
        #add links into VKChain_urdf
        for link in self.link_names:
            #find link in reference_urdf
            for link_ in reference_urdf.robot.links:
                if link_.name == link:
                    self.VKChain_urdf.robot.links.append(link_)
                    break

        #add joints into VKChain_urdf
        for joint in self.joint_names:
            #find joint in reference_urdf
            for joint_ in reference_urdf.robot.joints:
                if joint_.name == joint:
                    self.VKChain_urdf.robot.joints.append(joint_)
                    break

        save_path = os.path.join(URDF_RESOURCE_PATH, name+".urdf")
        self.VKChain_urdf.write_xml_file(save_path)
        return self.VKChain_urdf, save_path
        


    def validate(self, nxDiGraph):
        
        #validate link order 
        reference_link_names = [self.urdf.robot.links[i].name for i in range(len(self.urdf.robot.links))]
        for link in self.link_names:
            if link not in reference_link_names:
                print("link %s not in reference_link_names" % link)
                return False
        #check link order
        for i in range(len(self.link_names)):
            if i == 0:
                continue
            else:
                successors = [succ for succ in nxDiGraph.successors(self.link_names[i-1])]
                if self.link_names[i] not in successors:
                    print("link order wrong")
                    return False
            


        
        #validate joint order
        reference_joint_names = [self.urdf.robot.joints[i].name for i in range(len(self.urdf.robot.joints))]
        for joint in self.joint_names:
            if joint not in reference_joint_names:
                print("joint %s not in reference_joint_names" % joint)
                return False
        for i in range(len(self.joint_names)):
            if i == 0:
                continue
            else:
                joint_successors = self.downstream_joints_of_joint(self.joint_names[i-1], nxDiGraph)
                if self.joint_names[i] not in joint_successors:
                    print("joint order wrong")
                    return False
        #validate link-joint relationship
        if self.link_names[0] != self.joints[0].parent:
            print("link-joint relationships are not correct")
            return False
        for i in range(len(self.link_names)-1):
            if self.link_names[i+1] != self.joints[i].child:
                print("link-joint relationships are not correct")
                return False
        
        return True
    def downstream_joints_of_joint(self, joint_name, nxDiGraph):
        #find the downstream joints of a joint
        for joint in self.urdf.robot.joints:
            if joint.name == joint_name:
                break
        successor_links = [succ for succ in nxDiGraph.successors(joint.child)]
        successor_joints = []
        for link in successor_links:
            for joint_ in self.urdf.robot.joints:
                if joint_.child == link:
                    successor_joints.append(joint_.name)
        return successor_joints
            
                
    def fill_in_joint_names(self,verbose=False):
        self.joint_names = []
        for i in range(len(self.link_names)):
            if i == 0:
                continue
            else:
                for joint in self.urdf.robot.joints:
                    if joint.parent == self.link_names[i-1] and joint.child == self.link_names[i]:
                        self.joint_names.append(joint.name)
                        if verbose:
                            print(joint)
                        break
        return self.joint_names
    def reverse_chain(self):
        reversed_link_names = deepcopy(self.link_names)
        reversed_joint_names = deepcopy(self.joint_names)
        reversed_link_names.reverse()
        reversed_joint_names.reverse()
        return reversed_link_names, reversed_joint_names
    
    def get_reversed_chain_urdf(self, reference_urdf, name='reversed_chain'):
        '''
        get reversed chain urdf
        input: reference_urdf
        output: reversed_chain_urdf, the urdf of the rest of the urdf
        WYY need testing!!
        '''
        reversed_link_names, reversed_joint_names = self.reverse_chain()
        #form urdf based on this reversed chain
        reversed_chain_urdf = yourdfpy_init_URDF(name)
        #add links into reversed_chain_urdf
        #not right, need to change origins
        for link in reversed_link_names:
            for link_ in reference_urdf.robot.links:
                if link_.name == link:
                    reversed_chain_urdf.robot.links.append(link_)
                    break
        #add joints into reversed_chain_urdf
        for joint in reversed_joint_names:
            for joint_ in reference_urdf.robot.joints:
                if joint_.name == joint:
                    reversed_chain_urdf.robot.joints.append(joint_)
                    break
        
        #modify link origins
        print(reversed_chain_urdf.robot.links[0])
        print(reversed_chain_urdf.robot.joints[0])
        for i in range(len(reversed_chain_urdf.robot.links)):
            pass
        #Abandoned function
    def _update_maps(self):
        self._joint_map = {}
        self._link_map = {}
        for j in self.joints:
            self._joint_map[j.name] = j
        for l in self.links:
            self._link_map[l.name] = l
    
    def _build_graph(self, verbose = False):
        self.G = nx.DiGraph()
        for j in self.joints:
            self.G.add_edge(j.parent, j.child, joint=j.name)
        if verbose:
            nx.draw_networkx(self.G, with_labels=True, font_weight='bold')
            plt.show()
        return self.G
    def _reversed_graph(self, verbose=True):
        self.G_r = nx.DiGraph()
        for j in self.joints_reversed:
            self.G_r.add_edge(j.parent, j.child, joint=j.name)
        if verbose:
            nx.draw_networkx(self.G_r, with_labels=True, font_weight='bold')
            plt.show()
        return self.G_r

    def _get_path(self, root_link: str, tip_link: str) -> List[Joint]:
        """
        use bfs to find joint path from root link to tip link
        """
        queue = [(root_link, [])]
        while queue:
            print(queue)
            link, path = queue.pop(0)
            if link == tip_link:
                return path
            print('selfjoints in get path', self.joints)
            # print(root_link, tip_link)
            for joint in self.joints:
                if joint.parent == link:
                    queue.append((joint.child, path + [joint]))
        raise ValueError("No path found")
    
    def reverse_root_tip(self, root_link: str, tip_link: str):
        '''
        reverse root_tip chain
        '''
        #check the existence of world link first
        self.joints = deepcopy(self.urdf.robot.joints)
        if 'world' not in self.link_names:
            raise ValueError("world link not in link list")
        else:
            for link_ in self.links:
                if link_.name == 'world':
                    world_link = link_
                    break
            for joint_ in self.joints:
                if joint_.parent == 'world':
                    self.world_joint = joint_
                    break
        self._update_maps()
        # print(self._joint_map)
        # print(self._link_map)
        self._build_graph()
        print(root_link, tip_link)
        print('self.joints before get path',self.joints)
        path = self._get_path(root_link, tip_link)
        self.links_reversed = []
        mid = deepcopy(self.joints)
        self.joint_reversed = []
        tip_root_transform = tra.compose_matrix()
        prev_old_joint_tf = tra.compose_matrix()
        prev_old_joint_tf_inv = tra.compose_matrix()

        for joint in path[::-1]:
            current_joint_origin = deepcopy(joint.origin)
            if joint.parent == "world":
                #invert world joint
                tip_root_transform = self.world_joint.origin @ tip_root_transform
                childs = self.G.out_edges(joint.child)
                if not len(childs):
                    continue
                for child in childs:
                    joint_name = self.G[child[0]][child[1]]['joint']
                    if joint_name not in map(lambda j: j.name, path):
                        self._joint_map[joint_name].origin = inv(
                            prev_old_joint_tf
                        ) @ self._joint_map[joint_name].origin
                continue
            print("inverting joint %s" % joint.name)
            joint.parent, joint.child = joint.child, joint.parent
            tip_root_transform = joint.origin @ tip_root_transform

            if (joint.parent == tip_link):
                joint.origin = tra.compose_matrix()
            else:
                joint.origin = prev_old_joint_tf_inv

            #reverse branch
            childs = self.G.out_edges(joint.parent)
            if len(childs):
                for child in childs:
                    joint_name = self.G[child[0]][child[1]]['joint']
                    if joint_name not in map(lambda j: j.name, path):
                        self._joint_map[joint_name].origin = inv(
                            prev_old_joint_tf) @ (
                            self._joint_map[joint_name].origin
                            if self._joint_map[joint_name].origin is not None
                            else tra.compose_matrix())
            prev_old_joint_tf = current_joint_origin
            prev_old_joint_tf_inv = inv(prev_old_joint_tf)

            #get the link to be modified
            link = self._link_map[joint.child]

            if link.inertial:
                link.inertial.origin = prev_old_joint_tf_inv @ link.inertial.origin
            if len(link.visuals):
                for visual in link.visuals:
                    visual.origin = prev_old_joint_tf_inv @ (
                        visual.origin if visual.origin is not None else tra.compose_matrix())
            if len(link.collisions):
                for collision in link.collisions:
                    collision.origin = prev_old_joint_tf_inv @ (
                        collision.origin if collision.origin is not None else tra.compose_matrix())
            self.links_reversed.append(link)
            if joint.type in ['revolute', 'prismatic']:
                if joint.limit:
                    joint.limit.lower, joint.limit.upper = -joint.limit.upper, -joint.limit.lower
        self.world_joint.parent = "world"
        self.world_joint.child = tip_link
        self.world_joint.type = "fixed"
        self.world_joint.origin = tip_root_transform
        
        self.joints_reversed = self.joints
        self.joints = mid
        # print(self._joint_map)
        # print(self._link_map)
        # self._reversed_graph()
        # for joint in self.joints:
        #     print('joint %s,parent %s, child %s' % (joint.name, joint.parent, joint.child))
        for joint in self.joints_reversed:
            print('reversed joint %s,parent %s, child %s' % (joint.name, joint.parent, joint.child))
        

    def get_reversed_VKCs_urdf(self,name):
        affordance2root_URDF = yourdfpy_init_URDF(name)
        save_path = os.path.join(URDF_RESOURCE_PATH, name+'.urdf')
        for link in self.links_reversed:
            affordance2root_URDF.robot.links.append(link)
        for joint in self.joints_reversed:
            affordance2root_URDF.robot.joints.append(joint)
        affordance2root_URDF.write_xml_file(save_path)
        
        return affordance2root_URDF
        

        
        

        
    
    def test(self):
        self.get_VKChain_urdf()
        
    
    

                    
class PushIt():
    def __init__(self) -> None:
        pass
    def get_env_robot_urdf(self, scene_urdf_path, robot_urdf_path, 
                           robot_base = tra.compose_matrix(), robot_name='', scene_name = '',
                           verbose_level=logging.WARNING):
        self.scene_urdf_path = scene_urdf_path
        self.robot_urdf_path = robot_urdf_path
        self.AES_urdf_path = AESTRUCTURE_URDF_PATH
        #Validate before loading
        ValidateURDFs = UValid.URDFValidation(scene_urdf_path, robot_urdf_path, logging_level=verbose_level)
        ValidateURDFs.validate_robot_urdf()
        ValidateURDFs.validate_env_urdf()
        self.scene_urdf = yourdfpy.URDF.load(self.scene_urdf_path)
        self.robot_urdf = yourdfpy.URDF.load(self.robot_urdf_path)
        #add world link to the first of the link list
        if self.robot_urdf.robot.links[0].name != 'world':
            self.robot_urdf.robot.links.insert(0,Link(name='world'))
            self.robot_urdf.robot.joints.insert(0,Joint(name=robot_name+'_world2base_joint', type='fixed',parent='world', 
                                                    child=self.robot_urdf.robot.links[1].name, origin=robot_base))
        self.robot_base_link = self.robot_urdf.robot.links[1].name
        
        # self.scene_urdf.show()
        # self.robot_urdf.show()
        # path = os.path.join(URDF_RESOURCE_PATH, 'modified_robot.urdf')
        # self.robot_urdf.write_xml_file(path)
    def get_env_robot_nxGraph(self):
        self.scene_nxGraph = self.build_graph(self.scene_urdf)
        self.robot_nxGraph = self.build_graph(self.robot_urdf)
        return self.scene_nxGraph, self.robot_nxGraph
        
        

    def generate_affordance(self, verbose=False):
        '''
        input: links -> list
        return: affordances -> list
        '''
        links = self.scene_urdf.robot.links
        if verbose == True:
            for link in links:
                print(link.name)
        
        affordance = 'microwave_handle'
        return affordance
    
    def generate_effector(self,verbose = False):
        '''
        input: robot_urdf -> URDF
        return: effector -> list
        '''
        links = self.robot_urdf.robot.links
        if verbose == True:
            for link in links:
                print(link.name)
        effector = 'ForeArm_Link_effector_1'
        return effector
    
    def generate_AEjoint(self, verbose=False):
        '''
        input: robot_urdf -> URDF
        return: AEjoint -> URDF
        '''
        pass

    def yourdfpy_init_URDF(self, urdf_name):
        Robot_init = Robot(urdf_name)
        Robot_init.materials = []
        Robot_init.joints = []
        Robot_init.links = []
        Robot_init.actuated_joint_indices = []
        Robot_init.actuated_joints = []
        urdf = yourdfpy.URDF(Robot_init)
        return urdf
    def niu_s(self, effector, affordance,
                slide_tolerence_x=0.01, slide_tolerence_y = 0.01,
                                  tilt_tolerence_r = 0.5235, verbose=False):
        
        '''
        input: robot_urdf -> URDF
                affordance -> str
                effector -> str
        return: AEjoint -> URDF
        '''
        #help me initialize a new urdf using yourdfpy
        Robot_init = Robot('AESstructure')
        Robot_init.materials = []
        Robot_init.joints = []
        Robot_init.links = []
        Robot_init.actuated_joint_indices = []
        Robot_init.actuated_joints = []
        AEStructure = yourdfpy.URDF(Robot_init)
        slide_x_link = 'slide_x_link'
        slide_y_link = 'slide_y_link'
        tilt_y_link = 'tilt_y_link'
        tilt_r_link = 'tilt_r_link'
        slide_x_joint = 'slide_x_joint'
        slide_y_joint = 'slide_y_joint'
        tilt_y_joint = 'tilt_y_joint'
        tilt_r_joint = 'tilt_r_joint'


        AEStructure.robot.joints = []
        AEStructure.robot.links = []
        #find effector link in robot urdf
        effecor_Link = None
        for link in self.robot_urdf.robot.links:
            if link.name == effector:
                effecor_Link = link
                break

        #find affordance link in scene urdf
        affordance_Link = None
        for link in self.scene_urdf.robot.links:
            if link.name == affordance:
                affordance_Link = link
                break
        attachment_trans = tra.compose_matrix(translate = [0,-0.35,0.35], angles = [-3.14,0,-3.14])
        AEStructure.robot.links.append(effecor_Link)
        # AEStructure.robot.links.append(Link(name=slide_x_link))
        # AEStructure.robot.links.append(Link(name=slide_y_link))
        # AEStructure.robot.links.append(Link(name=tilt_y_link))
        # AEStructure.robot.links.append(Link(name=tilt_r_link))
        AEStructure.robot.links.append(affordance_Link)
        # AEStructure.robot.joints.append(Joint(name=slide_x_joint, type='prismatic', 
        #                                                 parent=effector, child=slide_x_link, 
        #                                                 axis=(1,0,0), limit=Limit(lower=-slide_tolerence_x, upper=slide_tolerence_x, 
        #                                                                           effort=DEFAULT_EFFORT, velocity=DEFAULT_VELOCITY)))
        # AEStructure.robot.joints.append(Joint(name=slide_y_joint, type='prismatic', 
        #                                                 parent=slide_x_link, child=slide_y_link, 
        #                                                 axis=(0,1,0), limit=Limit(lower=-slide_tolerence_y, upper=slide_tolerence_y,
        #                                                                           effort=DEFAULT_EFFORT, velocity=DEFAULT_VELOCITY)))
        # AEStructure.robot.joints.append(Joint(name=tilt_y_joint, type='revolute',
        #                                                 parent=slide_y_link, child=tilt_y_link,
        #                                                 axis=(0,0,1), limit=Limit(lower=-3.14159265, upper=3.14159265,
        #                                                                           effort=DEFAULT_EFFORT, velocity=DEFAULT_VELOCITY)))
        # AEStructure.robot.joints.append(Joint(name=tilt_r_joint, type='revolute',
        #                                                 parent=tilt_y_link, child=tilt_r_link,
        #                                                 axis=(1,0,0), limit=Limit(lower=-tilt_tolerence_r, upper=tilt_tolerence_r,
        #                                                                           effort=DEFAULT_EFFORT, velocity=DEFAULT_VELOCITY)))
        AEStructure.robot.joints.append(Joint(name=affordance, type='fixed', origin=attachment_trans,
                                                        parent=effector, child=affordance))
        self.build_graph(AEStructure, verbose=verbose)
        #save AEStructure to a urdf file
        AEStructure.write_xml_file(AESTRUCTURE_URDF_PATH)
        # AEStructure.robots(AESTRUCTURE_URDF_PATH)
        
        return AEStructure
        
    def generate_EAStruct_handmade(self, effector, affordance, name = 'AEStructure',
                                  slide_tolerence_x=0.01, slide_tolerence_y = 0.01,
                                  tilt_tolerence_r = 0.5235, verbose=False):
        '''
        input: robot_urdf -> URDF
                affordance -> str
                effector -> str
        return: AEjoint -> URDF
        '''
        #help me initialize a new urdf using yourdfpy
        Robot_init = Robot(name)
        Robot_init.materials = []
        Robot_init.joints = []
        Robot_init.links = []
        Robot_init.actuated_joint_indices = []
        Robot_init.actuated_joints = []
        AEStructure = yourdfpy.URDF(Robot_init)
        slide_x_link = 'slide_x_link'
        slide_y_link = 'slide_y_link'
        tilt_y_link = 'tilt_y_link'
        tilt_r_link = 'tilt_r_link'
        slide_x_joint = 'slide_x_joint'
        slide_y_joint = 'slide_y_joint'
        tilt_y_joint = 'tilt_y_joint'
        tilt_r_joint = 'tilt_r_joint'


        AEStructure.robot.joints = []
        AEStructure.robot.links = []
        #find effector link in robot urdf
        effecor_Link = None
        for link in self.robot_urdf.robot.links:
            if link.name == effector:
                effecor_Link = link
                break

        #find affordance link in scene urdf
        affordance_Link = None
        for link in self.scene_urdf.robot.links:
            if link.name == affordance:
                affordance_Link = link
                break
        AEStructure.robot.links.append(effecor_Link)
        AEStructure.robot.links.append(Link(name=slide_x_link))
        AEStructure.robot.links.append(Link(name=slide_y_link))
        AEStructure.robot.links.append(Link(name=tilt_y_link))
        AEStructure.robot.links.append(Link(name=tilt_r_link))
        AEStructure.robot.links.append(affordance_Link)
        AEStructure.robot.joints.append(Joint(name=slide_x_joint, type='prismatic', 
                                                        parent=effector, child=slide_x_link, 
                                                        axis=(1,0,0), limit=Limit(lower=-slide_tolerence_x, upper=slide_tolerence_x, 
                                                                                  effort=DEFAULT_EFFORT, velocity=DEFAULT_VELOCITY)))
        AEStructure.robot.joints.append(Joint(name=slide_y_joint, type='prismatic', 
                                                        parent=slide_x_link, child=slide_y_link, 
                                                        axis=(0,1,0), limit=Limit(lower=-slide_tolerence_y, upper=slide_tolerence_y,
                                                                                  effort=DEFAULT_EFFORT, velocity=DEFAULT_VELOCITY)))
        AEStructure.robot.joints.append(Joint(name=tilt_y_joint, type='revolute',
                                                        parent=slide_y_link, child=tilt_y_link,
                                                        axis=(0,0,1), limit=Limit(lower=-3.14159265, upper=3.14159265,
                                                                                  effort=DEFAULT_EFFORT, velocity=DEFAULT_VELOCITY)))
        AEStructure.robot.joints.append(Joint(name=tilt_r_joint, type='revolute',
                                                        parent=tilt_y_link, child=tilt_r_link,
                                                        axis=(1,0,0), limit=Limit(lower=-tilt_tolerence_r, upper=tilt_tolerence_r,
                                                                                  effort=DEFAULT_EFFORT, velocity=DEFAULT_VELOCITY)))
        AEStructure.robot.joints.append(Joint(name=affordance, type='fixed',
                                                        parent=tilt_r_link, child=affordance))
        self.build_graph(AEStructure, verbose=verbose)
        #save AEStructure to a urdf file
        AEStructure.write_xml_file(AESTRUCTURE_URDF_PATH)
        # AEStructure.robots(AESTRUCTURE_URDF_PATH)
        
        return AEStructure
        
    def inverse_VKChain(self, VKChain):
        '''
        input: VKChain -> class: VKChain
        return: (VKChain, urdf)
        '''
        inversed_joints = VKChain.link_names[::-1]
        inversed_links = VKChain.joint_names[::-1]

    def construct_vkc_urdf(self):
        '''
        input: scene_urdf, robot_urdf, affordance, effector
        return: vkc_urdf, scene_urdf
        '''
        robot_joints = self.robot_urdf.robot.joints
        # for joint in robot_joints:
        #     print('parent of %s is %s' % (joint.name, joint.parent))
        #     print('child of %s is %s' % (joint.name, joint.child))
        #     print('joint type of %s is %s' % (joint.name, joint.type))
        #     print('joint axis of %s is %s' % (joint.name, joint.axis))
    
    def build_graph(self, urdf, verbose=False):
        self.G = nx.DiGraph()
        for j in urdf.robot.joints:
            self.G.add_edge(j.parent, j.child, name=j.name)
        if verbose == True:
            #show self.G
            nx.draw_networkx(self.G, arrows=True, with_labels=True)
            plt.show()
            # nx.draw_networkx(self.G, arrows=True,)
            # plt.show()

        return self.G
    
    def get_main_stream_links(self):
        '''
            links that connected by actuated joints
        '''
        self.MainStreamLinks = []
        #assume the first link is world link
        self.MainStreamLinks.append(self.robot_urdf.robot.links[0])
        actuated_joints = self.robot_urdf.actuated_joints
        for joint in self.robot_urdf.robot.joints:
            if joint in actuated_joints:
                self.MainStreamLinks.append(joint.child)
        return self.MainStreamLinks
     
    def find_main_link_path(self, urdf, upstream_link, downstream_link):
        '''
        input: urdf, link_name1, link_name2
        return: link_path -> list
        comment: parent of the first joint is upstream_link,
                    child of the last joint is downstream_link.
        '''
        #use bfs to find path between two links
        self.joints = urdf.robot.joints
        queue = [(upstream_link, [upstream_link])]

        while queue:
            link, path = queue.pop(0)
            if link == downstream_link:
                return path
            for joint in self.joints:
                if joint.parent == link:
                    queue.append((joint.child, path + [joint.child]))
        raise ValueError("No path found")
    
    def find_remend_link_path(self, urdf, upstream_link, downstream_link):
        '''
        input: urdf, link_name1, link_name2
        return: link_path -> list
        comment: parent of the first joint is upstream_link,
                    child of the last joint is downstream_link.
        '''
        path_root2upstreamlink = self.find_main_link_path(urdf, urdf.robot.links[0].name, upstream_link)
        self.get_main_stream_links()
        last_main_stream_link = ''
        for i in range(len(path_root2upstreamlink)):
            if path_root2upstreamlink[i] in self.MainStreamLinks and \
                path_root2upstreamlink[i+1] not in self.MainStreamLinks:
                last_main_stream_link = path_root2upstreamlink[i]
        
        path_upstreamlink2lastmainstreamlink = self.find_main_link_path(urdf, last_main_stream_link, upstream_link)
        path_upstreamlink2lastmainstreamlink.reverse()
        upstream_link = last_main_stream_link
        self.joints = urdf.robot.joints
        queue = [(upstream_link, [upstream_link])]
        path_search_result = []
        #use bfs to find path between two links
        while queue:
            link, path = queue.pop(0)
            if link == downstream_link:
                path_search_result= path
                break
            for joint in self.joints:
                if joint.parent == link:
                    queue.append((joint.child, path + [joint.child]))
        if len(path_search_result) == 0:
            raise ValueError("No path found")
        else:
            final_path = path_upstreamlink2lastmainstreamlink + path_search_result
            #delete repeated elemets in the list:final_path when keep the order
            final_path = list(OrderedDict.fromkeys(final_path))
        return final_path
    

    

    def get_vkc_problem_chains(self, effector, affordance, verbose=False):
        '''
        input: affordance, effector
        return: 
        '''
        self.path_rroot2effector = self.find_main_link_path(self.robot_urdf, 'world', effector)
        if verbose:
            print('main_links: ', self.path_rroot2effector)
        self.path_reffector2end = self.find_remend_link_path(self.robot_urdf, effector, self.robot_urdf.robot.links[-1].name)
        if verbose:
            print('remend_links: ', self.path_reffector2end)
        self.path_objroot2affordance = self.find_main_link_path(self.scene_urdf, 'world', affordance)
        if verbose:
            print('objroot2affordance_links: ', self.path_objroot2affordance)

        return self.path_rroot2effector, self.path_reffector2end, self.path_objroot2affordance
    



    
    def generate_push_vkc_urdf(self, affordance, effector, mainvkc_name = 'MainVKC', 
                               root2effector_name = 'root2effector', 
                               objaffordance2root_name = 'objaffordance2root',
                               verbose=False):
        #get three link paths
        root2effector, effector2end, objroot2affordance = self.get_vkc_problem_chains(effector, affordance)
        #get all reference URDFs
        self.AES_urdf = yourdfpy.URDF.load(self.AES_urdf_path)
        #generate root->effector->affordance->object_base_link->world URDF
            
        #concatenate root2effector and AEStructure
        #initialize a new URDF
        MainVKC_urdf = self.yourdfpy_init_URDF(mainvkc_name)
        #initialize a new URDF
        root2effector_urdf = self.yourdfpy_init_URDF(root2effector_name)
        #add link
        for link in root2effector:
            #find the link list by name
            link_names = [l for l in self.robot_urdf.robot.links if l.name == link]
            
            root2effector_urdf.robot.links.append(link_names[0])
        #add joint
        for i in range(len(root2effector_urdf.robot.links)):
            if i == 0:
                continue
            else:
                for joint in self.robot_urdf.robot.joints:
                    if joint.parent == root2effector_urdf.robot.links[i-1].name and \
                            joint.child == root2effector_urdf.robot.links[i].name:
                        root2effector_urdf.robot.joints.append(joint)
        
        MainVKC_urdf = root2effector_urdf
        #add AEStructure links
        MainVKC_urdf_link_names = [link.name for link in MainVKC_urdf.robot.links]
        for link in self.AES_urdf.robot.links:
            if link.name not in MainVKC_urdf_link_names:
                MainVKC_urdf.robot.links.append(link)
        #add AEStructure joints
        for i in range(len(MainVKC_urdf.robot.links)):
            if i == 0:
                continue
            else:
                for joint in self.AES_urdf.robot.joints:
                    if joint.parent == MainVKC_urdf.robot.links[i-1].name and \
                            joint.child == MainVKC_urdf.robot.links[i].name:
                        MainVKC_urdf.robot.joints.append(joint)
        #Concatenated robotroot->affordance
        #now we concatenate robotroot->affordance with affordance->world
        #now we have objroot2affordance, print it
        print('objroot2affordance: ', objroot2affordance)
        #now we get the vkc chain and validate it
        
        VKC_chain = VKChain(self.scene_urdf, objroot2affordance)
        self.get_env_robot_nxGraph()
        print('validating VKC_chain: ObjectRoot2Affordance',VKC_chain.validate(self.scene_nxGraph) )
        #now we inverse the chain
        VKC_chain.reverse_root_tip(VKC_chain.link_names[0], VKC_chain.link_names[-1])
        reversed_chain_URDF = VKC_chain.get_reversed_VKCs_urdf(objaffordance2root_name)
        #now we concatenate the reversed chain with the MainVKC_urdf
        for joint in MainVKC_urdf.robot.joints:
            print('MainVKC_urdf joint: ', joint.name)
        
        #add links
        MainVKC_urdf_link_names = [link.name for link in MainVKC_urdf.robot.links]
        for link in reversed_chain_URDF.robot.links:
            if link.name not in MainVKC_urdf_link_names:
                MainVKC_urdf.robot.links.append(link)
        #add joints
        for i in range(len(MainVKC_urdf.robot.links)):
            if i == 0:
                continue
            else:
                for joint in reversed_chain_URDF.robot.joints:
                    if joint.parent == MainVKC_urdf.robot.links[i-1].name and \
                            joint.child == MainVKC_urdf.robot.links[i].name:
                        MainVKC_urdf.robot.joints.append(joint)
        # MainVKC_urdf.robot.joints.append(Joint(name='world2micro', parent='world', child='microwave_base_link', type='fixed', origin=tra.identity_matrix()))
        #now we have the concatenated urdf



        
        MainVKC_urdf.write_xml_file(MR_URDF_PATH)


        #visualize root2effector_urdf
        
            



        #generate world->object_base_link->affordance->effector->robot_end URDF
        #generate remaining world URDF
    def niu_generate_push_vkc_urdf(self, affordance, effector, mainvkc_name = 'MainVKC', 
                               root2effector_name = 'root2effector', 
                               objaffordance2root_name = 'objaffordance2root',
                               verbose=False):
        #get three link paths
        root2effector, effector2end, objroot2affordance = self.get_vkc_problem_chains(effector, affordance)
        #get all reference URDFs
        self.AES_urdf = yourdfpy.URDF.load(self.AES_urdf_path)
        #generate root->effector->affordance->object_base_link->world URDF
            
        #concatenate root2effector and AEStructure
        #initialize a new URDF
        MainVKC_urdf = self.yourdfpy_init_URDF(mainvkc_name)
        #initialize a new URDF
        root2effector_urdf = self.yourdfpy_init_URDF(root2effector_name)
        #add link
        for link in root2effector:
            #find the link list by name
            link_names = [l for l in self.robot_urdf.robot.links if l.name == link]
            
            root2effector_urdf.robot.links.append(link_names[0])
        #add joint
        for i in range(len(root2effector_urdf.robot.links)):
            if i == 0:
                continue
            else:
                for joint in self.robot_urdf.robot.joints:
                    if joint.parent == root2effector_urdf.robot.links[i-1].name and \
                            joint.child == root2effector_urdf.robot.links[i].name:
                        root2effector_urdf.robot.joints.append(joint)
        
        MainVKC_urdf.robot.links = deepcopy(self.robot_urdf.robot.links)
        MainVKC_urdf.robot.joints = deepcopy(self.robot_urdf.robot.joints)
        #add AEStructure links
        MainVKC_urdf_link_names = [link.name for link in MainVKC_urdf.robot.links]
        for link in self.AES_urdf.robot.links:
            if link.name not in MainVKC_urdf_link_names:
                MainVKC_urdf.robot.links.append(link)
        #add AEStructure joints
        print('self.AES_urdf.robot.joints: ', self.AES_urdf.robot.joints)
        for i in range(len(MainVKC_urdf.robot.links)):
            if i == 0:
                continue
            else:
                for joint in self.AES_urdf.robot.joints:
                    if joint.parent in [link.name for link in MainVKC_urdf.robot.links] and \
                            joint.child in [link.name for link in MainVKC_urdf.robot.links]:
                        MainVKC_urdf.robot.joints.append(joint)
        print('MainVKC_urdf.robot.joints names: ', [joint.name for joint in MainVKC_urdf.robot.joints])
        #Concatenated robotroot->affordance
        #now we concatenate robotroot->affordance with affordance->world
        #now we have objroot2affordance, print it
        print('objroot2affordance: ', objroot2affordance)
        #now we get the vkc chain and validate it
        scene_link_names = [link.name for link in self.scene_urdf.robot.links]
        VKC_chain = VKChain(self.scene_urdf, scene_link_names)
        self.get_env_robot_nxGraph()
        print('validating VKC_chain: ObjectRoot2Affordance',VKC_chain.validate(self.scene_nxGraph) )
        #now we inverse the chain
        self.joints = [joint for joint in self.scene_urdf.robot.joints]
        print('self.joints before reverse root tip', self.joints)
        VKC_chain.reverse_root_tip(VKC_chain.link_names[0], VKC_chain.link_names[-1])
        reversed_chain_URDF = VKC_chain.get_reversed_VKCs_urdf(objaffordance2root_name)
        #now we concatenate the reversed chain with the MainVKC_urdf
        for joint in MainVKC_urdf.robot.joints:
            print('MainVKC_urdf joint: ', joint.name)
        
        #add links
        MainVKC_urdf_link_names = [link.name for link in MainVKC_urdf.robot.links]
        for link in reversed_chain_URDF.robot.links:
            if link.name not in MainVKC_urdf_link_names:
                MainVKC_urdf.robot.links.append(link)
        #add joints
        for i in range(len(MainVKC_urdf.robot.links)):
            if i == 0:
                continue
            else:
                for joint in reversed_chain_URDF.robot.joints:
                    if joint.parent == MainVKC_urdf.robot.links[i-1].name and \
                            joint.child == MainVKC_urdf.robot.links[i].name:
                        MainVKC_urdf.robot.joints.append(joint)
        # MainVKC_urdf.robot.joints.append(Joint(name='world2micro', parent='world', child='microwave_base_link', type='fixed', origin=tra.identity_matrix()))
        #now we have the concatenated urdf



        
        MainVKC_urdf.write_xml_file(MR_URDF_PATH)
    def generate_pushed_obj_urdf(self, urdf, link_path, affordance, effector):
        pass
    

    


         

    def get_target_pose(self, scene_urdf, affordance_link, joint_states):
        '''
        :param scene_urdf: URDF of the scene
        :param affordance_link: the link of the affordance
        :param joint_states: the joint states of the object chain, dict{'joint_name': joint_value}
        '''
        #get the object chain
        #return return, I give up finding ik in this python script, go back to c++
        return None
        print('WYY DEBUG', self.path_objroot2affordance)
        # scene_urdf.show()
        obj_chain = VKChain(scene_urdf, self.path_objroot2affordance)
        urdf_or2a, save_path = obj_chain.get_VKChain_urdf(name='objroot2affordance')
        target_joint_states = []
        for joint in urdf_or2a.robot.joints:
            if joint.type != 'fixed':
                if joint.name in joint_states.keys():
                    target_joint_states.append(joint_states[joint.name])
                else:
                    raise ValueError('joint dict mismatch', joint.name)
            else:
                target_joint_states.append(0)
        target_joint_states = [1.57]
        print('target_joint_states: ', target_joint_states)
        link_mask = [False]*len(urdf_or2a.robot.links)
        for i in range(len(urdf_or2a.robot.links)):
            if urdf_or2a.robot.links[i].name == affordance_link:
                link_mask[i-1] = True
        print(link_mask)

        
        #get target pose in the world frame
        ikpyChain_objr2a = ikpy.chain.Chain.from_urdf_file(save_path,base_elements=['world'])
        #compute fk of the object chain
        fk_objr2a = ikpyChain_objr2a.forward_kinematics(target_joint_states)
        print('fk_objr2a: ', fk_objr2a)
        translations = tra.translation_from_matrix(fk_objr2a)
        orientations = tra.euler_from_matrix(fk_objr2a)
        print('translations: ', translations)
        print('orientations: ', orientations)
        ik_objr2a = ikpyChain_objr2a.inverse_kinematics_frame(target=fk_objr2a)
        print('ik_objr2a: ', ik_objr2a)
        

        
        
            
    def get_attatch_IK(self, robot_urdf, scene_urdf, attach_link, target_pose):
        #get target link pose in the world frame
        pass


    def generate_push_request(self):
        pass
    def get_push_response(self):
        pass

    

    def test_ground(self):
        # print(self.robot_urdf.robot.joints[1].origin)
        #joint.origin is a SE(3) matrix
        pass



def module_test():
    P = PushIt()
    robot_base = tra.compose_matrix(angles=[0,0,0], translate=[0,0,0.64]) #WYY: not sure about the correctness, need test
    P.get_env_robot_urdf(ORI_SCENE_URDF_PATH, ORI_ROBOT_URDF_PATH,robot_base=robot_base,
                         robot_name='gen3', scene_name='simple_scene')
    affordance_hm = P.generate_affordance()

    effector_hm = P.generate_effector()
    P.construct_vkc_urdf()
    # successors = [suc for suc in scene_graph.successors('microwave_base_joint')]
    # print(successors)
    
    P.get_vkc_problem_chains(P.generate_effector(),P.generate_affordance(),verbose=True)
    P.generate_EAStruct_handmade(effector=effector_hm, affordance=affordance_hm)
    P.generate_push_vkc_urdf(effector=effector_hm, affordance=affordance_hm)
    # scene_graph = P.build_graph(P.scene_urdf,verbose=True)
    # P.get_vkc_problem_chains(P.generate_effector(),P.generate_affordance(),verbose=True)[2]
    P.get_target_pose(P.scene_urdf, 'microwave_handle', {'microwave_joint_0': 1.57})

    #Test VKChain

    # VC = VKChain(P.scene_urdf, P.get_vkc_problem_chains(P.generate_effector(),P.generate_affordance())[2])
    # # VC.get_reversed_chain_urdf(P.scene_urdf)
    # VC.reverse_root_tip('world', 'microwave_handle')
    # print(' ')

def get_niu_urdf():
    P = PushIt()
    robot_base = tra.compose_matrix(angles=[0,0,0], translate=[0,0,0])
    P.get_env_robot_urdf(ORI_NIU_SCENE, ORI_NIU_ROBOT,robot_base=robot_base,
                            robot_name='hur5e', scene_name='niu_scene')
    
    affordance = 'swivel_chair_base'

    effector = 'robotiq_arg2f_base_link'
    P.construct_vkc_urdf()
    P.get_vkc_problem_chains(effector=effector, affordance=affordance, verbose=True)
    P.niu_s(effector=effector, affordance=affordance)
    P.niu_generate_push_vkc_urdf(effector=effector, affordance=affordance, mainvkc_name='niu_s',
                             root2effector_name='niusr2e',
                             objaffordance2root_name='niuobjroot2affordance')
    
    
    

    


if __name__ == "__main__":
    #module_test for pushit
    # module_test()
    get_niu_urdf()

