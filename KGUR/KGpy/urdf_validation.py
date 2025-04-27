import yourdfpy
import logging
import sys

ORI_SCENE_URDF_PATH = "/home/y/Documents/WYY_META/effectit/VKC/src/vkc-planner-ros/vkc_deps/scene_builder/output/env_description/singleobj3_env/main.urdf"
ORI_ROBOT_URDF_PATH = "/home/y/Documents/WYY_META/effectit/ros_kortex/kortex_description/arms/gen3/7dof/urdf/GEN3_URDF_V12_absPath_pushit.urdf"
class URDFValidation:
    def __init__(self, env_urdf_path, robot_urdf_path, logging_level=logging.DEBUG) -> None:
        '''
        Validate Robot and Environment URDFs
        ROBOT URDF CRITERIA:
        1. joint.limit should have effort and velocity property


        ENV URDF CRITERIA:
        1. ENV URDF must have and only have one 'world' link
        2. ENV URDF must have base_link
        '''
        self.env_urdf = yourdfpy.URDF.load(env_urdf_path)
        self.robot_urdf = yourdfpy.URDF.load(robot_urdf_path)
        self.logger = logging.getLogger("wyy_logger")
        handler = logging.StreamHandler(sys.stdout)
        self.logger.setLevel(logging_level)
        self.logger.addHandler(handler)


    def validate_robot_urdf(self, auto_fill=True):
        self.robot_links = self.robot_urdf.robot.links
        self.robot_joints = self.robot_urdf.robot.joints
        for joint in self.robot_joints:
            if joint.limit is not None:
                if joint.limit.effort is None:
                    if auto_fill:
                        joint.limit.effort = 0
                        logging.warning('Robot URDF joint.limit.effort is None, auto filled with 0')
                    else:
                        raise Exception('Robot URDF joint.limit.effort is None')
                if joint.limit.velocity is None:
                    if auto_fill:
                        joint.limit.velocity = 0
                        logging.warning('Robot URDF joint.limit.velocity is None, auto filled with 0')
                    else:
                        raise Exception('Robot URDF joint.limit.velocity is None')
        #find end_link in list self.robot_links
        # end_link_count = 0
        # for link in self.robot_links:
        #     if link.name.find('end')!=-1:
        #         end_link_count += 1
        # if end_link_count < 1:
        #     raise Exception('Robot URDF must have \'end_link\' link')
        self.logger.info('Robot URDF validation passed')
        # print('Robot URDF validation passed')
    
    def validate_env_urdf(self):
        self.env_link = self.env_urdf.robot.links
        self.env_joints = self.env_urdf.robot.joints
        #find world link in list self.env_link
        world_link_count = 0
        for link in self.env_link:
            if link.name == 'world':
                world_link_count += 1
        if world_link_count != 1:
            raise Exception('Environment URDF must have and only have one \'world\' link')
        #find base_link in list self.env_link
        world_child_count = 0
        base_link_count = 0
        for joint in self.env_joints:
            if joint.parent == 'world':
                world_child_count += 1
                if joint.child.find('base_link')!=-1:
                    base_link_count += 1
        if world_child_count != base_link_count:
            raise Exception('Environment URDF must have \'base_link\' as child of \'world\'')
            
            
        
        self.logger.info('Environment URDF validation passed')
        # print('Environment URDF validation passed')


def test_module():
    V = URDFValidation(ORI_SCENE_URDF_PATH, ORI_ROBOT_URDF_PATH)
    V.validate_robot_urdf()
    V.validate_env_urdf()

if __name__ == "__main__":
    test_module()


        