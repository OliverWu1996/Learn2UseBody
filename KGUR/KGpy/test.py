import trimesh.transformations as tf

ORI_SCENE_URDF_PATH = "/home/y/Documents/WYY_META/effectit/VKC/src/vkc-planner-ros/vkc_deps/scene_builder/output/simple_scene/main.xacro"
ORI_ROBOT_URDF_PATH = "/home/y/Documents/WYY_META/effectit/VKC/src/vkc-planner-ros/vkc_core/vkc_example/robot_config/husky_ur5.xacro"


#help me initialize a class named UR2KG
#input: scene_urdf_path, robot_urdf_path
#input: vkc(robot,push,object_goal)
#output: new_scene_urdf, new_robot_urdf, robot_goal


class UR2KG():
    def __init__(self, scene_urdf_path, robot_urdf_path):
        self.scene_urdf_path = scene_urdf_path
        self.robot_urdf_path = robot_urdf_path
        self.scene_urdf = yourdfpy.URDF.load(self.scene_urdf_path)
        self.robot_urdf = yourdfpy.URDF.load(self.robot_urdf_path)
        # self.scene_urdf.show()
        self.robot_urdf.show()


        self.robot_urdf_str = yourdfpy.urdf_to_string(self.robot_urdf)
        self.scene_urdf_str = yourdfpy.urdf_to_string(self.scene_urdf)
        self.vkc_urdf = self.combine_vkc_env_urdf(self.robot_urdf, self.scene_urdf)
        self.vkc_urdf_str = yourdfpy.urdf_to_string(self.vkc_urdf)

class TEST():
    def __init__(self,v) -> None:
        self.v = v

def rotate():
    rot_before = tf.quaternion_matrix([0.6532815, 0.2705981, -0.6532815, -0.2705981])
    rotation_matrix = tf.rotation_matrix(1.57,[0,1,0])
    rot_after = tf.concate
    print(tf.quaternion_from_matrix(rot_after))
def main():
    pass

def BFS(list):
    #Breadth first search for the list
    #copilot generated codes
    queue = []
    for i in list:
        queue.append(i)
    while queue:
        vertex = queue.pop(0)
        print(vertex)
        for i in vertex.get_connections():
            if (i.color == 'white'):
                i.color = 'gray'
                i.distance = vertex.distance + 1
                i.predecessor = vertex
                queue.append(i)
        vertex.color = 'black'

if __name__ == "__main__":
    main()

