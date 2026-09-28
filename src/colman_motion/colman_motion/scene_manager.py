from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class SceneManager(Node):
    def __init__(self, moveit):
        super().__init__("scene_manager")
        self.planning_scene_monitor = moveit.get_planning_scene_monitor()

    def add_box(self, obj_id, frame_id, dims, pos, orientation_w=1.0):
        with self.planning_scene_monitor.read_write() as scene:
            obj = CollisionObject()
            obj.header.frame_id = frame_id
            obj.id = obj_id

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = dims

            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = pos
            pose.orientation.w = orientation_w

            obj.primitives.append(box)
            obj.primitive_poses.append(pose)
            obj.operation = CollisionObject.ADD

            scene.apply_collision_object(obj)
            scene.current_state.update()

        self.get_logger().info(f"Added: {obj_id}")

    def remove_object(self, obj_id):
        with self.planning_scene_monitor.read_write() as scene:
            obj = CollisionObject()
            obj.id = obj_id
            obj.operation = CollisionObject.REMOVE
            scene.apply_collision_object(obj)
            scene.current_state.update()
        self.get_logger().info(f"Removed: {obj_id}")

    def clear_scene(self):
        with self.planning_scene_monitor.read_write() as scene:
            scene.remove_all_collision_objects()
            scene.current_state.update()
        self.get_logger().info("Cleared all objects")
