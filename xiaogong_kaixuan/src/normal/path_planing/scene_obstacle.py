#!/usr/bin/env python3
"""只修改 MoveIt 中的虚拟障碍物；不发送机械臂运动指令。"""

import argparse
import json
import math

from planning_common import FRAME, rclpy, ros_environment_text
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


# ===== 障碍物参数：位置相对于 base_link，单位米 =====
OBSTACLE_ID = "left_arm_demo_obstacle"
OBSTACLE_CENTER_M = [0.258365, 0.347649, -0.628524]
OBSTACLE_RADIUS_M = 0.06  # 半径 6 cm 的球；中心在原关节直线路径的中间


class SceneObstacle(Node):
    def __init__(self):
        super().__init__("left_arm_scene_obstacle")
        self.apply_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.query_client = self.create_client(GetPlanningScene, "/get_planning_scene")

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError(f"Service unavailable: {client.srv_name}; {ros_environment_text()}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            future.cancel()
            raise RuntimeError(f"Service timed out: {client.srv_name}")
        response = future.result()
        if response is None:
            raise RuntimeError(f"No response: {client.srv_name}")
        return response

    def set_obstacle(self, enabled):
        obstacle = CollisionObject(id=OBSTACLE_ID)
        obstacle.header.frame_id = FRAME
        if enabled:
            if (len(OBSTACLE_CENTER_M) != 3 or not all(math.isfinite(v) for v in OBSTACLE_CENTER_M)
                    or not math.isfinite(OBSTACLE_RADIUS_M) or OBSTACLE_RADIUS_M <= 0):
                raise ValueError("Invalid OBSTACLE_CENTER_M or OBSTACLE_RADIUS_M")
            pose = Pose()
            pose.orientation.w = 1.0
            pose.position.x, pose.position.y, pose.position.z = OBSTACLE_CENTER_M
            obstacle.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE,
                                                  dimensions=[OBSTACLE_RADIUS_M])]
            obstacle.primitive_poses = [pose]
            obstacle.operation = CollisionObject.ADD
        else:
            obstacle.operation = CollisionObject.REMOVE
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.world.collision_objects = [obstacle]
        if not self.call(self.apply_client, request).success:
            raise RuntimeError("MoveIt refused the obstacle scene update")

    def objects(self):
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        return self.call(self.query_client, request).scene.world.collision_objects


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=["add", "remove", "status"])
    args = parser.parse_args()
    rclpy.init(args=[])
    node = SceneObstacle()
    try:
        print(ros_environment_text())
        if args.operation != "status":
            node.set_obstacle(args.operation == "add")
        objects = node.objects()
        print(json.dumps({"operation": args.operation, "object_ids": [obj.id for obj in objects],
                          "obstacle_id": OBSTACLE_ID,
                          "present": any(obj.id == OBSTACLE_ID for obj in objects),
                          "configured_center_m": OBSTACLE_CENTER_M,
                          "configured_radius_m": OBSTACLE_RADIUS_M}, indent=2))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
