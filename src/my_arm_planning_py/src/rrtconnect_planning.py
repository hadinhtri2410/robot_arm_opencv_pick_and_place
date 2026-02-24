from __future__ import division
import threading
import numpy as np
import time
import math
import os
import sys
from rclpy.node import Node
import rclpy
from rclpy.logging import get_logger
from my_robot_interfaces.srv import PlanJoint
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit.core.planning_scene import PlanningScene
from moveit.core.robot_model import RobotModel
from moveit.core.collision_detection import CollisionRequest, CollisionResult
from moveit_planning import MoveItPy, MultiplePlanningPipelines
from moveit.core.robot_state import RobotState
from moveit_msg.msg import PlanningScene as PlanningSceneMsg

class CollisionChecker: 
    def __init__(self, robot_model, planning_scene, group_name: str):
        self.robot_model = robot_model
        self.planning_scene = planning_scene
        self.group = robot_model.get_joint_model_group(group_name)
        self.robot_state = RobotState(robot_model)
        self.req_collision = CollisionRequest()
        self.res_collision = CollisionResult()
        self.lock = threading.Lock()
   
    def in_collision(self, q: np.ndarray) -> bool:
        with self.lock:
            self.robot_state.set_joint_group_positions(self.group, q.tolist())
            self.robot_state.update()  # important so FK/transforms are consistent
            self.res_collision.clear()
            ret = self.planning_scene.checkCollision(self.req_collision, self.res_collision, self.robot_state)

            if isinstance(ret, bool):
                return ret
            return self.res_collision.collision
    
class RRT_Node:
    def __init__(self, conf):
        self.conf = conf
        self.parent = None
        self.children = []

    def set_parent(self, parent):
        self.parent = parent
        if parent is not None:
            parent.children.append(self)

    def add_child(self, child):
        self.children.append(child)
        child.parent = self

def sample_conf():
        joints_limit_lower, joints_limit_upper = -np.pi, np.pi
        if np.random.rand() < 0.02: 
            return goal_conf, True
        else:
            rand_conf = []
            for lower, upper in zip(joints_limit_lower, joints_limit_upper):
                rand_conf.append(np.random.uniform(lower, upper))
            return tuple(rand_conf), False
        


   
def find_nearest(rand_node, node_list):
    nearest_node = None
    distance = []
    for node in node_list:
        d = np.linalg.norm(np.array(rand_node.conf) - np.array(node.conf))
        distance.append(d)
    nearest_node = node_list[np.argmin(distance)]
    return nearest_node

def steer_to(rand_node, nearest_node, in_collision_fn):
    q_from = np.array(nearest_node.conf, dtype=float)
    q_to = np.array(rand_node.conf, dtype=float)

    dvec = q_to - q_from
    dist = np.linalg.norm(dvec)

    if dist < 1e-9:
        return tuple(q_from)

    step = 0.05
    n = int(np.ceil(dist / step))
    
    for i in range(1, n + 1):
        t = i / (n - 1)
        q = (1 - t) * q_from + t * q_to
        if in_collision_fn(q):
            return None

    return tuple(q_to)
    
def steer_to_until(rand_node, nearest_node, in_collision_fn):
    q_new = steer_to(rand_node, nearest_node, in_collision_fn)
    last_valid = None
    last_q = None
    while q_new is not None:
        if last_q is not None and np.linalg.norm(np.array(q_new) - np.array(last_q)) < 1e-6:
            break
        last_valid = q_new
        if np.linalg.norm(np.array(q_new) - np.array(rand_node.conf)) < 1e-6:
            break
        last_q = q_new
        nearest_node = RRT_Node(q_new)
        q_new = steer_to(rand_node, nearest_node)
    return last_valid

def BiRRT():
    #################################################
    start_node = RRT_Node(start_conf)
    goal_node = RRT_Node(goal_conf)
    tree_start = [start_node]
    tree_goal = [goal_node]
    start_node.parent = None
    goal_node.parent = None
    for i in range(5000):
        q_rand_start, is_goal = sample_conf()
        rand_node_start = RRT_Node(q_rand_start)
        nearest_start = find_nearest(rand_node_start, tree_start)
        q_new_start = steer_to_until(rand_node_start, nearest_start, collision_fn)
        if q_new_start is not None: 
            q_new_start_node = RRT_Node(q_new_start)
            tree_start.append(q_new_start_node)
            q_new_start_node.set_parent(nearest_start)
            nearest_goal = find_nearest(q_new_start_node, tree_goal)
            q_new_goal = steer_to_until(q_new_start_node, nearest_goal, collision_fn)
            if q_new_goal is not None: 
                q_new_goal_node = RRT_Node(q_new_goal)
                tree_goal.append(q_new_goal_node)
                q_new_goal_node.set_parent(nearest_goal)
                if np.linalg.norm(np.array(q_new_start)-np.array(q_new_goal)) <= 0.05:
                    path_conf_start = []
                    current_node = q_new_start_node
                    while current_node is not None:
                        path_conf_start.append(current_node.conf)
                        current_node = current_node.parent
                    path_conf_start.reverse()
                    path_conf_goal = []
                    current_node = q_new_goal_node
                    while current_node is not None:
                        path_conf_goal.append(current_node.conf)
                        current_node = current_node.parent
                    path_conf = path_conf_start + path_conf_goal
                    return path_conf
        tree_start, tree_goal = tree_goal, tree_start
    return None

class PlanningNode(Node): 
    def __init__(self, collision_checker):
        super().__init__('rrtconnect_planning_node')
        self.collision_checker = collision_checker
        self.srv = self.create_service(PlanJoint, 'plan_joint', self.plan_joint_callback)
    
    def plan_joint_callback(self, request, response):
        global start_conf, goal_conf, collision_fn
        start_conf = tuple(request.start_conf)
        goal_conf = tuple(request.goal_conf)

        self.get_logger().info(f'Received planning request from {start_conf} to {goal_conf}')

        path_conf = None
        path_conf = BiRRT(self.collision_checker.in_collision)
        if path_conf is None:
            response.success = False
            response.message = "No collision-free path found"
            self.get_logger().info('No collision-free path found')
        else:
            response.success = True
            response.message = "Path found"
            traj = JointTrajectory()
            traj.joint_names = request.joint_names
            for q in path_conf:
                point = JointTrajectoryPoint()
                point.positions = list(q)
                point.time_from_start = 0.1
                traj.points.append(point)
            response.trajectory = traj
            self.get_logger().info('Path found and trajectory prepared')

        return response



if __name__ == "__main__":
    rclpy.init()
    robot_arm = MoveItPy(node_name = "rrtconnect_planning_node")
    robot_model = robot_arm.get_robot_model()
    planning_scene = PlanningScene(robot_model)
    checker = CollisionChecker(robot_model, planning_scene, group_name="ur3e_manipulator")
    robot_state = RobotState(robot_model)
    planning_node = PlanningNode(checker)  
    rclpy.spin(planning_node)
    planning_node.destroy_node()
    rclpy.shutdown()
