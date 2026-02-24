from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetStateValidity
from my_robot_interfaces.srv import PlanJoint
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


@dataclass(frozen=True)
class NodeConf:
    q: Tuple[float, ...]


class RRTConnectPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("rrtconnect_planning_node")
        self.service = self.create_service(PlanJoint, "plan_joint", self._handle_plan_joint)
        self.group_name = self.declare_parameter("group_name", "ur3e_manipulator").value
        self.use_rrt_fallback = bool(self.declare_parameter("use_rrt_fallback", True).value)
        self.enable_state_validity_checks = bool(
            self.declare_parameter("enable_state_validity_checks", False).value
        )
        self.state_validity_client = self.create_client(GetStateValidity, "/check_state_validity")
        self._active_joint_names: List[str] = []
        self._collision_checks_active = False
        self.get_logger().info("Service '/plan_joint' ready")

    def _handle_plan_joint(self, request: PlanJoint.Request, response: PlanJoint.Response) -> PlanJoint.Response:
        start = tuple(request.start_conf)
        goal = tuple(request.goal_conf)
        self._active_joint_names = list(request.joint_names)
        self._collision_checks_active = False
        if self.enable_state_validity_checks:
            self._collision_checks_active = self.state_validity_client.wait_for_service(timeout_sec=2.0)
            if not self._collision_checks_active:
                self.get_logger().warn(
                    "Service '/check_state_validity' unavailable; continuing without state-validity checks"
                )

        if len(start) == 0 or len(goal) == 0:
            response.success = False
            response.message = "start_conf/goal_conf must be non-empty"
            return response

        if len(start) != len(goal):
            response.success = False
            response.message = "start_conf and goal_conf lengths differ"
            return response

        if request.joint_names and len(request.joint_names) != len(start):
            response.success = False
            response.message = "joint_names length must match joint dimensions"
            return response

        step_size = request.step_size if request.step_size > 0.0 else 0.05
        max_iterations = int(request.max_iterations) if request.max_iterations > 0 else 3000
        goal = self._wrap_goal_near_start(start, goal)

        # Try direct interpolation first for stable, deterministic execution.
        # Fall back to RRTConnect only when a direct path is not available.
        path = self._plan_direct(start, goal, step_size)
        if path is None and self.use_rrt_fallback:
            path = self._plan_rrtconnect(start, goal, step_size, max_iterations)
        if path is None or len(path) < 2:
            response.success = False
            if self.use_rrt_fallback:
                if self._collision_checks_active:
                    response.message = "No path found (direct+RRT, collision-checked)"
                else:
                    response.message = "No path found (direct+RRT)"
            else:
                if self._collision_checks_active:
                    response.message = "No path found (direct only, collision-checked; set use_rrt_fallback=true)"
                else:
                    response.message = "No path found (direct only; set use_rrt_fallback=true)"
            return response

        traj = JointTrajectory()
        traj.joint_names = list(request.joint_names)

        max_joint_speed = 0.20  # rad/s per segment for stable controller tracking
        min_dt = 0.10
        start_hold_dt = 0.30
        elapsed = 0.0
        prev_q: Optional[Tuple[float, ...]] = None
        for i, q in enumerate(path):
            pt = JointTrajectoryPoint()
            pt.positions = list(q)
            if i == 0:
                elapsed += start_hold_dt
            else:
                assert prev_q is not None
                max_delta = max(abs(qj - pj) for qj, pj in zip(q, prev_q))
                seg_dt = max(min_dt, max_delta / max_joint_speed)
                elapsed += seg_dt
            sec_int = int(elapsed)
            nsec = int((elapsed - sec_int) * 1e9)
            pt.time_from_start = Duration(sec=sec_int, nanosec=nsec)
            traj.points.append(pt)
            prev_q = q

        response.success = True
        response.message = f"Path found with {len(path)} waypoints"
        response.trajectory = traj
        return response

    def _distance(self, a: Tuple[float, ...], b: Tuple[float, ...]) -> float:
        return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))

    def _interpolate(self, a: Tuple[float, ...], b: Tuple[float, ...], t: float) -> Tuple[float, ...]:
        return tuple((1.0 - t) * ai + t * bi for ai, bi in zip(a, b))

    def _sample_uniform(self, dim: int) -> Tuple[float, ...]:
        # Generic joint-space bounds; replace with real model limits when available.
        return tuple(random.uniform(-math.pi, math.pi) for _ in range(dim))

    def _wrap_goal_near_start(
        self,
        start: Tuple[float, ...],
        goal: Tuple[float, ...],
    ) -> Tuple[float, ...]:
        wrapped: List[float] = []
        two_pi = 2.0 * math.pi
        for s, g in zip(start, goal):
            # Keep equivalent joint angle closest to current value to avoid large
            # wraparound jumps that trigger trajectory tolerance aborts.
            delta = g - s
            g_near = g - two_pi * round(delta / two_pi)
            wrapped.append(g_near)
        return tuple(wrapped)

    def _plan_direct(
        self,
        start: Tuple[float, ...],
        goal: Tuple[float, ...],
        step_size: float,
    ) -> Optional[List[Tuple[float, ...]]]:
        d = self._distance(start, goal)
        if d <= 1e-9:
            return [start, goal]

        step = max(step_size, 1e-3)
        n = max(1, int(math.ceil(d / step)))
        path = [self._interpolate(start, goal, float(i) / float(n)) for i in range(n + 1)]
        for q_from, q_to in zip(path, path[1:]):
            if not self._collision_free(q_from, q_to, step):
                return None
        return path

    def _nearest(self, tree: Dict[Tuple[float, ...], Optional[Tuple[float, ...]]], q: Tuple[float, ...]) -> Tuple[float, ...]:
        return min(tree.keys(), key=lambda n: self._distance(n, q))

    def _steer(self, q_from: Tuple[float, ...], q_to: Tuple[float, ...], step: float) -> Tuple[float, ...]:
        d = self._distance(q_from, q_to)
        if d <= step:
            return q_to
        t = step / d
        return self._interpolate(q_from, q_to, t)

    def _collision_free(self, _q_from: Tuple[float, ...], _q_to: Tuple[float, ...], _step: float) -> bool:
        if not self._collision_checks_active:
            return True

        q_from = tuple(float(v) for v in _q_from)
        q_to = tuple(float(v) for v in _q_to)
        step = max(float(_step), 1e-3)
        d = self._distance(q_from, q_to)
        segments = max(1, int(math.ceil(d / step)))
        for i in range(1, segments + 1):
            t = float(i) / float(segments)
            q = self._interpolate(q_from, q_to, t)
            if not self._is_state_valid(q):
                return False
        return True

    def _is_state_valid(self, q: Tuple[float, ...]) -> bool:
        if not self._collision_checks_active:
            return True
        if len(self._active_joint_names) != len(q):
            return False

        req = GetStateValidity.Request()
        req.group_name = self.group_name
        req.robot_state.joint_state.name = list(self._active_joint_names)
        req.robot_state.joint_state.position = [float(v) for v in q]
        req.robot_state.joint_state.velocity = []
        req.robot_state.joint_state.effort = []

        future = self.state_validity_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        if not future.done() or future.result() is None:
            return False
        return bool(future.result().valid)

    def _extend(
        self,
        tree: Dict[Tuple[float, ...], Optional[Tuple[float, ...]]],
        q_target: Tuple[float, ...],
        step: float,
    ) -> Tuple[str, Tuple[float, ...]]:
        q_near = self._nearest(tree, q_target)
        q_new = self._steer(q_near, q_target, step)

        if not self._collision_free(q_near, q_new, step):
            return "trapped", q_near

        tree[q_new] = q_near
        if q_new == q_target:
            return "reached", q_new
        return "advanced", q_new

    def _connect(
        self,
        tree: Dict[Tuple[float, ...], Optional[Tuple[float, ...]]],
        q_target: Tuple[float, ...],
        step: float,
    ) -> Tuple[str, Tuple[float, ...]]:
        status = "advanced"
        q_last = q_target
        while status == "advanced":
            status, q_last = self._extend(tree, q_target, step)
        return status, q_last

    def _trace_path(
        self,
        tree: Dict[Tuple[float, ...], Optional[Tuple[float, ...]]],
        q: Tuple[float, ...],
    ) -> List[Tuple[float, ...]]:
        path: List[Tuple[float, ...]] = [q]
        cur = q
        while tree[cur] is not None:
            cur = tree[cur]  # type: ignore[index]
            path.append(cur)
        path.reverse()
        return path

    def _plan_rrtconnect(
        self,
        start: Tuple[float, ...],
        goal: Tuple[float, ...],
        step_size: float,
        max_iterations: int,
    ) -> Optional[List[Tuple[float, ...]]]:
        if start == goal:
            return [start, goal]

        tree_start: Dict[Tuple[float, ...], Optional[Tuple[float, ...]]] = {start: None}
        tree_goal: Dict[Tuple[float, ...], Optional[Tuple[float, ...]]] = {goal: None}

        dim = len(start)
        extend_start_tree = True
        for k in range(max_iterations):
            q_rand = goal if random.random() < 0.1 else self._sample_uniform(dim)
            if extend_start_tree:
                tree_a = tree_start
                tree_b = tree_goal
            else:
                tree_a = tree_goal
                tree_b = tree_start

            status_a, q_new = self._extend(tree_a, q_rand, step_size)
            if status_a != "trapped":
                status_b, q_meet = self._connect(tree_b, q_new, step_size)
                if status_b == "reached":
                    if extend_start_tree:
                        path_from_start = self._trace_path(tree_start, q_new)
                        path_from_goal = self._trace_path(tree_goal, q_meet)
                    else:
                        path_from_start = self._trace_path(tree_start, q_meet)
                        path_from_goal = self._trace_path(tree_goal, q_new)
                    path_from_goal.reverse()
                    return path_from_start + path_from_goal[1:]

            extend_start_tree = not extend_start_tree

        return None


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = RRTConnectPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
