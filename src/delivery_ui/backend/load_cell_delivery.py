#!/usr/bin/env python3
"""
load_cell_delivery.py — Dual AprilTag Docking Delivery Node
─────────────────────────────────────────────────────────────
Full state machine:

  IDLE
    weight stable N s ──► DOCK_LOAD
  DOCK_LOAD          align at loading station tag before departure
    docked ──────────► WAIT_BEFORE_NAV
  WAIT_BEFORE_NAV    countdown
    done ────────────► NAVIGATE_TO_DELIVERY
  NAVIGATE_TO_DELIVERY  Nav2 coarse waypoint near delivery tag
    arrived ─────────► DOCK_DELIVERY
  DOCK_DELIVERY      visual servo onto delivery station tag
    docked ──────────► WAIT_FOR_UNLOAD
  WAIT_FOR_UNLOAD    wait for weight to drop
    weight gone ─────► RETURN_WAIT
  RETURN_WAIT        countdown after unload
    done ────────────► NAVIGATE_TO_LOADING
  NAVIGATE_TO_LOADING  Nav2 coarse waypoint near loading tag
    arrived ─────────► DOCK_LOAD_RETURN
  DOCK_LOAD_RETURN   visual servo back onto loading station tag
    docked ──────────► IDLE
"""

import time
import enum
import math
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy,
    QoSReliabilityPolicy, QoSHistoryPolicy,
)

from std_msgs.msg import Float32
from geometry_msgs.msg import (
    PoseStamped, PoseWithCovarianceStamped,
    Twist, Point, Quaternion,
)
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

try:
    from apriltag_msgs.msg import AprilTagDetectionArray
    APRILTAG_MSGS = True
except ImportError:
    try:
        from apriltag_ros.msg import AprilTagDetectionArray
        APRILTAG_MSGS = True
    except ImportError:
        APRILTAG_MSGS = False


def build_pose(frame_id, px, py, pz, qx, qy, qz, qw) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id  = frame_id
    ps.pose.position    = Point(x=float(px), y=float(py), z=float(pz))
    ps.pose.orientation = Quaternion(
        x=float(qx), y=float(qy), z=float(qz), w=float(qw)
    )
    return ps


class State(enum.Enum):
    IDLE                 = "IDLE"
    DOCK_LOAD            = "DOCK_LOAD"
    WAIT_BEFORE_NAV      = "WAIT_BEFORE_NAV"
    NAVIGATE_TO_DELIVERY = "NAVIGATE_TO_DELIVERY"
    DOCK_DELIVERY        = "DOCK_DELIVERY"
    WAIT_FOR_UNLOAD      = "WAIT_FOR_UNLOAD"
    RETURN_WAIT          = "RETURN_WAIT"
    NAVIGATE_TO_LOADING  = "NAVIGATE_TO_LOADING"
    DOCK_LOAD_RETURN     = "DOCK_LOAD_RETURN"


class LoadCellDeliveryNode(Node):

    def __init__(self) -> None:
        super().__init__("load_cell_delivery_node")

        self.declare_parameter("weight_threshold",    2.0)
        self.declare_parameter("weight_stable_time",  5.0)
        self.declare_parameter("pre_navigate_delay",  5.0)
        self.declare_parameter("post_unload_delay",   5.0)
        self.declare_parameter("nav_server_timeout",  10.0)
        self.declare_parameter("map_frame",           "map")

        self.declare_parameter("dock_distance",    0.40)
        self.declare_parameter("align_tolerance",  0.03)
        self.declare_parameter("angle_tolerance",  0.05)
        self.declare_parameter("linear_speed",     0.08)
        self.declare_parameter("angular_speed",    0.30)
        self.declare_parameter("dock_timeout",     15.0)

        self.declare_parameter("load_tag_id",   1)
        self.declare_parameter("load_wp_x",     0.24)
        self.declare_parameter("load_wp_y",     0.08)
        self.declare_parameter("load_wp_z",     0.0)
        self.declare_parameter("load_wp_qx",    0.0)
        self.declare_parameter("load_wp_qy",    0.0)
        self.declare_parameter("load_wp_qz",   -0.13)
        self.declare_parameter("load_wp_qw",    0.99)

        self.declare_parameter("delivery_tag_id",  0)
        self.declare_parameter("delivery_wp_x",    0.21)
        self.declare_parameter("delivery_wp_y",    3.99)
        self.declare_parameter("delivery_wp_z",    0.0)
        self.declare_parameter("delivery_wp_qx",   0.0)
        self.declare_parameter("delivery_wp_qy",   0.0)
        self.declare_parameter("delivery_wp_qz",   0.03)
        self.declare_parameter("delivery_wp_qw",   0.99)

        self.declare_parameter("tag_topic",        "/tag_detections")
        self.declare_parameter("load_cell_topic",  "/load_cell_data")
        self.declare_parameter("amcl_pose_topic",  "/amcl_pose")
        self.declare_parameter("cmd_vel_topic",    "/cmd_vel")
        self.declare_parameter("nav_action",       "/navigate_to_pose")

        gp = self.get_parameter

        self.WEIGHT_THRESHOLD   = gp("weight_threshold").value
        self.WEIGHT_STABLE_TIME = gp("weight_stable_time").value
        self.PRE_NAV_DELAY      = gp("pre_navigate_delay").value
        self.POST_UNLOAD_DELAY  = gp("post_unload_delay").value
        self.NAV_TIMEOUT        = gp("nav_server_timeout").value
        self.MAP_FRAME          = gp("map_frame").value

        self.DOCK_DIST          = gp("dock_distance").value
        self.ALIGN_TOL          = gp("align_tolerance").value
        self.ANGLE_TOL          = gp("angle_tolerance").value
        self.LINEAR_SPD         = gp("linear_speed").value
        self.ANGULAR_SPD        = gp("angular_speed").value
        self.DOCK_TIMEOUT       = gp("dock_timeout").value

        self.LOAD_TAG_ID        = gp("load_tag_id").value
        self.DELIVERY_TAG_ID    = gp("delivery_tag_id").value

        TAG_TOPIC               = gp("tag_topic").value
        LOAD_CELL_TOPIC         = gp("load_cell_topic").value
        AMCL_TOPIC              = gp("amcl_pose_topic").value
        CMD_VEL_TOPIC           = gp("cmd_vel_topic").value
        NAV_ACTION              = gp("nav_action").value

        self._loading_waypoint = build_pose(
            self.MAP_FRAME,
            gp("load_wp_x").value,  gp("load_wp_y").value,  gp("load_wp_z").value,
            gp("load_wp_qx").value, gp("load_wp_qy").value,
            gp("load_wp_qz").value, gp("load_wp_qw").value,
        )
        self._delivery_waypoint = build_pose(
            self.MAP_FRAME,
            gp("delivery_wp_x").value,  gp("delivery_wp_y").value,
            gp("delivery_wp_z").value,
            gp("delivery_wp_qx").value, gp("delivery_wp_qy").value,
            gp("delivery_wp_qz").value, gp("delivery_wp_qw").value,
        )

        self._state              = State.IDLE
        self._state_lock         = threading.Lock()
        self._current_weight     = 0.0
        self._weight_start_time: Optional[float] = None
        self._unload_start_time: Optional[float] = None
        self._distance_remaining = 0.0
        self._last_log: dict     = {}

        self._detections: dict              = {}
        self._dock_start_time: Optional[float] = None
        self._dock_target_tag: Optional[int]   = None

        self._cbg = ReentrantCallbackGroup()

        amcl_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            Float32, LOAD_CELL_TOPIC,
            self._weight_cb, sensor_qos, callback_group=self._cbg,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, AMCL_TOPIC,
            self._amcl_cb, amcl_qos, callback_group=self._cbg,
        )

        if APRILTAG_MSGS:
            self.create_subscription(
                AprilTagDetectionArray, TAG_TOPIC,
                self._tag_cb, sensor_qos, callback_group=self._cbg,
            )
            self.get_logger().info(f"[TAG] Subscribed to {TAG_TOPIC}")
        else:
            self.get_logger().warn(
                "apriltag_msgs not found — will stop at waypoints without visual dock"
            )

        self._cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        self._nav_client = ActionClient(
            self, NavigateToPose, NAV_ACTION, callback_group=self._cbg,
        )

        self.create_timer(0.1, self._sm_tick, callback_group=self._cbg)

        self.get_logger().info(
            f"\n  Dual-Dock Delivery Node ready\n"
            f"  Weight threshold : {self.WEIGHT_THRESHOLD} kg\n"
            f"  Loading  tag ID  : {self.LOAD_TAG_ID}\n"
            f"  Delivery tag ID  : {self.DELIVERY_TAG_ID}\n"
            f"  Dock distance    : {self.DOCK_DIST} m"
        )

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _weight_cb(self, msg: Float32) -> None:
        self._current_weight = float(msg.data)

    def _amcl_cb(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        self._log("amcl", 30.0, f"[AMCL] x={p.x:.2f} y={p.y:.2f}")

    def _tag_cb(self, msg) -> None:
        now = time.monotonic()
        for det in msg.detections:
            tag_id = getattr(det, 'id', None)
            if isinstance(tag_id, (list, tuple)):
                tag_id = tag_id[0] if tag_id else None
            if tag_id is None:
                continue
            try:
                p = det.pose.pose.pose.position
                q = det.pose.pose.pose.orientation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                self._detections[tag_id] = {
                    "x": p.x, "z": p.z,
                    "yaw": math.atan2(siny, cosy),
                    "t": now,
                }
            except Exception:
                pass

    # ── State machine ─────────────────────────────────────────────────────────

    def _transition(self, new: State) -> None:
        with self._state_lock:
            old = self._state
            self._state = new
        self.get_logger().info(f"[SM] {old.value} → {new.value}")

    def _get_state(self) -> State:
        with self._state_lock:
            return self._state

    def _log(self, tag: str, interval: float, msg: str) -> None:
        now = time.monotonic()
        if now - self._last_log.get(tag, 0.0) >= interval:
            self._last_log[tag] = now
            self.get_logger().info(msg)

    def _sm_tick(self) -> None:
        state  = self._get_state()
        weight = self._current_weight

        if state == State.IDLE:
            self._log("idle", 2.0,
                f"[IDLE] weight={weight:.2f} kg  threshold={self.WEIGHT_THRESHOLD} kg"
            )
            if weight >= self.WEIGHT_THRESHOLD:
                if self._weight_start_time is None:
                    self._weight_start_time = time.monotonic()
                if time.monotonic() - self._weight_start_time >= self.WEIGHT_STABLE_TIME:
                    self.get_logger().info("[IDLE] Load detected — docking at loading station")
                    self._start_dock(self.LOAD_TAG_ID, State.DOCK_LOAD)
            else:
                self._weight_start_time = None

        elif state in (State.DOCK_LOAD, State.DOCK_DELIVERY, State.DOCK_LOAD_RETURN):
            self._dock_tick(state)

        elif state == State.WAIT_FOR_UNLOAD:
            self._log("unload", 2.0,
                f"[DELIVERY] Waiting for unload — weight={weight:.2f} kg"
            )
            if weight < self.WEIGHT_THRESHOLD:
                if self._unload_start_time is None:
                    self._unload_start_time = time.monotonic()
                if time.monotonic() - self._unload_start_time >= self.WEIGHT_STABLE_TIME:
                    self.get_logger().info("[DELIVERY] Unloaded — returning home")
                    self._transition(State.RETURN_WAIT)
                    threading.Thread(target=self._return_wait_thread, daemon=True).start()
            else:
                self._unload_start_time = None

        elif state in (State.NAVIGATE_TO_DELIVERY, State.NAVIGATE_TO_LOADING):
            self._log("nav", 2.0,
                f"[NAV] distance remaining: {self._distance_remaining:.2f} m"
            )

    # ── Docking ───────────────────────────────────────────────────────────────

    def _start_dock(self, tag_id: int, dock_state: State) -> None:
        self._dock_target_tag = tag_id
        self._dock_start_time = None
        self._transition(dock_state)

    def _dock_tick(self, current_state: State) -> None:
        if self._dock_start_time is None:
            self._dock_start_time = time.monotonic()

        elapsed = time.monotonic() - self._dock_start_time
        if elapsed > self.DOCK_TIMEOUT:
            self.get_logger().warn(f"[DOCK:{current_state.value}] Timeout — proceeding anyway")
            self._cmd_pub.publish(Twist())
            self._dock_start_time = None
            self._on_dock_complete(current_state)
            return

        if not APRILTAG_MSGS:
            self._cmd_pub.publish(Twist())
            self._dock_start_time = None
            self._on_dock_complete(current_state)
            return

        det   = self._detections.get(self._dock_target_tag)
        fresh = det and (time.monotonic() - det["t"] < 0.5)
        cmd   = Twist()

        if not fresh:
            self._log(f"srch_{current_state.value}", 1.0,
                f"[DOCK:{current_state.value}] Tag #{self._dock_target_tag} not visible — searching"
            )
            cmd.angular.z = self.ANGULAR_SPD * 0.4
            self._cmd_pub.publish(cmd)
            return

        dist    = det["z"]
        lateral = det["x"]
        yaw_err = det["yaw"]

        self._log(f"dock_{current_state.value}", 0.5,
            f"[DOCK:{current_state.value}] tag={self._dock_target_tag} "
            f"dist={dist:.2f}m lat={lateral:.3f}m yaw={yaw_err:.3f}rad"
        )

        if (dist <= self.DOCK_DIST and
                abs(lateral) < self.ALIGN_TOL and
                abs(yaw_err) < self.ANGLE_TOL):
            self.get_logger().info(
                f"[DOCK:{current_state.value}] Docked at {dist:.2f}m"
            )
            self._cmd_pub.publish(Twist())
            self._dock_start_time = None
            self._on_dock_complete(current_state)
            return

        angular_error = yaw_err + (lateral / max(dist, 0.1)) * 0.5
        cmd.angular.z = max(-self.ANGULAR_SPD,
                        min( self.ANGULAR_SPD, -angular_error * 1.5))

        if dist > self.DOCK_DIST:
            aligned = (abs(lateral) < self.ALIGN_TOL * 3 and
                       abs(yaw_err) < self.ANGLE_TOL * 3)
            if aligned:
                cmd.linear.x = min(self.LINEAR_SPD,
                                   (dist - self.DOCK_DIST) * 0.5 + 0.03)

        self._cmd_pub.publish(cmd)

    def _on_dock_complete(self, dock_state: State) -> None:
        if dock_state == State.DOCK_LOAD:
            self._transition(State.WAIT_BEFORE_NAV)
            threading.Thread(target=self._pre_nav_thread, daemon=True).start()
        elif dock_state == State.DOCK_DELIVERY:
            self._transition(State.WAIT_FOR_UNLOAD)
        elif dock_state == State.DOCK_LOAD_RETURN:
            self.get_logger().info("[DOCK] Home dock complete — cycle finished")
            self._transition(State.IDLE)
            self._weight_start_time = None
            self._unload_start_time = None

    # ── Countdown threads ─────────────────────────────────────────────────────

    def _pre_nav_thread(self) -> None:
        for i in range(int(self.PRE_NAV_DELAY), 0, -1):
            self.get_logger().info(f"[WAIT] Moving to delivery in {i} s...")
            time.sleep(1.0)
        self._transition(State.NAVIGATE_TO_DELIVERY)
        self._send_nav_goal(self._delivery_waypoint,
                            self._on_delivery_waypoint_reached, self._on_nav_failure)

    def _return_wait_thread(self) -> None:
        for i in range(int(self.POST_UNLOAD_DELAY), 0, -1):
            self.get_logger().info(f"[WAIT] Returning to loading station in {i} s...")
            time.sleep(1.0)
        self._transition(State.NAVIGATE_TO_LOADING)
        wp = self._loading_waypoint
        wp.header.stamp = self.get_clock().now().to_msg()
        self._send_nav_goal(wp, self._on_loading_waypoint_reached, self._on_nav_failure)

    # ── Nav2 pipeline ─────────────────────────────────────────────────────────

    def _send_nav_goal(self, pose, on_success, on_failure) -> None:
        if not self._nav_client.wait_for_server(timeout_sec=self.NAV_TIMEOUT):
            self.get_logger().error("[NAV] Action server unavailable!")
            on_failure()
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._nav_feedback_cb)
        future.add_done_callback(
            lambda f: self._goal_response_cb(f, on_success, on_failure))

    def _goal_response_cb(self, future, on_success, on_failure) -> None:
        gh: ClientGoalHandle = future.result()
        if not gh.accepted:
            self.get_logger().error("[NAV] Goal rejected!")
            on_failure()
            return
        gh.get_result_async().add_done_callback(
            lambda f: self._result_cb(f, on_success, on_failure))

    def _result_cb(self, future, on_success, on_failure) -> None:
        self._distance_remaining = 0.0
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            on_success()
        else:
            self.get_logger().error("[NAV] Navigation failed")
            on_failure()

    def _nav_feedback_cb(self, msg) -> None:
        self._distance_remaining = msg.feedback.distance_remaining

    def _on_delivery_waypoint_reached(self) -> None:
        self.get_logger().info("[NAV] Delivery waypoint reached — starting dock")
        self._start_dock(self.DELIVERY_TAG_ID, State.DOCK_DELIVERY)

    def _on_loading_waypoint_reached(self) -> None:
        self.get_logger().info("[NAV] Loading waypoint reached — docking home")
        self._start_dock(self.LOAD_TAG_ID, State.DOCK_LOAD_RETURN)

    def _on_nav_failure(self) -> None:
        self.get_logger().error("[NAV] Failed — resetting to IDLE in 3 s")
        time.sleep(3.0)
        self._transition(State.IDLE)

    # ── Flask API hooks ───────────────────────────────────────────────────────

    def get_status(self) -> dict:
        state  = self._get_state()
        target = self._dock_target_tag
        det    = self._detections.get(target) if target is not None else None
        fresh  = det and (time.monotonic() - det["t"] < 0.5)
        return {
            "state":        state.value,
            "weight":       round(self._current_weight, 3),
            "threshold":    self.WEIGHT_THRESHOLD,
            "distance":     round(self._distance_remaining, 3),
            "tag_visible":  bool(fresh),
            "tag_id":       target,
            "tag_x":        round(det["x"], 3) if fresh else 0.0,
            "tag_z":        round(det["z"], 3) if fresh else 0.0,
            "load_tag":     self.LOAD_TAG_ID,
            "delivery_tag": self.DELIVERY_TAG_ID,
        }

    def manual_trigger(self) -> str:
        state = self._get_state()
        if state != State.IDLE:
            return state.value
        self._weight_start_time = None
        self._start_dock(self.LOAD_TAG_ID, State.DOCK_LOAD)
        return "triggered"

    def destroy_node(self) -> None:
        self._cmd_pub.publish(Twist())
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LoadCellDeliveryNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
