#!/usr/bin/env python3
import math
import time
import yaml
from typing import List, Dict

import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from nav2_msgs.action import FollowWaypoints
from sensor_msgs.msg import BatteryState


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class PatrolManager(Node):
    def __init__(self):
        super().__init__('patrol_manager')

        # ── Subscriptions
        self.cmd_sub = self.create_subscription(String, '/patrol/cmd', self.on_cmd, 10)
        self.sec_evt_sub = self.create_subscription(String, '/security/event', self.on_security_event, 10)
        self.sec_ack_sub = self.create_subscription(Empty, '/security/ack', self.on_security_ack, 10)

        # ── Publishers
        self.initpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/patrol/status', 10)

        # ── Manual nudge 제어
        self.manual_active = False
        self.manual_twist = Twist()
        self.manual_timer = self.create_timer(0.1, self._on_manual_timer)

        # ── Nav2 Action
        self.client = ActionClient(self, FollowWaypoints, '/follow_waypoints')

        # ── Params (기본)
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', 0.0)
        self.declare_parameter('origin_yaw', 0.0)     # rad
        self.declare_parameter('start_mode', 'fixed')  # 'fixed' or 'live'

        # ── Params (배터리)
        self.declare_parameter('batt_topic', '/battery_state')
        self.declare_parameter('v_min', 10.5)       # 0%
        self.declare_parameter('v_max', 12.5)       # 100%
        self.declare_parameter('low_percent', 35)   # 이 미만이면 자동 복귀
        self.declare_parameter('avg_window', 10)    # 이동평균 샘플 수

        # ── Param values
        self.frame_id = self.get_parameter('frame_id').value
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)
        self.origin_yaw = float(self.get_parameter('origin_yaw').value)
        self.start_mode = self.get_parameter('start_mode').value

        self.batt_topic = self.get_parameter('batt_topic').value
        self.v_min = float(self.get_parameter('v_min').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.low_percent = int(self.get_parameter('low_percent').value)
        self.avg_window = int(self.get_parameter('avg_window').value)

        # ── State
        self.base_route: List[PoseStamped] = []     # 전체 순찰용 (poses)
        self.route: List[PoseStamped] = []          # 현재 실행 중인 루트 (전체 or zone or sequence)
        self.zones: Dict[str, Dict] = {}            # { 'A': {entry: PoseStamped, route: [PoseStamped,...]}, ... }

        self.current_zone = None                    # 단일 존 모드일 때 현재 ZONE
        self.zone_sequence: List[str] = []          # 시퀀스 모드일 때 존 리스트 (['A','D','C'])
        self.in_progress = False
        self.paused = False
        self.waiting_ack = False
        self.current_index = 0
        self._goal_handle = None

        self.repeat_mode = False       # 반복 여부 (전체 루트 or zone or sequence)
        self.auto_return = False       # 끝나면 origin 복귀 여부
        self._current_task = 'none'    # 'route' | 'zone_route' | 'zone_seq' | 'return' | 'none'

        # ── AMCL 안정화 체크
        self.localized = False
        self._last_localize_ok = 0.0
        self._initialpose_cooldown = 8.0
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._on_amcl_pose,
            10
        )

        # ── Battery
        self._volt_buf: List[float] = []
        self._low_batt_active = False
        self.create_subscription(BatteryState, self.batt_topic, self.on_battery, 5)
        self._pause_requested = False

        # ── Load waypoints & zones
        wp_file = self.get_parameter('waypoint_file').value
        if not wp_file:
            self.get_logger().error('waypoint_file param required')
        else:
            self.load_waypoints(wp_file)

        self.get_logger().info(
            f'PatrolManager ready frame={self.frame_id} '
            f'origin=({self.origin_x:.2f},{self.origin_y:.2f},{self.origin_yaw:.3f}rad) '
            f'start_mode={self.start_mode} '
            f'batt=[{self.v_min:.2f}V→0%, {self.v_max:.2f}V→100%, low<{self.low_percent}%] '
            f'zones={list(self.zones.keys())}'
        )

    # ──────────────────────────────────────────────────────────
    # Waypoints & Zones
    # ──────────────────────────────────────────────────────────
    def load_waypoints(self, path: str):
        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        # home_pose 있으면 origin_* 덮어쓰기
        home = data.get('home_pose')
        if home:
            self.origin_x = float(home.get('x', 0.0))
            self.origin_y = float(home.get('y', 0.0))
            self.origin_yaw = float(home.get('yaw', 0.0))
            self.get_logger().info(
                f'[HOME] from yaml: ({self.origin_x:.2f}, {self.origin_y:.2f}, {self.origin_yaw:.3f}rad)'
            )

        # frame_id
        self.frame_id = data.get('frame_id', self.frame_id)

        # poses (옵션, 전체 순찰 루트)
        self.route = []
        now = self.get_clock().now().to_msg()
        for p in data.get('poses', []):
            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.header.stamp = now
            pos = p['position']
            ori = p['orientation']
            ps.pose.position.x = float(pos.get('x', 0.0))
            ps.pose.position.y = float(pos.get('y', 0.0))
            ps.pose.position.z = float(pos.get('z', 0.0))
            ps.pose.orientation.x = float(ori.get('x', 0.0))
            ps.pose.orientation.y = float(ori.get('y', 0.0))
            ps.pose.orientation.z = float(ori.get('z', 0.0))
            ps.pose.orientation.w = float(ori.get('w', 1.0))
            self.route.append(ps)

        self.base_route = list(self.route)
        self.get_logger().info(
            f'Loaded {len(self.base_route)} base waypoints from {path} (frame_id={self.frame_id})'
        )

        # zones (A/B/C/…) - polygon은 현재 로직에서 미사용 (시각화용)
        self.zones = {}
        zones_data = data.get('zones', {})
        for name, zconf in zones_data.items():
            zname = str(name).upper()

            entry = zconf.get('entry_pose')
            if not entry:
                self.get_logger().warn(f'[ZONES] {zname}: no entry_pose, skipped')
                continue

            ep = PoseStamped()
            ep.header.frame_id = self.frame_id
            ep.header.stamp = now
            ep.pose.position.x = float(entry.get('x', 0.0))
            ep.pose.position.y = float(entry.get('y', 0.0))
            ep.pose.position.z = 0.0
            ep.pose.orientation = yaw_to_quat(float(entry.get('yaw', 0.0)))

            wps: List[PoseStamped] = []
            for w in zconf.get('patrol_waypoints', []):
                pps = PoseStamped()
                pps.header.frame_id = self.frame_id
                pps.header.stamp = now
                pps.pose.position.x = float(w.get('x', 0.0))
                pps.pose.position.y = float(w.get('y', 0.0))
                pps.pose.position.z = 0.0
                yaw = float(w.get('yaw', 0.0))
                pps.pose.orientation = yaw_to_quat(yaw)
                wps.append(pps)

            if not wps:
                self.get_logger().warn(f'[ZONES] {zname}: no patrol_waypoints, skipped')
                continue

            self.zones[zname] = {
                'entry': ep,
                'route': wps,
            }

        if self.zones:
            self.get_logger().info(f'[ZONES] Loaded zones: {list(self.zones.keys())}')
        else:
            self.get_logger().info('[ZONES] No zones defined in waypoint_file')

    def make_origin_pose(self) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = self.origin_x
        ps.pose.position.y = self.origin_y
        ps.pose.position.z = 0.0
        ps.pose.orientation = yaw_to_quat(self.origin_yaw)
        return ps

    # ──────────────────────────────────────────────────────────
    # AMCL / initialpose
    # ──────────────────────────────────────────────────────────
    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        cov = msg.pose.covariance
        pos_var = cov[0] + cov[7]
        yaw_var = cov[35]

        pos_ok = pos_var < (0.05 ** 2 + 0.05 ** 2)
        yaw_ok = yaw_var < (math.radians(5)) ** 2

        if pos_ok and yaw_ok:
            if not self.localized:
                self.get_logger().info('🎯 AMCL localized')
            self.localized = True
            self._last_localize_ok = self.get_clock().now().nanoseconds * 1e-9

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.origin_x
        msg.pose.pose.position.y = self.origin_y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quat(self.origin_yaw)

        cov = [0.0] * 36
        cov[0] = (0.05) ** 2
        cov[7] = (0.05) ** 2
        cov[35] = (math.radians(5)) ** 2
        msg.pose.covariance = cov

        self.initpose_pub.publish(msg)
        self.get_logger().info('[INITPOSE] published once')

    def _maybe_publish_initial_pose(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        if self.localized:
            self.get_logger().info('[INITPOSE] skipped (already localized)')
            return
        if now - self._last_localize_ok < self._initialpose_cooldown:
            self.get_logger().info('[INITPOSE] skipped (cooldown)')
            return

        try:
            self.publish_initial_pose()
        except Exception as e:
            self.get_logger().warn(f'[INITPOSE] publish failed: {e}')

    # ──────────────────────────────────────────────────────────
    # FollowWaypoints
    # ──────────────────────────────────────────────────────────
    def _zero_cmd_vel(self, times: int = 3, interval: float = 0.05):
        zero = Twist()
        for _ in range(times):
            self.vel_pub.publish(zero)
            time.sleep(interval)

    def send_follow_waypoints(self, poses: List[PoseStamped]):
        if not poses:
            self.get_logger().warn('send_follow_waypoints: empty list')
            return

        if self._goal_handle is not None and self.in_progress:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._zero_cmd_vel()

        goal = FollowWaypoints.Goal()
        goal.poses = poses

        self.client.wait_for_server()
        future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)

        self.in_progress = True
        self.paused = False
        self.waiting_ack = False

    # ──────────────────────────────────────────────────────────
    # Status publish (/patrol/status)
    # ──────────────────────────────────────────────────────────
    def publish_status(self, event: str = ""):
        """현재 순찰 상태를 /patrol/status(JSON String)로 publish."""
        payload = {
            "event": event,                     # "start_route", "stop", "low_batt_return" 등
            "task": self._current_task,         # "route" | "zone_route" | "zone_seq" | "return" | "none"
            "in_progress": self.in_progress,
            "paused": self.paused,
            "repeat": self.repeat_mode,
            "auto_return": self.auto_return,
            "current_zone": self.current_zone or "",
            "zone_sequence": self.zone_sequence,
            "current_index": self.current_index,
            "route_length": len(self.route),
            "low_batt_active": self._low_batt_active,
            "localized": self.localized,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)

    # ──────────────────────────────────────────────────────────
    # Commands (/patrol/cmd)
    # ──────────────────────────────────────────────────────────
    def on_cmd(self, msg: String):
        cmd = (msg.data or '').strip().lower()

        if cmd == 'start':
            self.start_once()
        elif cmd == 'pause':
            self.pause_patrol()
        elif cmd == 'stop':
            self.stop_patrol()
        elif cmd == 'resume':
            self.resume_patrol()
        elif cmd == 'return':
            self.return_to_origin_immediate()
        elif cmd == 'start_once':
            self.start_once()
        elif cmd == 'start_repeat':
            self.start_repeat()
        elif cmd.startswith('zone:'):
            parts = cmd.split(':')
            if len(parts) == 2:
                self.start_zone(parts[1], repeat=False)      # 1회 + origin 복귀
            elif len(parts) >= 3 and parts[2] == 'repeat':
                self.start_zone(parts[1], repeat=True)       # 해당 zone 반복
            else:
                self.get_logger().warn(f'Unknown zone cmd: "{cmd}"')
        elif cmd.startswith('zones:'):
            # 예: zones:a,d,c  /  zones:a,d,c:repeat
            parts = cmd.split(':')
            if len(parts) == 2:
                seq_raw = parts[1]
                repeat = False
            elif len(parts) >= 3 and parts[2] == 'repeat':
                seq_raw = parts[1]
                repeat = True
            else:
                self.get_logger().warn(f'Unknown zones cmd: "{cmd}"')
                return

            zone_names = [s.strip() for s in seq_raw.split(',') if s.strip()]
            self.start_zone_sequence(zone_names, repeat=repeat)
        elif cmd == 'manual_forward':
            self.set_manual_motion(0.15, 0.0)
        elif cmd == 'manual_backward':
            self.set_manual_motion(-0.15, 0.0)
        elif cmd == 'manual_left':
            self.set_manual_motion(0.0, 0.8)
        elif cmd == 'manual_right':
            self.set_manual_motion(0.0, -0.8)
        elif cmd == 'manual_stop':
            self.stop_manual()
        else:
            self.get_logger().warn(f'Unknown /patrol/cmd: "{cmd}"')

    # ──────────────────────────────────────────────────────────
    # 전체 루트
    # ──────────────────────────────────────────────────────────
    def start_once(self):
        if not self.base_route:
            self.get_logger().error('No base waypoints loaded')
            return
        if self.in_progress:
            self.get_logger().info('Patrol already running')
            return

        self._reset_state_for_new_run()
        self.route = list(self.base_route)
        self.current_zone = None
        self.zone_sequence = []

        self.repeat_mode = False
        self.auto_return = True
        self._current_task = 'route'

        self._maybe_publish_initial_pose()

        plan = self.route if self.start_mode == 'live' else [self.make_origin_pose()] + self.route
        self.current_index = 0
        self.get_logger().info(f'[START ONCE] n={len(plan)} (auto_return=True)')
        self.send_follow_waypoints(plan)
        self.publish_status(event="start_route_once")

    def start_repeat(self):
        if not self.base_route:
            self.get_logger().error('No base waypoints loaded')
            return
        if self.in_progress and self.repeat_mode and not self.current_zone and not self.zone_sequence:
            self.get_logger().info('Repeat patrol already running')
            return

        self._reset_state_for_new_run()
        self.route = list(self.base_route)
        self.current_zone = None
        self.zone_sequence = []

        self.repeat_mode = True
        self.auto_return = False
        self._current_task = 'route'

        self._maybe_publish_initial_pose()

        plan = self.route if self.start_mode == 'live' else [self.make_origin_pose()] + self.route
        self.current_index = 0
        self.get_logger().info(f'[START REPEAT] n={len(plan)} (loop until return)')
        self.send_follow_waypoints(plan)
        self.publish_status(event="start_route_repeat")

    # ──────────────────────────────────────────────────────────
    # 단일 ZONE
    # ──────────────────────────────────────────────────────────
    def start_zone(self, zone_name: str, repeat: bool = False):
        if not self.zones:
            self.get_logger().error('[ZONE] no zones configured')
            return

        zkey = str(zone_name).upper()
        if zkey not in self.zones:
            self.get_logger().error(f'[ZONE] "{zkey}" not found in zones')
            return

        if self.in_progress and self.repeat_mode and self.current_zone == zkey:
            self.get_logger().info(f'[ZONE {zkey}] already running')
            return

        z = self.zones[zkey]

        self._reset_state_for_new_run()

        plan_core = [z['entry']] + z['route']
        self.route = list(plan_core)
        self.current_zone = zkey
        self.zone_sequence = []

        self.repeat_mode = repeat
        self.auto_return = not repeat
        self._current_task = 'zone_route'

        self._maybe_publish_initial_pose()

        self.current_index = 0
        mode_txt = 'REPEAT' if repeat else 'ONCE'
        self.get_logger().info(f'[ZONE {zkey} {mode_txt}] n={len(plan_core)}')
        self.send_follow_waypoints(plan_core)
        self.publish_status(event=f"start_zone_{zkey}_{mode_txt.lower()}")

    # ──────────────────────────────────────────────────────────
    # ZONE 시퀀스 (예: A,D,C)
    # ──────────────────────────────────────────────────────────
    def start_zone_sequence(self, zone_names: List[str], repeat: bool = False):
        if not self.zones:
            self.get_logger().error('[ZONES SEQ] no zones configured')
            return

        norm: List[str] = []
        for z in zone_names:
            zkey = str(z).upper()
            if not zkey:
                continue
            if zkey not in self.zones:
                self.get_logger().error(f'[ZONES SEQ] "{zkey}" not found in zones')
                return
            norm.append(zkey)

        if not norm:
            self.get_logger().error('[ZONES SEQ] empty sequence')
            return

        self._reset_state_for_new_run()

        plan_core: List[PoseStamped] = []
        for zkey in norm:
            z = self.zones[zkey]
            plan_core.append(z['entry'])
            plan_core.extend(z['route'])

        self.route = list(plan_core)
        self.current_zone = None
        self.zone_sequence = norm

        self.repeat_mode = repeat               # repeat=True면 시퀀스 전체 반복
        self.auto_return = not repeat           # repeat=False면 마지막에 origin 복귀
        self._current_task = 'zone_seq'

        self._maybe_publish_initial_pose()

        self.current_index = 0
        mode_txt = 'REPEAT' if repeat else 'ONCE'
        self.get_logger().info(
            f'[ZONES SEQ {mode_txt}] {norm} n={len(plan_core)} '
            f'auto_return={self.auto_return}'
        )
        self.send_follow_waypoints(plan_core)
        self.publish_status(event=f"start_seq_{','.join(norm)}_{mode_txt.lower()}")

    # ──────────────────────────────────────────────────────────
    # 제어
    # ──────────────────────────────────────────────────────────
    def pause_patrol(self):
        if not self.in_progress:
            self.get_logger().info('No patrol to pause')
            return
        
        self._pause_requested = True

        self._zero_cmd_vel()

        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass

        self.in_progress = False
        self.paused = True
        self.get_logger().info(f'Patrol pause requeseted at index {self.current_index}')
        self.publish_status(event="pause_requested")
    
    def stop_patrol(self):
        self.repeat_mode = False
        self.auto_return = False
        self.current_zone = None
        self.zone_sequence = []

        self._zero_cmd_vel()

        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass

        for _ in range(20):
            self.vel_pub.publish(Twist())
            time.sleep(0.02)

        self.in_progress = False
        self.paused = False
        self._current_task = 'none'
        self.get_logger().info(f'Patrol stopped at index {self.current_index}')
        self.publish_status(event="stop")

    def resume_patrol(self):
        if not self.route:
            self.get_logger().error('No route to resume')
            return
        if self.in_progress:
            self.get_logger().info('Already in progress')
            return

        next_idx = min(self.current_index + 1, len(self.route) - 1)
        plan = self.route[next_idx:]
        if not plan:
            self.get_logger().info('Nothing to resume')
            return

        if self.zone_sequence:
            self._current_task = 'zone_seq'
            self.get_logger().info(
                f'[ZONES SEQ {self.zone_sequence}] Resuming from index {next_idx} (n={len(plan)})'
            )
        elif self.current_zone:
            self._current_task = 'zone_route'
            self.get_logger().info(
                f'[ZONE {self.current_zone}] Resuming from index {next_idx} (n={len(plan)})'
            )
        else:
            self._current_task = 'route'
            self.get_logger().info(
                f'Resuming from waypoint index {next_idx} (n={len(plan)})'
            )

        self.paused = False
        self.send_follow_waypoints(plan)
        self.publish_status(event="resume")

    def return_to_origin_immediate(self):
        self.repeat_mode = False
        self.auto_return = False
        self.current_zone = None
        self.zone_sequence = []
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._zero_cmd_vel()
        origin = self.make_origin_pose()
        self._current_task = 'return'
        self.get_logger().info('Returning to ORIGIN (home_pose)')
        self.send_follow_waypoints([origin])
        self.publish_status(event="return_immediate")

    def _reset_state_for_new_run(self):
        if self._goal_handle is not None and self.in_progress:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._zero_cmd_vel()
        self.in_progress = False
        self.paused = False
        self.waiting_ack = False
        self.current_index = 0
        self._goal_handle = None
        self._current_task = 'none'
        self.current_zone = None
        self.zone_sequence = []

    # ──────────────────────────────────────────────────────────
    # Battery (auto return)
    # ──────────────────────────────────────────────────────────
    def _volt_to_percent(self, v: float) -> int:
        if self.v_max <= self.v_min:
            return 0
        p = round(100.0 * (v - self.v_min) / (self.v_max - self.v_min))
        return max(0, min(100, p))

    def on_battery(self, msg: BatteryState):
        v = float(msg.voltage) if (msg.voltage is not None) else 0.0
        if v <= 0.0:
            return

        self._volt_buf.append(v)
        if len(self._volt_buf) > self.avg_window:
            self._volt_buf.pop(0)
        v_avg = sum(self._volt_buf) / len(self._volt_buf)

        percent = self._volt_to_percent(v_avg)

        if self._low_batt_active:
            return

        if percent < self.low_percent:
            self._low_batt_active = True
            self.get_logger().warn(
                f'[LOW BATT] {percent}% (avg {v_avg:.2f}V) → stop & return'
            )
            self.return_to_origin_immediate()
            self.publish_status(event="low_batt_auto_return")

    # ──────────────────────────────────────────────────────────
    # Security events
    # ──────────────────────────────────────────────────────────
    def on_security_event(self, msg: String):
        evt = (msg.data or '').strip().lower()
        self.get_logger().warn(f'[SECURITY EVENT] "{evt}" → immediate PAUSE')
        self.pause_patrol()
        self.waiting_ack = True
        self.publish_status(event=f"security_event_{evt}")

    def on_security_ack(self, _msg: Empty):
        if not self.waiting_ack:
            self.get_logger().info('ACK received but no pending alert')
            return
        self.get_logger().info('ACK → resume patrol')
        self.waiting_ack = False
        self.resume_patrol()
        self.publish_status(event="security_ack")

    # ──────────────────────────────────────────────────────────
    # Action callbacks
    # ──────────────────────────────────────────────────────────
    def on_goal_response(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle or not self._goal_handle.accepted:
            self.get_logger().error('FollowWaypoints goal rejected')
            self.in_progress = False
            self._goal_handle = None
            self._current_task = 'none'
            return
        self.get_logger().info(
            f'FollowWaypoints goal accepted (task={self._current_task})'
        )
        self._goal_handle.get_result_async().add_done_callback(self.on_result)

    def on_feedback(self, feedback_msg):
        try:
            self.current_index = int(feedback_msg.feedback.current_waypoint)
        except Exception:
            pass

    def on_result(self, _future):
        if self._pause_requested:
            self.get_logger().info(
                f'FollowWaypoints goal cancelled due to PAUSE (task={self._current_task})'
            )
            self._pause_requested = False
            self.in_progress = False
            self._goal_handle = None
            self.get_logger().info('Patrol is now paused and ready to resume')
            self.publish_status(event="pause_goal_cancelled")
            return
        
        self.in_progress = False
        self._goal_handle = None
        self.get_logger().info(f'Goal finished (task={self._current_task})')

        # 1) 전체 루트
        if self._current_task == 'route':
            if self.repeat_mode:
                plan = (
                    self.route
                    if self.start_mode == 'live'
                    else [self.make_origin_pose()] + self.route
                )
                self.current_index = 0
                self.get_logger().info('[REPEAT] re-run route')
                self._current_task = 'route'
                self.send_follow_waypoints(plan)
                self.publish_status(event="route_repeat_restart")
                return

            if self.auto_return:
                origin = self.make_origin_pose()
                self._current_task = 'return'
                self.get_logger().info('[AUTO RETURN] returning to origin')
                self.send_follow_waypoints([origin])
                self.publish_status(event="route_auto_return")
                return

            self._current_task = 'none'
            self.get_logger().info('Patrol finished and ready')
            self.publish_status(event="route_finished")
            return

        # 2) 단일 ZONE
        if self._current_task == 'zone_route':
            zname = self.current_zone
            if self.repeat_mode and zname and zname in self.zones:
                z = self.zones[zname]
                plan = [z['entry']] + z['route']
                self.current_index = 0
                self.get_logger().info(f'[ZONE {zname}] repeat')
                self._current_task = 'zone_route'
                self.send_follow_waypoints(plan)
                self.publish_status(event=f"zone_{zname}_repeat_restart")
                return

            if self.auto_return:
                origin = self.make_origin_pose()
                self._current_task = 'return'
                self.get_logger().info(
                    f'[ZONE {zname}] auto return to origin'
                )
                self.send_follow_waypoints([origin])
                self.publish_status(event=f"zone_{zname}_auto_return")
                return

            self.get_logger().info(f'[ZONE {zname}] finished')
            self.current_zone = None
            self._current_task = 'none'
            self.get_logger().info('Patrol finished and ready')
            self.publish_status(event=f"zone_{zname}_finished")
            return

        # 3) ZONE 시퀀스
        if self._current_task == 'zone_seq':
            if self.repeat_mode and self.zone_sequence:
                # 전체 시퀀스를 다시 수행 (origin 복귀 없이 loop)
                plan = list(self.route)
                self.current_index = 0
                self.get_logger().info(
                    f'[ZONES SEQ REPEAT] {self.zone_sequence}'
                )
                self._current_task = 'zone_seq'
                self.send_follow_waypoints(plan)
                self.publish_status(event="zone_seq_repeat_restart")
                return

            if self.auto_return:
                origin = self.make_origin_pose()
                self._current_task = 'return'
                self.get_logger().info(
                    f'[ZONES SEQ {self.zone_sequence}] auto return to origin'
                )
                self.send_follow_waypoints([origin])
                self.publish_status(event="zone_seq_auto_return")
                return

            self.get_logger().info(
                f'[ZONES SEQ {self.zone_sequence}] finished'
            )
            self.zone_sequence = []
            self._current_task = 'none'
            self.get_logger().info('Patrol finished and ready')
            self.publish_status(event="zone_seq_finished")
            return

        # 4) origin 복귀 등
        self.current_zone = None
        self.zone_sequence = []
        self._current_task = 'none'
        self.get_logger().info('Patrol finished and ready')
        self.publish_status(event="finished")

    # ──────────────────────────────────────────────────────────
    # Manual control (once → keep moving until stop)
    # ──────────────────────────────────────────────────────────
    def _on_manual_timer(self):
        """수동 모드일 때 주기적으로 cmd_vel publish"""
        if not self.manual_active:
            return
        self.vel_pub.publish(self.manual_twist)

    def set_manual_motion(self, lin_x: float, ang_z: float):
        """수동 전/후/좌/우 시작 (기존 순찰은 모두 정리)"""
        # 순찰 / 네비게이션 목표 있으면 취소
        if self._goal_handle is not None and self.in_progress:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass

        self.in_progress = False
        self.paused = False
        self._current_task = 'none'

        # 수동 모드 설정
        self.manual_twist = Twist()
        self.manual_twist.linear.x = lin_x
        self.manual_twist.angular.z = ang_z
        self.manual_active = True

        self.get_logger().info(
            f'[MANUAL] lin={lin_x:.2f}, ang={ang_z:.2f}'
        )
        self.publish_status(event="manual_motion")

    def stop_manual(self):
        """수동 모드 정지"""
        self.manual_active = False
        self.manual_twist = Twist()
        self._zero_cmd_vel()
        self.get_logger().info('[MANUAL] stop')
        self.publish_status(event="manual_stop")

def main():
    rclpy.init()
    node = PatrolManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()