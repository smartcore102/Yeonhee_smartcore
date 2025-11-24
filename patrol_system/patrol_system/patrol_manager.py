#!/usr/bin/env python3
"""
Patrol Manager 노드

기능 개요
- Nav2의 FollowWaypoints 액션을 사용하여 다음 기능을 제공
  1) 전체 웨이포인트 기반 순찰 (1회 / 반복)
  2) ZONE(구역) 단위 순찰 (단일 구역, 구역 시퀀스)
  3) 수동 제어 (전/후/좌/우) 및 정지
  4) 배터리 상태(BatteryState) 기반 저전압 자동 복귀
  5) /initialpose 자동 퍼블리시를 통한 AMCL 초기 위치 설정 보조
  6) /patrol/status 토픽을 통한 현재 상태 JSON 브로드캐스트(웹 연동 용도)

토픽/액션 인터페이스
- 구독(Subscriptions)
    /patrol/cmd         : String, 순찰 및 수동 제어 명령
    /amcl_pos           : PoseWithCovarianceeStamped, 위치 추정 상태 확인
    /battery_state      : BatteryState, 로봇 배터리 전압 모니터링

- 발행(Publishers)
    /initialpose        : PoseWithCovarianceStamped, 초기 위치 설정
    /cmd_vel            : Twist, 수동 제어 및 안전 정지를 위한 속도 명령
    /patrol/status      : String(JSON), 웹을 위한 순찰 상태 브로드캐스트

- 액션(Action)
    /follow_waypoints   : nav2_msgs/action/FollowWaypoints, Nav2 기반 다중 웨이포인트 읻동
"""
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
    """
    Z축 기준 yaw(rad)를 Quaternion(z, w)로 변환
    """
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class PatrolManager(Node):
    """
    순찰 로봇 상위 제어를 담당하는 ROS2 노드

    - Nav2 FollowWaypoints 액션 클라이언트로 웨이포인트 기반 순찰 수행
    - /patrol/cmd 명령 문자열을 해석하여 순찰/정지/복귀/수동 제어 등 수행
    - waypoint_file YAML을 읽어서 전체 루트 및 zones(A/B/C) 구성을 로딩
    - AMCL 상태를 모니터링하여 초기 위치 설정(initialpose) 자동화 보조
    - 배터리 전압이 일정 임계치 이하로 떨어지면 자동으로 원점 복귀
    - /patrol/status로 현재 상태를 JSON 형태로 브로드캐스트
    """
    def __init__(self):
        super().__init__('patrol_manager')

        # ── Subscriptions: 외부에서 들어오는 제어·상태 토픽 구독 ──────────────────────
        # /patrol/cmd : 문자열 기반 순찰/수동 제어 명령
        self.cmd_sub = self.create_subscription(String, '/patrol/cmd', self.on_cmd, 10)
        # /amcl_pose : 위치 추정 상태 모니터링
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._on_amcl_pose,
            10
        )
        # /battery_state : 배터리 전압 모니터링
        self.create_subscription(BatteryState, self.batt_topic, self.on_battery, 5)

        # ── Publishers: 로봇 제어 및 상태 브로드캐스트 ─────────────────────────────
        # /initialpose : AMCL 초기 위치 설정용
        self.initpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # /cmd_vel : 수동 제어 및 긴급 정지를 위한 속도 명령
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # /patrol/status : 현재 순찰 상태를 JSON 문자열로 송출 (웹에서 사용)
        self.status_pub = self.create_publisher(String, '/patrol/status', 10)

        # ── Manual nudge 제어: 수동 모드에서 cmd_vel을 주기적으로 발행 ───────────────
        self.manual_active = False      # 수동 제어 모드 활성 여부
        self.manual_twist = Twist()     # 수동 제어 시 발행할 속도 메시지
        # 0.1초 주기 타이머로 수동 제어 명령을 지속적으로 발행
        self.manual_timer = self.create_timer(0.1, self._on_manual_timer)

        # ── Nav2 FollowWaypoints 액션 클라이언트 ─────────────────────────────────
        self.client = ActionClient(self, FollowWaypoints, '/follow_waypoints')

        # ── 일반 파라미터 (맵/시작 위치 관련) ────────────────────────────────────
        # waypoint_file : 순찰 루트 및 zones가 정의된 YAML 파일 경로
        self.declare_parameter('waypoint_file', '')
        # frame_id :웨이포인트 및 초기 pose가 기준으로 하는 좌표계
        self.declare_parameter('frame_id', 'map')
        # origin_* : home_pose(원점) 좌표 및 yaw(rad)
        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', 0.0)
        self.declare_parameter('origin_yaw', 0.0)     # rad
        # start_mode : 'fixed'면 항상 origin에서 출발, 'live'면 현재 위치에서 바로 출발
        self.declare_parameter('start_mode', 'fixed')  # 'fixed' or 'live'

        # ── 배터리 관련 파라미터 ────────────────────────────────────────────────
        self.declare_parameter('batt_topic', '/battery_state')
        # v_min, v_max : 전압을 0~100%로 환산하기 위한 최소/최대 전압 값
        self.declare_parameter('v_min', 10.5)       # 0%에 해당하는 전압
        self.declare_parameter('v_max', 12.5)       # 100%에 해당하는 전압
        # low_percent : 35% 미만일 경우 저전압으로 판단하고 자동 복귀 수행
        self.declare_parameter('low_percent', 35)   # %
        # avg_window : 이동 평균 계산 시 사용할 샘플 개수
        self.declare_parameter('avg_window', 10)

        # ── 파라미터 실제 값 로딩 ───────────────────────────────────────────────
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

        # ── 내부 상태 변수들(State) ─────────────────────────────────────────────
        self.base_route: List[PoseStamped] = []     # YAML에서 읽은 전체 순찰 경로
        self.route: List[PoseStamped] = []          # 현재 실행 중인 순찰 경로
        self.zones: Dict[str, Dict] = {}            # 구역별 entry_pose와 내부 patrol_waypoints 구성

        self.current_zone = None                    # 단일 구역 모드에서 현재 순찰 중인 구역 이름
        self.zone_sequence: List[str] = []          # 구역 시퀀스 모드에서 구역 리스트

        self.in_progress = False                    # FollowWaypoints 실행 중인지 여부
        self.paused = False                         # 일시정지 상태인지 여부
        self.current_index = 0                      # 현재 웨이포인트 인덱스
        self._goal_handle = None                    # 현재 액션 goal 핸들

        self.repeat_mode = False                    # 반복 순찰 여부
        self.auto_return = False                    # 순찰 완료 후 origin 자동 복귀 여부
        self._current_task = 'none'                 # 'route' | 'zone_route' | 'zone_seq' | 'return' | 'none'

        # ── AMCL 안정화 체크 관련 변수 ─────────────────────────────────────────
        self.localized = False                      # AMCL이 충분히 수렴했다고 판단되면 True
        self._last_localize_ok = 0.0                # 마지막으로 완정화됨을 확인한 시간
        self._initialpose_cooldown = 8.0            # initialpose를 과도하게 퍼블리시하지 않도록 하는 쿨다운

        # ── 배터리 상태 모니터링 ──────────────────────────────────────────────
        self._volt_buf: List[float] = []            # 전압 이동 평균 버퍼
        self._low_batt_active = False               # 저전압 자동 복귀 1회만 수행하도록 관리
        self._pause_requested = False

        # ── 웨이포인트 및 존 정보 로딩 ────────────────────────────────────────
        wp_file = self.get_parameter('waypoint_file').value
        if not wp_file:
            # waypoints_file 파라미터가 제공되지 않으면 로그 출력
            self.get_logger().error('waypoint_file 파라미터가 설정되지 않았습니다.')
        else:
            self.load_waypoints(wp_file)

        # 초기 구성 정보 로그 출력
        self.get_logger().info(
            f'PatrolManager 준비 완료 | frame={self.frame_id} '
            f'origin=({self.origin_x:.2f},{self.origin_y:.2f},{self.origin_yaw:.3f}rad) '
            f'start_mode={self.start_mode} '
            f'배터리=[{self.v_min:.2f}V→0%, {self.v_max:.2f}V→100%, 저전압<{self.low_percent}%] '
            f'zones={list(self.zones.keys())}'
        )

    # ──────────────────────────────────────────────────────────
    # Waypoints & Zones 로딩
    # ──────────────────────────────────────────────────────────
    def load_waypoints(self, path: str):
        """
        YAML 파일에서 home_pose, 전체 순찰 웨이포인트, zones 정보를 로딩
        """
        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        # home_pose가 정의되어 있으면 origin_* 값을 덮어씀
        home = data.get('home_pose')
        if home:
            self.origin_x = float(home.get('x', 0.0))
            self.origin_y = float(home.get('y', 0.0))
            self.origin_yaw = float(home.get('yaw', 0.0))
            self.get_logger().info(
                f'[HOME] from yaml: ({self.origin_x:.2f}, {self.origin_y:.2f}, {self.origin_yaw:.3f}rad)'
            )

        # frame_id가 별도로 정의되어 있으면 반영
        self.frame_id = data.get('frame_id', self.frame_id)

        # poses: 전체 순찰 루트
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

        # base_route에 원본을 보관해 두고, 실제 실행 시에는 복사해서 사용
        self.base_route = list(self.route)
        self.get_logger().info(
            f'{path}에서 기본 웨이포인트 {len(self.base_route)}개 로드 완료 '
            f'(frame_id={self.frame_id})'
        )

        # zones: A/B/C 각 구역에 대한 entry_pose 및 내부 순찰 웨이포인트
        # polygon 정보는 현재 로직에서는 사용하지 않지만, 시각화 등의 용도로 남겨둠
        self.zones = {}
        zones_data = data.get('zones', {})
        for name, zconf in zones_data.items():
            zname = str(name).upper()

            #entry_pose 필수
            entry = zconf.get('entry_pose')
            if not entry:
                self.get_logger().warn(f'[ZONES] {zname}: entry_pose가 없어 스킵합니다.')
                continue

            ep = PoseStamped()
            ep.header.frame_id = self.frame_id
            ep.header.stamp = now
            ep.pose.position.x = float(entry.get('x', 0.0))
            ep.pose.position.y = float(entry.get('y', 0.0))
            ep.pose.position.z = 0.0
            ep.pose.orientation = yaw_to_quat(float(entry.get('yaw', 0.0)))

            # patrol_waypoints: 해당 구역 내부를 도는 세부 웨이포인트 리스트
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
                self.get_logger().warn(f'[ZONES] {zname}: patrol_waypoints가 없어 스킵힙니다.')
                continue

            self.zones[zname] = {
                'entry': ep,
                'route': wps,
            }

        if self.zones:
            self.get_logger().info(f'[ZONES] 구역 로드 완료: {list(self.zones.keys())}')
        else:
            self.get_logger().info('[ZONES] waypoint_file에 정의된 구역이 없습니다.')

    def make_origin_pose(self) -> PoseStamped:
        """
        현재 origin_x, origin_y, origin_yaw 설정값을 기반으로
        origin 위치에 해당하는 PoseStamped 메시지를 생성
        """
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = self.origin_x
        ps.pose.position.y = self.origin_y
        ps.pose.position.z = 0.0
        ps.pose.orientation = yaw_to_quat(self.origin_yaw)
        return ps

    # ──────────────────────────────────────────────────────────
    # AMCL / initialpose 관련
    # ──────────────────────────────────────────────────────────
    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        """
        /amcl_pose 콜백
        - 공분산 정보를 활용하여 위치(x, y)와 yaw의 분산이 충분히 작으면
          AMCL이 수렴했다고 판단하고 localized 플래그를 True로 설정
        """
        cov = msg.pose.covariance
        # x, y 방향 분산의 합 (대략적인 위치 분산)
        pos_var = cov[0] + cov[7]
        # yaw(roll/pitch가 0이라고 가정했을 때 마지막 요소)를 분산으로 사용
        yaw_var = cov[35]

        # 위치 오차가 약 5cm 이내라고 가정하는 조건ㄴ
        pos_ok = pos_var < (0.05 ** 2 + 0.05 ** 2)
        # yaw 오차가 약 5도 이낸라고 가정하는 조건
        yaw_ok = yaw_var < (math.radians(5)) ** 2

        if pos_ok and yaw_ok:
            if not self.localized:
                self.get_logger().info('🎯 AMCL 위치 추정 수렴(Localized)')
            self.localized = True
            self._last_localize_ok = self.get_clock().now().nanoseconds * 1e-9

    def publish_initial_pose(self):
        """
        /initialpose에 origin 기반 PoseWithCovarianceStamped를 1회 발행
        - Nav2가 시작된 후, RViz에서 초기 위치를 수동으로 설정하는 대신
          코드에서 자동 설정을 도와주는 용도
        """
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.origin_x
        msg.pose.pose.position.y = self.origin_y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quat(self.origin_yaw)

        # Covariance 설정 : 위치 ±5cm, yaw ±5도 정도의 불확실성을 가정
        cov = [0.0] * 36
        cov[0] = (0.05) ** 2
        cov[7] = (0.05) ** 2
        cov[35] = (math.radians(5)) ** 2
        msg.pose.covariance = cov

        self.initpose_pub.publish(msg)
        self.get_logger().info('[INITPOSE] origin 기준 초기 위치를 1회 퍼블리시했습니다.')

    def _maybe_publish_initial_pose(self):
        """
        AMCL 수렴 상태와 쿨다운 시간을 고려하여 필요한 경우에만
        publish_initial_pose()를 호출함
        """
        now = self.get_clock().now().nanoseconds * 1e-9

        # 이미 localized 된 경우에는 재발행하지 않음
        if self.localized:
            self.get_logger().info('[INITPOSE] 이미 AMCL이 수렴하여 초기 위치 퍼블리시 건너뜁니다.')
            return
        # 최근에 안정화 상태를 확인했으면 일정 시간 동안은 재발행하지 않음
        if now - self._last_localize_ok < self._initialpose_cooldown:
            self.get_logger().info('[INITPOSE] 쿨다운 시간 내이므로 초기 위치 퍼블리시를 건너뜁니다.')
            return

        try:
            self.publish_initial_pose()
        except Exception as e:
            self.get_logger().warn(f'[INITPOSE] 초기 위치 퍼블리시 실패: {e}')

    # ──────────────────────────────────────────────────────────
    # FollowWaypoints 액션 관련 공통 유틸
    # ──────────────────────────────────────────────────────────
    def _zero_cmd_vel(self, times: int = 3, interval: float = 0.05):
        """
        로봇을 안전하게 정지시키기 위해 일정 횟수 동안 0 속도를 발행
        """
        zero = Twist()
        for _ in range(times):
            self.vel_pub.publish(zero)
            time.sleep(interval)

    def send_follow_waypoints(self, poses: List[PoseStamped]):
        """
        FollowWaypoints 액션 서버로 다중 웨이포인트 goal을 전송

        - 기존 goal이 실행 중이면 cancel_goal_async()로 취소 후
          cmd_vel을 0으로 여러 번 발행하여 정지 상태를 보장
        - goal 전송 후 on_goal_response, on_feedback, on_result 콜백을 ㅗㅌㅇ해
          진행 상황 및 결과를 처리
        """
        if not poses:
            self.get_logger().warn('send_follow_waypoints: 웨이포인트 리스트가 비어 있어 전송하지 않습니다.')
            return

        # 기존 액션 goal이 활성화 중이면 취소 후 정지
        if self._goal_handle is not None and self.in_progress:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._zero_cmd_vel()

        goal = FollowWaypoints.Goal()
        goal.poses = poses

        # FollowWaypoints 서버가 준비될 때까지 대기
        self.client.wait_for_server()
        # 비동기 방식으로 goal 전송
        future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)

        self.in_progress = True
        self.paused = False
        self.waiting_ack = False

    # ──────────────────────────────────────────────────────────
    # /patrol/status 상태 브로드캐스트
    # ──────────────────────────────────────────────────────────
    def publish_status(self, event: str = ""):
        """
        현재 순찰 상태를 JSON 문자열로 구성하여 /patrol/status로 발행
        """
        payload = {
            "event": event,
            "task": self._current_task,
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
        # 한글이 포함되어도 그대로 출력되도록 설정
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)

    # ──────────────────────────────────────────────────────────
    # /patrol/cmd 명령 처리
    # ──────────────────────────────────────────────────────────
    def on_cmd(self, msg: String):
        """
        /patrol/cmd 콜백

        지원하는 명령 예시:
        - "start"               : 전체 루트 1회 순찰 후 자동 복귀
        - "start_once"          : 전체 루트 1회 순찰 후 자동 복귀
        - "start_repeat"        : 전체 루트 반복 순찰
        - "pause"               : 현재 순찰 일시정지
        - "stop"                : 순찰 완전 정지
        - "resume"              : pause 위치에서 순찰 재개
        - "return"              : 즉시 origin으로 복귀
        - "zone:A"              : A 구역을 1회 순찰 후 origin 복귀
        - "zone:A:repeat"       : A 구역 내부를 반복 순찰
        - "zones:A,B,C"         : A → B → C 순서로 1회 수행 후 origin 복귀
        - "zones:A,B,C:repeat"  : A → B → C 순서를 계속 반복 (origin 자동 복귀 X)
        - "manual_forward"      : 전진 수동 제어
        - "manual_backward"     : 후진 수동 제어
        - "manual_left"         : 좌회전 수동 제어
        - "manual_right"        : 우회전 수동 제어
        - "manual_stop"         : 수동 제어 정지
        """
        cmd = (msg.data or '').strip().lower()

        # 전체 루트 관련 명령
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

        # 단일 구역 명령: "zone:A" 또는 "zone:A:repeat"
        elif cmd.startswith('zone:'):
            parts = cmd.split(':')
            if len(parts) == 2:
                # 1회 순찰 후 origin 복귀
                self.start_zone(parts[1], repeat=False)
            elif len(parts) >= 3 and parts[2] == 'repeat':
                # 해당 구역 반복 순찰
                self.start_zone(parts[1], repeat=True)
            else:
                self.get_logger().warn(f'알 수 없는 zone 명령: "{cmd}"')

        # 구역 시퀀스 명령: "zones:A,B,C" 또는 "zones:A,B,C:repeat"
        elif cmd.startswith('zones:'):
            parts = cmd.split(':')
            if len(parts) == 2:
                seq_raw = parts[1]
                repeat = False
            elif len(parts) >= 3 and parts[2] == 'repeat':
                seq_raw = parts[1]
                repeat = True
            else:
                self.get_logger().warn(f'알 수 없는 zones 명령: "{cmd}"')
                return

            zone_names = [s.strip() for s in seq_raw.split(',') if s.strip()]
            self.start_zone_sequence(zone_names, repeat=repeat)

        # 수동 제어 명령
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
            self.get_logger().warn(f'알 수 없는 /patrol/cmd 명령입니다: "{cmd}"')

    # ──────────────────────────────────────────────────────────
    # 전체 루트 순찰 (1회 / 반복)
    # ──────────────────────────────────────────────────────────
    def start_once(self):
        """
        전체 base_route를 1회 순찰하고, 종료 후 origin으로 자동 복귀
        """
        if not self.base_route:
            self.get_logger().error('로드된 기본 웨이포인트가 없어 start_once를 수행할 수 없습니다.')
            return
        if self.in_progress:
            self.get_logger().info('이미 순찰이 실행 중이어서 start_once 명령을 무시합니다.')
            return

        # 새 순찰 실행을 위해 상태 초기화
        self._reset_state_for_new_run()
        self.route = list(self.base_route)
        self.current_zone = None
        self.zone_sequence = []

        self.repeat_mode = False
        self.auto_return = True
        self._current_task = 'route'

        # AMCL 상태에 따라 필요하면 initialpose 발행
        self._maybe_publish_initial_pose()

        # start_mode에 따른 시작 전략
        # - 'love': 현재 위치에서 바로 route 수행
        # - 'fixed': origin pose를 먼저 거친 뒤 route 수행
        plan = self.route if self.start_mode == 'live' else [self.make_origin_pose()] + self.route
        self.current_index = 0
        self.get_logger().info(f'[START ONCE] 웨이포인트 {len(plan)}개로 1회 순찰을 시작합니다. (auto_return=True)')
        self.send_follow_waypoints(plan)
        self.publish_status(event="start_route_once")

    def start_repeat(self):
        """
        전체 base_route를 반복 순찰
        (사용자가 stop 또는 return 명령을 내릴 때까지 반복)
        """
        if not self.base_route:
            self.get_logger().error('로드된 기본 웨이포인트가 없어 start_repeat를 수행할 수 없습니다.')
            return
        # 이미 전체 루트 반복 모드가 실행 중이면 재시작하지 않음
        if self.in_progress and self.repeat_mode and not self.current_zone and not self.zone_sequence:
            self.get_logger().info('이미 전체 루트 반복 순찰이 실행 중입니다.')
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
        self.get_logger().info(f'[START REPEAT] 웨이포인트 {len(plan)}개로 반복 순찰을 시작합니다.')
        self.send_follow_waypoints(plan)
        self.publish_status(event="start_route_repeat")

    # ──────────────────────────────────────────────────────────
    # 단일 ZONE 순찰
    # ──────────────────────────────────────────────────────────
    def start_zone(self, zone_name: str, repeat: bool = False):
        """
        특정 구역을 대상으로 순찰을 시작
        """
        if not self.zones:
            self.get_logger().error('[ZONE] zones 설정이 없어 구역 순찰을 수행할 수 없습니다.')
            return

        zkey = str(zone_name).upper()
        if zkey not in self.zones:
            self.get_logger().error(f'[ZONE] "{zkey}" 구역을 찾을 수 없습니다.')
            return

        # 이미 동일 존에 대해 repeat 모드로 순찰하고 있으면 재시작하지 않음
        if self.in_progress and self.repeat_mode and self.current_zone == zkey:
            self.get_logger().info(f'[ZONE {zkey}] 해당 구역 반복 순찰이 이미 실행 중입니다.')
            return

        z = self.zones[zkey]

        self._reset_state_for_new_run()

        # entry_pose → 구역 내부 웨이포인트 순으로 구성
        plan_core = [z['entry']] + z['route']
        self.route = list(plan_core)
        self.current_zone = zkey
        self.zone_sequence = []

        self.repeat_mode = repeat
        # repeat=False인 경우에만 자동 복귀
        self.auto_return = not repeat
        self._current_task = 'zone_route'

        self._maybe_publish_initial_pose()

        self.current_index = 0
        mode_txt = 'REPEAT' if repeat else 'ONCE'
        self.get_logger().info(f'[ZONE {zkey} {mode_txt}] 웨이포인트 {len(plan_core)}개로 구역 순찰을 시작합니다.')
        self.send_follow_waypoints(plan_core)
        self.publish_status(event=f"start_zone_{zkey}_{mode_txt.lower()}")

    # ──────────────────────────────────────────────────────────
    # ZONE 시퀀스 순찰 (예: A→B→C)
    # ──────────────────────────────────────────────────────────
    def start_zone_sequence(self, zone_names: List[str], repeat: bool = False):
        """
        여러 구역을 지정된 순서대로 순찰하는 시퀀스
        """
        if not self.zones:
            self.get_logger().error('[ZONES SEQ] zones 설정이 없어 구역 시퀀스를 수행할 수 없습니다.')
            return

        # 유효한 구역 이름만 정규화
        norm: List[str] = []
        for z in zone_names:
            zkey = str(z).upper()
            if not zkey:
                continue
            if zkey not in self.zones:
                self.get_logger().error(f'[ZONES SEQ] "{zkey}" 구역을 찾을 수 없습니다.')
                return
            norm.append(zkey)

        if not norm:
            self.get_logger().error('[ZONES SEQ] 유효한 구역 이름이 없어 시퀀스를 시작할 수 없습니다.')
            return

        self._reset_state_for_new_run()

        # 각 구역에 대해 entry_pose + route를 순서대로 이어붙임
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
            f'[ZONES SEQ {mode_txt}] {norm} 순서로 웨이포인트 {len(plan_core)}개 시퀀스를 시작합니다. '
            f'auto_return={self.auto_return}'
        )
        self.send_follow_waypoints(plan_core)
        self.publish_status(event=f"start_seq_{','.join(norm)}_{mode_txt.lower()}")

    # ──────────────────────────────────────────────────────────
    # 제어(일시정지, 정지, 재개, 즉시 복귀)
    # ──────────────────────────────────────────────────────────
    def pause_patrol(self):
        """
        현재 수행 중인 FollowWaypoints goal을 취소하여 순찰을 일시정지
        - 현재 위치에서 정지 후, resume_patrol() 호출 시 다음 웨이포인트부터 재개
        """
        if not self.in_progress:
            self.get_logger().info('일시정지할 순찰이 없어 pause 명령을 무시합니다.')
            return
        
        # on_result()에서 PAUSE로 인한 완료인지 구분하기 위해 플래그 설정
        self._pause_requested = True

        # 로봇을 즉시 멈추도록 cmd_vel 0 발행
        self._zero_cmd_vel()

        # FollowWaypoints goal 취소 요청
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass

        self.in_progress = False
        self.paused = True
        self.get_logger().info(f'순찰 일시정지를 요청했습니다.  (현재 인덱스={self.current_index})')
        self.publish_status(event="pause_requested")
    
    def stop_patrol(self):
        """
        순찰을 완전히 종료
        - repeat 모드, auto_return, zone/sequence 설정을 모두 초기화
        - FollowWaypoints goal을 취소하고, 충분히 0 속도를 발행하여 정지 보장
        """
        # 반복 및 자동 복귀 설정 해제
        self.repeat_mode = False
        self.auto_return = False
        self.current_zone = None
        self.zone_sequence = []

        # 현재 속도 명령을 0으로 여러 번 발행하여 즉시 정지
        self._zero_cmd_vel()

        # 실행 중인 FollowWaypoints goal이 있으면 취소
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass

        # 추가 안전을 위해 여러 번 0 속도 발행
        for _ in range(20):
            self.vel_pub.publish(Twist())
            time.sleep(0.02)

        self.in_progress = False
        self.paused = False
        self._current_task = 'none'
        self.get_logger().info(f'순찰을 완전히 정지했습니다.    (마지막 인덱스={self.current_index})')
        self.publish_status(event="stop")

    def resume_patrol(self):
        """
        pause_patrol()로 일시정지된 순찰을 재개
        - current_index + 1부터 route 끝까지 새로운 FollowWaypoints goal로 생성
        - 현재 모드(전체 루트/단일 구역/구역 시퀀스)에 따라 로그와 task 종류를 나누어 기록
        """
        if not self.route:
            self.get_logger().error('재개할 route 정보가 없어 resume을 수행할 수 없습니다.')
            return
        if self.in_progress:
            self.get_logger().info('이미 순찰이 실행 중이어서 resume 명령을 무시합니다.')
            return

        # 현재까지 도달한 인덱스 다음부터 새 plan 구성
        next_idx = min(self.current_index + 1, len(self.route) - 1)
        plan = self.route[next_idx:]
        if not plan:
            self.get_logger().info('재개할 남은 웨이포인트가 없어 resume을 수행하지 않습니다.')
            return

        # 모드에 따라 현재 task 태그를 설정
        if self.zone_sequence:
            self._current_task = 'zone_seq'
            self.get_logger().info(
                f'[ZONES SEQ {self.zone_sequence}] {next_idx}번 인덱스부터 순찰을 재개합니다. (남은 n={len(plan)})'
            )
        elif self.current_zone:
            self._current_task = 'zone_route'
            self.get_logger().info(
                f'[ZONE {self.current_zone}] {next_idx}번 인덱스부터 순찰을 재개합니다. (남은 n={len(plan)})'
            )
        else:
            self._current_task = 'route'
            self.get_logger().info(
                f'전체 루트 {next_idx}번 웨이포인트부터 순찰을 재개합니다. (남은 n={len(plan)})'
            )

        self.paused = False
        self.send_follow_waypoints(plan)
        self.publish_status(event="resume")

    def return_to_origin_immediate(self):
        """
        현재 순찰을 중단하고 즉시 origin(home_pose)으로 복귀
        - 반복 및 자동 복귀 모드를 해제하고,
          origin 포즈만 포함하는 FollowWaypoints goal을 새로 생성하여 실행
        """
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
        self.get_logger().info('현재 순찰을 중단하고 ORIGIN (home_pose)으로 즉시 복귀를 시작합니다.')
        self.send_follow_waypoints([origin])
        self.publish_status(event="return_immediate")

    def _reset_state_for_new_run(self):
        """
        새 순찰을 시작하기 전, 기존 액션/상태를 정리
        - 실행 중인 FollowWaypoints goal이 있으면 취소 후 정지
        - in_progess, paused, current_index, _goal_handel 등을 초기화
        - current_zone, zone_sequence도 초기화
        """
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
        """
        전압 값을 0~100%의 배터리 퍼센트로 선형 변환
        """
        if self.v_max <= self.v_min:
            return 0
        p = round(100.0 * (v - self.v_min) / (self.v_max - self.v_min))
        return max(0, min(100, p))

    def on_battery(self, msg: BatteryState):
        """
        /battery_state 콜백
        - 최근 avg_window 개수만큼 전압을 저장하고 이동 평균을 계산
        - 변환된 퍼센트가 low_percent 미만을 경우,
          한 번만 저전압 이벤트로 처리하여 origin 복귀를 수행
        """
        v = float(msg.voltage) if (msg.voltage is not None) else 0.0
        if v <= 0.0:
            return

        # 이동 평균 버퍼에 전압 추가
        self._volt_buf.append(v)
        if len(self._volt_buf) > self.avg_window:
            self._volt_buf.pop(0)
        v_avg = sum(self._volt_buf) / len(self._volt_buf)

        percent = self._volt_to_percent(v_avg)

        # 이미 저전압 자동 복귀가 수행된 이후라면 중복 처리하지 않음
        if self._low_batt_active:
            return

        if percent < self.low_percent:
            self._low_batt_active = True
            self.get_logger().warn(
                f'[LOW BATT] 배터리 {percent}% (이동 평균 {v_avg:.2f}V) 이하 → 순찰 중단 후 원점 복귀를 수행합니다.'
            )
            self.return_to_origin_immediate()
            self.publish_status(event="low_batt_auto_return")

    # ──────────────────────────────────────────────────────────
    # FollowWaypoints 액션 콜백
    # ──────────────────────────────────────────────────────────
    def on_goal_response(self, future):
        """
        FollowWaypoints goal 전송 이후 호출되는 응답 콜백
        - goal이 수락되면 _goal_handel을 저장하고,
          get_result_async()를 등록하여 완료 시 on_result()가 호출
        """
        self._goal_handle = future.result()
        if not self._goal_handle or not self._goal_handle.accepted:
            self.get_logger().error('FollowWaypoints goal이 서버에서 거부되었습니다.')
            self.in_progress = False
            self._goal_handle = None
            self._current_task = 'none'
            return
        self.get_logger().info(
            f'FollowWaypoints goal이 수락되었습니다. (task={self._current_task})'
        )
        self._goal_handle.get_result_async().add_done_callback(self.on_result)

    def on_feedback(self, feedback_msg):
        """
        FollowWaypoints 진행 중에 수신되는 피드백 콜백
        - 현재 도달한 웨이포인트 인덱스를 current_index에 갱신
        """
        try:
            self.current_index = int(feedback_msg.feedback.current_waypoint)
        except Exception:
            # 피드백 형식이 예상과 다르더라도 전체 동작에는 치명적이지 않으므로 무시
            pass

    def on_result(self, _future):
        """
        FollowWaypoints goal 완료(성공/실패/취소) 시 호출되는 콜백
        - 현재 _current_task 종류(route/zone_route/zone_seq/return)에 따라
          다음 동작(반복, origin 자동 복귀, 완료 후 대기 등)을 결정
        """
        # 1) PAUSE 요청에 의해 취소된 경우: 순찰은 '일시정지' 상태로 유지
        if self._pause_requested:
            self.get_logger().info(
                f'PAUSE 요청으로 인해 FollowWaypoints goal이 취소되었습니다. (task={self._current_task})'
            )
            self._pause_requested = False
            self.in_progress = False
            self._goal_handle = None
            self.get_logger().info('순찰이 일시정지 상태로 전환되었으며, resume 명령을 기다립니다.')
            self.publish_status(event="pause_goal_cancelled")
            return
        
        # 일반적인 완료 처리
        self.in_progress = False
        self._goal_handle = None
        self.get_logger().info(f'FollowWaypoints goal이 종료되었습니다. (task={self._current_task})')

        # 2) 전체 루트(route) 모드 처리
        if self._current_task == 'route':
            if self.repeat_mode:
                # 반복 모드인 경우, 동일한 route를 다시 실행
                plan = (
                    self.route
                    if self.start_mode == 'live'
                    else [self.make_origin_pose()] + self.route
                )
                self.current_index = 0
                self.get_logger().info('[REPEAT] 전체 루트 반복 순찰을 다시 시작합니다.')
                self._current_task = 'route'
                self.send_follow_waypoints(plan)
                self.publish_status(event="route_repeat_restart")
                return

            if self.auto_return:
                # 자동 복귀 설정이 켜져 있으면 origin으로 복귀
                origin = self.make_origin_pose()
                self._current_task = 'return'
                self.get_logger().info('[AUTO RETURN] 순찰이 끝나 origin으로 자동 복귀를 수행합니다.')
                self.send_follow_waypoints([origin])
                self.publish_status(event="route_auto_return")
                return

            # 반복도, 자동 복귀도 아닌 경우: 단순 종료
            self._current_task = 'none'
            self.get_logger().info('전체 루트 순찰을 마쳣으며, 다음 명령을 대기합니다.')
            self.publish_status(event="route_finished")
            return

        # 3) 단일 ZONE(zone_route) 모드 처리
        if self._current_task == 'zone_route':
            zname = self.current_zone
            if self.repeat_mode and zname and zname in self.zones:
                # 구역 반복 모드인 경우: 동일 구역 entry + route를 다시 실행
                z = self.zones[zname]
                plan = [z['entry']] + z['route']
                self.current_index = 0
                self.get_logger().info(f'[ZONE {zname}] 구역 반복 순찰을 다시 시작합니다.')
                self._current_task = 'zone_route'
                self.send_follow_waypoints(plan)
                self.publish_status(event=f"zone_{zname}_repeat_restart")
                return

            if self.auto_return:
                # 구역 순찰 완료 후 자동 origin 복귀
                origin = self.make_origin_pose()
                self._current_task = 'return'
                self.get_logger().info(
                    f'[ZONE {zname}] 순찰을 마쳐 origin으로 자동 복귀를 수행합니다.'
                )
                self.send_follow_waypoints([origin])
                self.publish_status(event=f"zone_{zname}_auto_return")
                return

            # 구역 1회 순찰 종료
            self.get_logger().info(f'[ZONE {zname}] 구역 순찰을 1회 완료했습니다.')
            self.current_zone = None
            self._current_task = 'none'
            self.get_logger().info('순찰이 종료되었으며, 다음 명령을 대기합니다.')
            self.publish_status(event=f"zone_{zname}_finished")
            return

        # 4) ZONE 시퀀스(zone_seq) 모드 처리
        if self._current_task == 'zone_seq':
            if self.repeat_mode and self.zone_sequence:
                # repeat=True인 경우 시퀀스 전체를 다시 시작
                plan = list(self.route)
                self.current_index = 0
                self.get_logger().info(
                    f'[ZONES SEQ REPEAT] {self.zone_sequence} 시퀀스 반복 순찰을 다시 시작합니다.'
                )
                self._current_task = 'zone_seq'
                self.send_follow_waypoints(plan)
                self.publish_status(event="zone_seq_repeat_restart")
                return

            if self.auto_return:
                # 시퀀스 한 바퀴 완료 후 origin 자동 복귀
                origin = self.make_origin_pose()
                self._current_task = 'return'
                self.get_logger().info(
                    f'[ZONES SEQ {self.zone_sequence}] 시퀀스를 마쳐 origin으로 자동 복귀를 수행합니다.'
                )
                self.send_follow_waypoints([origin])
                self.publish_status(event="zone_seq_auto_return")
                return

            # 시퀀스 1회 수행 종료
            self.get_logger().info(
                f'[ZONES SEQ {self.zone_sequence}] 구역 시퀀스 순찰을 1회 완료했습니다.'
            )
            self.zone_sequence = []
            self._current_task = 'none'
            self.get_logger().info('순찰이 종료되었으며, 다음 명령을 대기합니다.')
            self.publish_status(event="zone_seq_finished")
            return

        # 5) origin 복귀(return) 등 기타 경우
        self.current_zone = None
        self.zone_sequence = []
        self._current_task = 'none'
        self.get_logger().info('origin 복귀를 포함한 현재 작업이 모두 완료되었습니다. 다음 명령을 대기합니다.')
        self.publish_status(event="finished")

    # ──────────────────────────────────────────────────────────
    # Manual control (수동 조작)
    # ──────────────────────────────────────────────────────────
    def _on_manual_timer(self):
        """
        수동 모드가 활성화되어 있을 때, 주기적으로 cmd_vel을 발행
        - set_manual_motion()에서 manual_active=True로 설정되면
          이 타이머 콜백에서 manual_twist를 지속적으로 발행
        """
        if not self.manual_active:
            return
        self.vel_pub.publish(self.manual_twist)

    def set_manual_motion(self, lin_x: float, ang_z: float):
        """
        수동 전/후/좌/우 제어 시작
        - 기존에 수행 중이던 FollowWaypoints goal이 있으면 취소하고 정지한 뒤,
          manual_twist에 지정된 속도를 설정하여 manual_active 플래그를 킴
        """
        # 순찰/네비게이션 목표 있다면 우선 취소
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
            f'[MANUAL] 수동 모드 시작: 선속도={lin_x:.2f}, 각속도={ang_z:.2f}'
        )
        self.publish_status(event="manual_motion")

    def stop_manual(self):
        """
        수동 모드 정지
        - manual_active를 False로 설정하고,
          _zero_cmd_vel()을 호출하여 로봇을 완전히 멈춤
        """
        self.manual_active = False
        self.manual_twist = Twist()
        self._zero_cmd_vel()
        self.get_logger().info('[MANUAL] 수동 모드를 종료하고 로봇을 정지했습니다.')
        self.publish_status(event="manual_stop")

def main():
    """
    ROS2 노드 실행 엔트리 포인트
    """
    rclpy.init()
    node = PatrolManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
