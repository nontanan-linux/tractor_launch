#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster

import numpy as np
import math
import time
import httpx
from pyproj import Transformer
from typing import Optional

from kinematic_model import TractorTrailerModel

class AGVPoseDisplayNode(Node):
    def __init__(self):
        super().__init__('agv_pose_display_node')
        
        # ROS Parameters
        self.declare_parameter('namespace', 'agv1')
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('map_elevation', 0.0)
        self.declare_parameter('coordinate_mode', 'mgrs_grid')
        self.declare_parameter('map_origin_latitude', 12.662594744072885)
        self.declare_parameter('map_origin_longitude', 99.9208930823293)
        self.declare_parameter('display_rate_hz', 30.0)
        self.declare_parameter('use_server_velocity', True)
        self.declare_parameter('server_velocity_unit', 'kmh')
        self.declare_parameter('stationary_speed_threshold_mps', 0.15)
        self.declare_parameter('prediction_timeout_sec', 2.0)

        # Vehicle Dimensions
        self.declare_parameter('wheel_base', 1.400)
        self.declare_parameter('wheel_radius', 0.3)
        self.declare_parameter('wheel_width', 0.3)
        self.declare_parameter('wheel_tread', 1.0)
        self.declare_parameter('max_steer_angle', 0.70)
        self.declare_parameter('max_drawbar_angle', 0.70)
        self.declare_parameter('front_overhang', 0.85)
        self.declare_parameter('rear_overhang', 0.70)
        self.declare_parameter('left_overhang', 0.75)
        self.declare_parameter('right_overhang', 0.75)
        self.declare_parameter('hitch_length', 0.70)
        
        self.declare_parameter('trailer_wheel_radius', 0.33)
        self.declare_parameter('drawbar_track_width', 1.0)
        self.declare_parameter('drawbar_joint_length', 1.53)
        self.declare_parameter('drawbar_left_overhang', 0.50)
        self.declare_parameter('drawbar_right_overhang', 0.50)
        self.declare_parameter('drawbar_top_height', 0.70)
        
        self.declare_parameter('trailer_track_width', 1.0)
        self.declare_parameter('trailer_joint_length', 1.98)
        self.declare_parameter('trailer_left_overhang', 0.75)
        self.declare_parameter('trailer_right_overhang', 0.75)
        self.declare_parameter('trailer_hitch_length', 0.515)
        self.declare_parameter('trailer_thickness', 0.20)
        self.declare_parameter('hitch_height', 0.40)

        # Color Parameters (Kept for fallback or line markers)
        self.declare_parameter('color_tractor', [1.0, 0.27, 0.0, 0.5])
        self.declare_parameter('color_trailer', [0.0, 0.0, 1.0, 0.5])
        self.declare_parameter('color_wheel', [1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('color_drawbar', [0.8, 0.9, 1.0, 1.0])

        # API & Authentication Parameters
        self.declare_parameter('server_adds', '192.168.2.178')
        self.declare_parameter('server_port', 5000)
        self.declare_parameter('auth_endpoint', '/authentication/login')
        self.declare_parameter('position_endpoint', '/vehicle/vehicles')
        self.declare_parameter('username', 'admin3@minebea.co.th')
        self.declare_parameter('password', 'admin')
        self.declare_parameter('vehicle_name', 'AGV1')
        self.declare_parameter('state_filter', 'ALL')
        self.declare_parameter('poll_rate_hz', 2.0)

        # Get Parameters
        self.ns = self.get_parameter('namespace').value
        self.dt = self.get_parameter('dt').value
        self.map_elevation = self.get_parameter('map_elevation').value
        self.coordinate_mode = str(self.get_parameter('coordinate_mode').value).lower()
        if self.coordinate_mode not in ('mgrs_grid', 'local_cartesian_utm'):
            raise ValueError(
                "coordinate_mode must be 'mgrs_grid' or 'local_cartesian_utm'"
            )
        self.map_origin_latitude = float(self.get_parameter('map_origin_latitude').value)
        self.map_origin_longitude = float(self.get_parameter('map_origin_longitude').value)
        self.display_rate_hz = max(1.0, float(self.get_parameter('display_rate_hz').value))
        self.use_server_velocity = bool(self.get_parameter('use_server_velocity').value)
        self.server_velocity_unit = str(
            self.get_parameter('server_velocity_unit').value
        ).lower()
        if self.server_velocity_unit not in ('kmh', 'mps'):
            raise ValueError("server_velocity_unit must be 'kmh' or 'mps'")
        self.stationary_speed_threshold = max(
            0.0,
            float(self.get_parameter('stationary_speed_threshold_mps').value),
        )
        self.prediction_timeout = max(
            0.1, float(self.get_parameter('prediction_timeout_sec').value)
        )
        self.utm_zone = max(1, min(60, int((self.map_origin_longitude + 180.0) / 6.0) + 1))
        utm_epsg = (32600 if self.map_origin_latitude >= 0.0 else 32700) + self.utm_zone
        self.wgs84_to_utm = Transformer.from_crs('EPSG:4326', f'EPSG:{utm_epsg}', always_xy=True)
        self.map_origin_easting, self.map_origin_northing = self.wgs84_to_utm.transform(
            self.map_origin_longitude, self.map_origin_latitude
        )
        self.L0 = self.get_parameter('wheel_base').value
        self.wheel_diam = self.get_parameter('wheel_radius').value * 2.0
        self.W = self.get_parameter('wheel_tread').value
        self.trailer_wheel_radius = self.get_parameter('trailer_wheel_radius').value
        self.db_tw = self.get_parameter('drawbar_track_width').value
        self.tl_tw = self.get_parameter('trailer_track_width').value

        self.color_tractor = self.get_parameter('color_tractor').value
        self.color_trailer = self.get_parameter('color_trailer').value
        self.color_wheel = self.get_parameter('color_wheel').value
        self.color_drawbar = self.get_parameter('color_drawbar').value

        # Trailer Structure Setup
        db_len = self.get_parameter('drawbar_joint_length').value
        tr_len = self.get_parameter('trailer_joint_length').value
        hitch_len = self.get_parameter('hitch_length').value
        tr_hitch_len = self.get_parameter('trailer_hitch_length').value
        
        num_trailers = 4
        l_bars = [db_len] * num_trailers
        l_trls = [tr_len] * num_trailers
        dh_prevs = [hitch_len] + [tr_hitch_len] * (num_trailers - 1)

        self.trailers = []
        for i in range(num_trailers):
            self.trailers.append({
                'L_bar': l_bars[i],
                'L_trl': l_trls[i],
                'dh_prev': dh_prevs[i]
            })

        max_steer_angle = self.get_parameter('max_steer_angle').value
        max_drawbar_angle = self.get_parameter('max_drawbar_angle').value

        # Kinematics Model
        self.model = TractorTrailerModel(
            self.L0, 
            self.trailers, 
            dt=self.dt, 
            max_steering_angle=max_steer_angle, 
            max_drawbar_angle=max_drawbar_angle
        )
        
        self.state = np.zeros(3 + 2 * len(self.trailers))
        self.wheel_phi = np.zeros(2 + len(self.trailers))
        self.v0 = 0.0
        self.yaw_rate = 0.0
        self.delta = 0.0
        self.commanded_speed = 0.0
        self.commanded_steering = 0.0
        self.last_yaw_measurement = None
        self.last_yaw_measurement_time = None
        self.pose_initialized = False
        self.last_update_time = None
        
        self.last_api_pos = None
        self.last_api_time = None
        self.last_api_receive_time = None
        self.last_position_change_time = None
        self.motion_confirmed = False
        self.marker_id_counter = 0

        # Setup HTTP Client
        self.server_adds = self.get_parameter('server_adds').value
        self.server_port = self.get_parameter('server_port').value
        self.auth_endpoint = self.get_parameter('auth_endpoint').value
        self.position_endpoint = self.get_parameter('position_endpoint').value
        self.username = self.get_parameter('username').value
        self.password = self.get_parameter('password').value
        self.vehicle_name = self.get_parameter('vehicle_name').value
        self.state_filter = self.get_parameter('state_filter').value
        
        self.login_url = f"http://{self.server_adds}:{self.server_port}{self.auth_endpoint}"
        self.position_url = f"http://{self.server_adds}:{self.server_port}{self.position_endpoint}"
        
        self.http_client = httpx.Client(timeout=2.0)
        self.auth_token = None
        
        # Initial Authentication
        self.login()

        # Publishers & Broadcasters
        self.pub_joints = self.create_publisher(JointState, f'/{self.ns}/joint_states', 10)
        self.pub_markers = self.create_publisher(MarkerArray, f'/{self.ns}/vehicle_markers', 10)
        self.pub_agv_pose = self.create_publisher(PoseStamped, f'/{self.ns}/pose', 10)
        self.pub_agv_odometry = self.create_publisher(Odometry, f'/{self.ns}/odometry', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Manual Initial Pose Subscriber
        self.sub_initial = self.create_subscription(
            PoseWithCovarianceStamped, f'/{self.ns}/initialpose', self.initial_pose_callback, 10
        )

        # Timers
        poll_rate = self.get_parameter('poll_rate_hz').value
        self.api_timer = self.create_timer(1.0 / poll_rate, self.get_api_pose)

        system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.timer = self.create_timer(1.0 / self.display_rate_hz, self.update, clock=system_clock)

        self.get_logger().info(f'AGV Pose Display Node initialized for {self.vehicle_name} (Topic NS: {self.ns})')

    def login(self) -> bool:
        """Authenticate with username and password to retrieve JWT access token."""
        headers = {
            'accept': 'application/json',
            'Content-Type': 'application/x-www-form-urlencoded'
        }
        form_data = {
            'grant_type': 'password',
            'username': self.username,
            'password': self.password,
            'scope': '',
            'client_id': 'string',
            'client_secret': 'string'
        }

        try:
            response = self.http_client.post(self.login_url, headers=headers, data=form_data)
            if response.status_code == 200:
                data = response.json()
                self.auth_token = data.get('access_token') or data.get('payload', {}).get('access_token')
                if self.auth_token:
                    self.get_logger().info(f'Successfully logged in as {self.username}. Token acquired.')
                    return True
                else:
                    self.get_logger().error('Login succeeded but no access_token found in response body.')
                    return False
            else:
                self.get_logger().error(f'Login failed: HTTP {response.status_code} - {response.text}', throttle_duration_sec=3.0)
                return False
        except Exception as exc:
            self.get_logger().error(f'Login connection error: {exc}', throttle_duration_sec=3.0)
            return False

    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float):
        """Convert Roll-Pitch-Yaw angles to quaternion list."""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def publish_vehicle_state(self, x: float, y: float, yaw: float, timestamp):
        """Publish pose and velocity derived from consecutive API positions."""
        q = self.euler_to_quaternion(0.0, 0.0, yaw)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = float(y)
        pose_msg.pose.position.z = float(self.map_elevation)
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        self.pub_agv_pose.publish(pose_msg)

        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.child_frame_id = f'{self.ns}_base_link'
        odom_msg.pose.pose = pose_msg.pose
        # Twist is expressed in child_frame_id: +x is forward along the tractor.
        odom_msg.twist.twist.linear.x = float(self.v0)
        odom_msg.twist.twist.angular.z = float(self.yaw_rate)
        self.pub_agv_odometry.publish(odom_msg)

    def to_map_coordinate(self, cx: float, cy: float, px: float, py: float, theta: float):
        """Transform local CAD coordinate (cx, cy) to global map coordinate."""
        mx = px + cx * math.cos(theta) - cy * math.sin(theta)
        my = py + cx * math.sin(theta) + cy * math.cos(theta)
        return mx, my

    def wheel_mesh_position(self, hub_x, hub_y, yaw, spin, z, center):
        """Place a CAD wheel so its measured mesh center remains at the hub."""
        cx, cy, cz = center  # Mesh center after the fixed 90-degree roll and 0.01 scale.
        spun_x = math.cos(spin) * cx + math.sin(spin) * cz
        spun_z = -math.sin(spin) * cx + math.cos(spin) * cz
        c, s = math.cos(yaw), math.sin(yaw)
        return (hub_x - c * spun_x + s * cy,
                hub_y - s * spun_x - c * cy,
                z + cz - spun_z)

    def broadcast_relative_tf(self, parent_frame, child_frame, parent_xy, child_xy,
                              parent_yaw, child_yaw, stamp):
        """Publish a hitch link in its parent's moving coordinate frame."""
        dx = float(child_xy[0] - parent_xy[0])
        dy = float(child_xy[1] - parent_xy[1])
        c, s = math.cos(parent_yaw), math.sin(parent_yaw)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = c * dx + s * dy
        t.transform.translation.y = -s * dx + c * dy
        t.transform.rotation.z = math.sin((child_yaw - parent_yaw) / 2.0)
        t.transform.rotation.w = math.cos((child_yaw - parent_yaw) / 2.0)
        self.tf_broadcaster.sendTransform(t)

    def broadcast_hitch_tf(self, roll: float, pitch: float, yaw: float, child_frame: str, parent_frame: str, tf_timestamp):
        """Broadcast a TF transform for hitch connections with namespace prefix."""
        t_h = TransformStamped()
        t_h.header.stamp = tf_timestamp
        t_h.header.frame_id = parent_frame
        t_h.child_frame_id = child_frame
        t_h.transform.translation.x = 0.0
        t_h.transform.translation.y = 0.0
        t_h.transform.translation.z = 0.0
        q_h = self.euler_to_quaternion(roll, pitch, yaw)
        t_h.transform.rotation.x = q_h[0]
        t_h.transform.rotation.y = q_h[1]
        t_h.transform.rotation.z = q_h[2]
        t_h.transform.rotation.w = q_h[3]
        self.tf_broadcaster.sendTransform(t_h)

    def create_mesh_marker(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float, mesh_path: str, timestamp, frame_id=None):
        """Construct a mesh marker relative to the tractor base frame."""
        if frame_id is None:
            frame_id = f'{self.ns}_base_link'
            dx = x - self.state[0]
            dy = y - self.state[1]
            c = math.cos(self.state[2])
            s = math.sin(self.state[2])
            x = c * dx + s * dy
            y = -s * dx + c * dy
            z -= self.map_elevation
            yaw = self.normalize_angle(yaw - self.state[2])
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = timestamp
        m.ns = "meshes"
        m.id = self.marker_id_counter
        self.marker_id_counter += 1
        m.type = Marker.MESH_RESOURCE
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        q_m = self.euler_to_quaternion(roll, pitch, yaw)
        m.pose.orientation.x = q_m[0]
        m.pose.orientation.y = q_m[1]
        m.pose.orientation.z = q_m[2]
        m.pose.orientation.w = q_m[3]
        m.scale.x = 0.01
        m.scale.y = 0.01
        m.scale.z = 0.01
        m.frame_locked = True
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 1.0
        m.mesh_resource = mesh_path
        m.mesh_use_embedded_materials = True
        return m

    def initial_pose_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta0 = math.atan2(siny_cosp, cosy_cosp)
        
        self.state[0] = msg.pose.pose.position.x
        self.state[1] = msg.pose.pose.position.y
        self.state[2] = theta0
        for i in range(len(self.trailers)):
            self.state[3 + 2*i] = theta0
            self.state[3 + 2*i + 1] = theta0
        
        self.last_api_pos = None
        self.last_api_time = None
        self.last_api_receive_time = None
        self.last_position_change_time = None
        self.motion_confirmed = False
        self.pose_initialized = True
        self.v0 = 0.0
        self.commanded_speed = 0.0
        self.commanded_steering = 0.0
        self.last_yaw_measurement = theta0
        self.last_yaw_measurement_time = time.monotonic()
        self.yaw_rate = 0.0
        self.delta = 0.0

    def get_api_pose(self):
        """Fetches latest pose from server. Automatically re-logs in if unauthenticated."""
        if not self.auth_token:
            self.login()
            return

        headers = {
            'accept': 'application/json',
            'Authorization': f'Bearer {self.auth_token}'
        }
        params = {
            'vehicle_name': self.vehicle_name,
            'state': self.state_filter
        }

        try:
            response = self.http_client.get(self.position_url, headers=headers, params=params)

            # Auto re-login if token expired or invalid
            if response.status_code == 401:
                self.get_logger().warn('Token expired (HTTP 401). Refreshing token...')
                if self.login():
                    headers['Authorization'] = f'Bearer {self.auth_token}'
                    response = self.http_client.get(self.position_url, headers=headers, params=params)
                else:
                    return

            if response.status_code != 200:
                return

            res_json = response.json()
            payload_list = res_json.get('payload', [])
            if not payload_list:
                return

            vehicle_data = payload_list[0]
            server_velocity_mps = None
            if self.use_server_velocity:
                try:
                    velocity_value = vehicle_data.get('velocity')
                    if velocity_value is not None and velocity_value != '':
                        server_velocity_mps = float(velocity_value)
                        if self.server_velocity_unit == 'kmh':
                            server_velocity_mps /= 3.6
                        if not math.isfinite(server_velocity_mps):
                            server_velocity_mps = None
                except (TypeError, ValueError):
                    server_velocity_mps = None

            coord_raw = vehicle_data.get('coordinate', None)
            if coord_raw is None:
                return

            if isinstance(coord_raw, list):
                parts = [float(val) for val in coord_raw]
            elif isinstance(coord_raw, str):
                parts = [float(val.strip()) for val in coord_raw.split(',') if val.strip()]
            else:
                return

            if len(parts) < 3:
                return

            raw_x = parts[0]
            raw_y = parts[1]
            new_yaw = self.normalize_angle(parts[2])

            # The current server may still return latitude/longitude, while the
            # newer payload returns map x/y directly. Geographic values are
            # converted once; map coordinates pass through unchanged.
            if -90.0 <= raw_x <= 90.0 and -180.0 <= raw_y <= 180.0:
                try:
                    new_x, new_y = self.convert_latlon_to_map(raw_x, raw_y)
                except ValueError as exc:
                    self.get_logger().error(
                        f'Failed to convert latitude/longitude: {exc}'
                    )
                    return
            else:
                new_x, new_y = raw_x, raw_y

            now_sec = time.monotonic()
            self.last_api_receive_time = now_sec

            measured_speed = None
            if server_velocity_mps is not None:
                measured_speed = float(np.clip(
                    server_velocity_mps, 0.0, self.model.max_velocity
                ))

            # The first valid API coordinate is the initial pose. From this
            # point onward x/y never corrects the displayed pose; the model is
            # advanced only by non-negative speed and steering.
            if not self.pose_initialized:
                self.state[0] = new_x
                self.state[1] = new_y
                self.state[2] = new_yaw
                for i in range(len(self.trailers)):
                    self.state[3 + 2*i] = new_yaw
                    self.state[3 + 2*i + 1] = new_yaw
                # The first sample is an initial pose only. A later position
                # change must confirm motion before server velocity is used.
                self.commanded_speed = 0.0
                self.commanded_steering = 0.0
                self.motion_confirmed = False
                self.last_yaw_measurement = new_yaw
                self.last_yaw_measurement_time = now_sec
                self.pose_initialized = True
                self.last_api_pos = (new_x, new_y, new_yaw)
                self.last_api_time = now_sec
                self.last_position_change_time = now_sec
                return

            position_changed = (
                self.last_api_pos is None
                or new_x != self.last_api_pos[0]
                or new_y != self.last_api_pos[1]
            )
            sample_changed = (
                position_changed
                or new_yaw != self.last_api_pos[2]
            )

            if position_changed:
                self.last_position_change_time = now_sec
                if (measured_speed is not None and
                        measured_speed >= self.stationary_speed_threshold):
                    self.motion_confirmed = True

            # The server can retain a non-zero velocity after the coordinate
            # has stopped. A deadband freezes every state, while a non-zero
            # velocity is accepted only after x/y has confirmed movement.
            if measured_speed is not None:
                if measured_speed < self.stationary_speed_threshold:
                    self.commanded_speed = 0.0
                    self.commanded_steering = 0.0
                    self.motion_confirmed = False
                elif self.motion_confirmed:
                    self.commanded_speed = measured_speed

            # Polling can return the exact same record more than once. Keep the
            # previous steering for duplicate records, otherwise the duplicated
            # yaw would be interpreted as a new straight-line measurement.
            if sample_changed:
                yaw_dt = now_sec - self.last_yaw_measurement_time
                yaw_change = self.normalize_angle(
                    new_yaw - self.last_yaw_measurement
                )

                if (yaw_dt > 1.0e-3 and
                        self.commanded_speed >= self.stationary_speed_threshold):
                    measured_yaw_rate = yaw_change / yaw_dt
                    steering = math.atan(
                        self.L0 * measured_yaw_rate / self.commanded_speed
                    )
                    self.commanded_steering = float(np.clip(
                        steering,
                        -self.model.max_steering_angle,
                        self.model.max_steering_angle,
                    ))
                else:
                    self.commanded_steering = 0.0

                self.last_yaw_measurement = new_yaw
                self.last_yaw_measurement_time = now_sec
                self.last_api_pos = (new_x, new_y, new_yaw)
                self.last_api_time = now_sec

        except Exception as exc:
            self.get_logger().error(f'API Polling error: {exc}', throttle_duration_sec=2.0)

    def update(self):
        """
        Full Standalone Update Loop using 2-Step Transform for Mesh Pivot Alignment.
        """
        now_sec = time.monotonic()
        if self.last_update_time is None:
            update_dt = 1.0 / self.display_rate_hz
        else:
            update_dt = min(max(now_sec - self.last_update_time, 0.001), 0.25)
        self.last_update_time = now_sec

        if not self.pose_initialized:
            return

        # Stop if either the API disappears or a supposedly moving vehicle's
        # x/y remains unchanged. Yaw noise alone must not keep motion alive.
        api_stale = (
            self.last_api_receive_time is not None
            and now_sec - self.last_api_receive_time > self.prediction_timeout
        )
        position_stale = (
            self.motion_confirmed
            and self.last_position_change_time is not None
            and now_sec - self.last_position_change_time > self.prediction_timeout
        )
        if api_stale or position_stale:
            self.commanded_speed = 0.0
            self.commanded_steering = 0.0
            self.motion_confirmed = False

        # Continuous forward-only dead reckoning. The same kinematic update
        # propagates the tractor motion through every drawbar and trailer joint.
        self.v0 = max(0.0, self.commanded_speed)
        if self.v0 >= self.stationary_speed_threshold:
            self.delta = float(np.clip(
                self.commanded_steering,
                -self.model.max_steering_angle,
                self.model.max_steering_angle,
            ))
            self.yaw_rate = self.v0 / self.L0 * math.tan(self.delta)
        else:
            self.yaw_rate = 0.0
            self.delta = 0.0

        self.model.dt = update_dt
        self.state = self.model.update(self.state, self.v0, self.delta)

        # Extract CAD reference coordinates from kinematic model
        coords = self.model.get_coordinates(self.state)
        
        # Unified system clock timestamp
        sys_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        current_stamp = sys_clock.now().to_msg()
        self.publish_vehicle_state(
            self.state[0], self.state[1], self.state[2], current_stamp
        )

        # 2. Broadcast the global tractor pose. Mesh markers are expressed in
        # this base frame so RViz never stores large coordinates per mesh.
        t = TransformStamped()
        t.header.stamp = current_stamp
        t.header.frame_id = 'map'
        t.child_frame_id = f'{self.ns}_base_link'
        t.transform.translation.x = float(self.state[0])
        t.transform.translation.y = float(self.state[1])
        t.transform.translation.z = float(self.map_elevation)
        q = self.euler_to_quaternion(0.0, 0.0, self.state[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # 3. Wheel Rotation Integration
        v_tr_rear = self.v0
        self.wheel_phi[0] += (v_tr_rear / (self.wheel_diam / 2.0)) * update_dt
        v_tr_front = self.v0 / math.cos(self.delta) if math.cos(self.delta) != 0 else self.v0
        self.wheel_phi[1] += (v_tr_front / (self.wheel_diam / 2.0)) * update_dt
        for i in range(len(self.trailers)):
            self.wheel_phi[2 + i] += (self.v0 / (self.wheel_diam / 2.0)) * update_dt

        # 4. Generate Visual 3D Meshes via MarkerArray
        marker_array = MarkerArray()
        self.marker_id_counter = 0

        # CAD Origin Offsets (x, y, z)
        tr_cad = (0.850064, -0.001778, 0.0)
        db_cad = (3.08664, -0.001778, 0.0)
        tl_cad = (5.08157, -0.001778, 0.0)
        mesh_rpy = (1.570796, 0.0, 0.0)
        # Centers measured from the OBJ vertex bounds after scale and fixed roll.
        wheel_centers = {
            'fl': (0.550311, 0.498933, 0.270015),
            'fr': (0.550311, -0.495179, 0.270015),
            'rl': (-0.850064, 0.497945, 0.331764),
            'rr': (-0.850064, -0.494389, 0.331764),
            'dl': (-3.086639, 0.497945, 0.331764),
            'dr': (-3.086639, -0.494389, 0.331764),
            'tl': (-5.081575, 0.497945, 0.331764),
            'tr': (-5.081575, -0.494389, 0.331764),
        }
        
        map_z = float(self.map_elevation)
        p0 = coords[0]
        theta0 = self.state[2]

        # --- Tractor Chassis ---
        tr_x, tr_y = self.to_map_coordinate(tr_cad[0], tr_cad[1], p0[0], p0[1], theta0)
        marker_array.markers.append(self.create_mesh_marker(
            tr_x, tr_y, map_z, mesh_rpy[0], mesh_rpy[1], theta0,
            'package://tractor_description/mesh/TowTractor_Chassis.obj', current_stamp
        ))

        # --- Tractor Wheels ---
        wb = self.L0
        tw = self.W
        steer = self.delta

        # Front-Left Wheel
        fl_x, fl_y = self.to_map_coordinate(wb, tw / 2.0, p0[0], p0[1], theta0)
        fl_mx, fl_my, fl_mz = self.wheel_mesh_position(fl_x, fl_y, theta0 + steer, self.wheel_phi[1], map_z, wheel_centers['fl'])
        marker_array.markers.append(self.create_mesh_marker(
            fl_mx, fl_my, fl_mz, mesh_rpy[0], mesh_rpy[1] + self.wheel_phi[1], theta0 + steer,
            'package://tractor_description/mesh/TowTractor_FL_Wheel.obj', current_stamp
        ))

        # Front-Right Wheel
        fr_x, fr_y = self.to_map_coordinate(wb, -tw / 2.0, p0[0], p0[1], theta0)
        fr_mx, fr_my, fr_mz = self.wheel_mesh_position(fr_x, fr_y, theta0 + steer, self.wheel_phi[1], map_z, wheel_centers['fr'])
        marker_array.markers.append(self.create_mesh_marker(
            fr_mx, fr_my, fr_mz, mesh_rpy[0], mesh_rpy[1] + self.wheel_phi[1], theta0 + steer,
            'package://tractor_description/mesh/TowTractor_FR_Wheel.obj', current_stamp
        ))

        # Rear-Left Wheel
        rl_x, rl_y = self.to_map_coordinate(0.0, tw / 2.0, p0[0], p0[1], theta0)
        rl_mx, rl_my, rl_mz = self.wheel_mesh_position(rl_x, rl_y, theta0, self.wheel_phi[0], map_z, wheel_centers['rl'])
        marker_array.markers.append(self.create_mesh_marker(
            rl_mx, rl_my, rl_mz, mesh_rpy[0], mesh_rpy[1] + self.wheel_phi[0], theta0,
            'package://tractor_description/mesh/TowTractor_RL_Wheel.obj', current_stamp
        ))

        # Rear-Right Wheel
        rr_x, rr_y = self.to_map_coordinate(0.0, -tw / 2.0, p0[0], p0[1], theta0)
        rr_mx, rr_my, rr_mz = self.wheel_mesh_position(rr_x, rr_y, theta0, self.wheel_phi[0], map_z, wheel_centers['rr'])
        marker_array.markers.append(self.create_mesh_marker(
            rr_mx, rr_my, rr_mz, mesh_rpy[0], mesh_rpy[1] + self.wheel_phi[0], theta0,
            'package://tractor_description/mesh/TowTractor_RR_Wheel.obj', current_stamp
        ))

        # --- Trailers and Drawbars (1 to 4) ---
        for k in range(len(self.trailers)):
            p_dolly = coords[3 + 3 * k]
            p_axle = coords[4 + 3 * k]
            theta_drawbar = self.state[3 + 2 * k]
            theta_trailer = self.state[3 + 2 * k + 1]

            parent_xy = coords[0] if k == 0 else coords[4 + 3 * (k - 1)]
            parent_yaw = self.state[2] if k == 0 else self.state[3 + 2 * (k - 1) + 1]
            parent_frame = f'{self.ns}_base_link' if k == 0 else f'{self.ns}_trailer_{k}_link'
            drawbar_frame = f'{self.ns}_drawbar_{k + 1}_link'
            trailer_frame = f'{self.ns}_trailer_{k + 1}_link'
            self.broadcast_relative_tf(parent_frame, drawbar_frame, parent_xy, p_dolly,
                                       parent_yaw, theta_drawbar, current_stamp)
            self.broadcast_relative_tf(drawbar_frame, trailer_frame, p_dolly, p_axle,
                                       theta_drawbar, theta_trailer, current_stamp)

            # Drawbar Body
            db_x, db_y = self.to_map_coordinate(db_cad[0], db_cad[1], p_dolly[0], p_dolly[1], theta_drawbar)
            marker_array.markers.append(self.create_mesh_marker(
                db_x, db_y, map_z, mesh_rpy[0], mesh_rpy[1], theta_drawbar,
                'package://tractor_description/mesh/Drawbar.obj', current_stamp
            ))

            # Drawbar Left Wheel
            dl_x, dl_y = self.to_map_coordinate(0.0, self.db_tw / 2.0, p_dolly[0], p_dolly[1], theta_drawbar)
            dl_mx, dl_my, dl_mz = self.wheel_mesh_position(dl_x, dl_y, theta_drawbar, self.wheel_phi[2 + k], map_z, wheel_centers['dl'])
            marker_array.markers.append(self.create_mesh_marker(
                dl_mx, dl_my, dl_mz, mesh_rpy[0], mesh_rpy[1] + self.wheel_phi[2 + k], theta_drawbar,
                'package://tractor_description/mesh/DL_Wheel.obj', current_stamp
            ))

            # Drawbar Right Wheel
            dr_x, dr_y = self.to_map_coordinate(0.0, -self.db_tw / 2.0, p_dolly[0], p_dolly[1], theta_drawbar)
            dr_mx, dr_my, dr_mz = self.wheel_mesh_position(dr_x, dr_y, theta_drawbar, self.wheel_phi[2 + k], map_z, wheel_centers['dr'])
            marker_array.markers.append(self.create_mesh_marker(
                dr_mx, dr_my, dr_mz, mesh_rpy[0], mesh_rpy[1] + self.wheel_phi[2 + k], theta_drawbar,
                'package://tractor_description/mesh/DR_Wheel.obj', current_stamp
            ))

            # Trailer Chassis
            tl_x, tl_y = self.to_map_coordinate(tl_cad[0], tl_cad[1], p_axle[0], p_axle[1], theta_trailer)
            marker_array.markers.append(self.create_mesh_marker(
                tl_x, tl_y, map_z, mesh_rpy[0], mesh_rpy[1], theta_trailer,
                'package://tractor_description/mesh/Trailerl.obj', current_stamp
            ))

            # Trailer Left Wheel
            tl_wl_x, tl_wl_y = self.to_map_coordinate(0.0, self.tl_tw / 2.0, p_axle[0], p_axle[1], theta_trailer)
            tl_mx, tl_my, tl_mz = self.wheel_mesh_position(tl_wl_x, tl_wl_y, theta_trailer, self.wheel_phi[2 + k], map_z, wheel_centers['tl'])
            marker_array.markers.append(self.create_mesh_marker(
                tl_mx, tl_my, tl_mz, mesh_rpy[0], mesh_rpy[1] + self.wheel_phi[2 + k], theta_trailer,
                'package://tractor_description/mesh/TL_Wheel.obj', current_stamp
            ))

            # Trailer Right Wheel
            tr_wl_x, tr_wl_y = self.to_map_coordinate(0.0, -self.tl_tw / 2.0, p_axle[0], p_axle[1], theta_trailer)
            tr_mx, tr_my, tr_mz = self.wheel_mesh_position(tr_wl_x, tr_wl_y, theta_trailer, self.wheel_phi[2 + k], map_z, wheel_centers['tr'])
            marker_array.markers.append(self.create_mesh_marker(
                tr_mx, tr_my, tr_mz, mesh_rpy[0], mesh_rpy[1] + self.wheel_phi[2 + k], theta_trailer,
                'package://tractor_description/mesh/TR_Wheel.obj', current_stamp
            ))

        # 5. Publish MarkerArray Topic
        self.pub_markers.publish(marker_array)

    def convert_latlon_to_map(self, lat: float, lon: float):
        """Convert WGS84 directly to the selected Cartesian map frame."""
        if not (-80.0 <= lat <= 84.0 and -180.0 <= lon <= 180.0):
            raise ValueError(f'coordinate outside UTM bounds: lat={lat}, lon={lon}')

        zone = max(1, min(60, int((lon + 180.0) / 6.0) + 1))
        if zone != self.utm_zone or (lat >= 0.0) != (self.map_origin_latitude >= 0.0):
            raise ValueError(
                f'coordinate is outside map UTM zone {self.utm_zone}: lat={lat}, lon={lon}'
            )

        easting, northing = self.wgs84_to_utm.transform(lon, lat)
        if self.coordinate_mode == 'mgrs_grid':
            # The map uses the numeric part inside the current 100 km MGRS square.
            # Keep UTM fractions; converting through an MGRS string would truncate them.
            x = easting % 100000.0
            y = northing % 100000.0
        else:
            x = easting - self.map_origin_easting
            y = northing - self.map_origin_northing

        # Remove floating-point noise such as 34832.99999998 without discarding
        # real sub-meter precision received from latitude/longitude.
        if abs(x - round(x)) < 1e-6:
            x = float(round(x))
        if abs(y - round(y)) < 1e-6:
            y = float(round(y))
        return x, y

    def destroy_node(self):
        self.http_client.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AGVPoseDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
