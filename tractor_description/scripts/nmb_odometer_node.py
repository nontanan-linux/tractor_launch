#!/usr/bin/env python3
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
import math
from kinematic_model import TractorTrailerModel
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Point, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray

class NMBOdometer(Node):
    def __init__(self):
        super().__init__('nmb_odometer')
        
        # Load specific parameters matching vehicle_info.param.yaml
        self.declare_parameter('wheel_base', 1.400)
        self.declare_parameter('dt', 0.05)
        self.declare_parameter('map_elevation', 0.0)
        
        self.declare_parameter('wheel_radius', 0.3)
        self.declare_parameter('wheel_width', 0.3)
        self.declare_parameter('wheel_tread', 1.0)
        
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
        
        # Color Parameters (Still kept for fallback or line markers)
        self.declare_parameter('color_tractor', [1.0, 0.27, 0.0, 0.5])
        self.declare_parameter('color_trailer', [0.0, 0.0, 1.0, 0.5])
        self.declare_parameter('color_wheel', [1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('color_drawbar', [0.8, 0.9, 1.0, 1.0])

        self.L0 = self.get_parameter('wheel_base').value
        self.dt = self.get_parameter('dt').value
        dt = self.dt
        
        self.tractor_width = self.get_parameter('wheel_tread').value + self.get_parameter('left_overhang').value + self.get_parameter('right_overhang').value
        self.tractor_front_overhang = self.get_parameter('front_overhang').value
        self.tractor_overhang = self.get_parameter('rear_overhang').value
        
        self.wheel_diam = self.get_parameter('wheel_radius').value * 2.0
        self.wheel_width = self.get_parameter('wheel_width').value
        self.W = self.get_parameter('wheel_tread').value
        
        self.trailer_wheel_radius = self.get_parameter('trailer_wheel_radius').value
        
        db_len = self.get_parameter('drawbar_joint_length').value
        tr_len = self.get_parameter('trailer_joint_length').value
        hitch_len = self.get_parameter('hitch_length').value
        tr_hitch_len = self.get_parameter('trailer_hitch_length').value
        
        num_trailers = 4
        l_bars = [db_len] * num_trailers
        l_trls = [tr_len] * num_trailers
        dh_prevs = [hitch_len] + [tr_hitch_len] * (num_trailers - 1)

        
        self.c_tractor = self.get_parameter('color_tractor').value
        self.c_trailer = self.get_parameter('color_trailer').value
        self.c_wheel = self.get_parameter('color_wheel').value
        self.c_drawbar = self.get_parameter('color_drawbar').value
        
        self.trailers = []
        if len(l_bars) == len(l_trls) == len(dh_prevs):
            for i in range(len(l_bars)):
                self.trailers.append({
                    'L_bar': l_bars[i],
                    'L_trl': l_trls[i],
                    'dh_prev': dh_prevs[i]
                })
        else:
            self.get_logger().error("Trailer parameter arrays must have equal length!")
        
        self.model = TractorTrailerModel(self.L0, self.trailers, dt=dt)
        
        self.state = np.zeros(3 + 2 * len(self.trailers))
        self.wheel_phi = np.zeros(2 + len(self.trailers))
        
        self.sub_odom = self.create_subscription(Odometry, '/localization/kinematic_state', self.odom_callback, 10)
        self.pub_joints = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_joints_veh = self.create_publisher(JointState, '/vehicle/joint_states', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/vehicle_markers', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.timer = self.create_timer(dt, self.update, clock=system_clock)
        self.v0 = 0.0
        self.delta = 0.0
        
        self.sub_initial = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)
        
        self.map_elevation = self.get_parameter('map_elevation').value
        self.get_logger().info(f'NMB Tractor Odometer Started with {len(self.trailers)} trailers at elevation {self.map_elevation}m')
        
        # Publish TFs once on startup
        self.update()

    def initial_pose_callback(self, msg):
        self.state[0] = msg.pose.pose.position.x
        self.state[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta0 = math.atan2(siny_cosp, cosy_cosp)
        
        # Reset tractor and all trailer headings to align perfectly straight
        self.state[2] = theta0
        for i in range(len(self.trailers)):
            self.state[3 + 2*i] = theta0     # drawbar absolute heading
            self.state[3 + 2*i + 1] = theta0 # trailer absolute heading
            
        self.update()

    def odom_callback(self, msg):
        self.v0 = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        
        if abs(self.v0) > 0.1:
            self.delta = math.atan(omega * self.L0 / self.v0)
        else:
            self.delta = 0.0
            
        new_x = msg.pose.pose.position.x
        new_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        new_theta = math.atan2(siny_cosp, cosy_cosp)
        
        # Detect teleport (distance > 1.0m or instant heading change > 0.5 rad)
        dist = math.hypot(new_x - self.state[0], new_y - self.state[1])
        angle_diff = abs(math.atan2(math.sin(new_theta - self.state[2]), math.cos(new_theta - self.state[2])))
        
        if dist > 1.0 or angle_diff > 0.5:
            # Teleport detected! Snap all trailers to the new heading
            for i in range(len(self.trailers)):
                self.state[3 + 2*i] = new_theta     # drawbar
                self.state[3 + 2*i + 1] = new_theta # trailer
                
        self.state[0] = new_x
        self.state[1] = new_y
        self.state[2] = new_theta

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def update(self):
        tractor_state_odom = self.state[0:3].copy()
        new_state = self.model.update(self.state, self.v0, self.delta)
        self.state = new_state
        self.state[0:3] = tractor_state_odom
        
        coords = self.model.get_coordinates(self.state)
        
        # Get system time for TF to prevent rejection if sim time is frozen
        sys_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        tf_timestamp = sys_clock.now().to_msg()
        
        timestamp = self.get_clock().now().to_msg()
        self.get_logger().info('Odometer timer triggered', throttle_duration_sec=1.0)
        
        def broadcast_tf(x, y, theta, child_frame, parent_frame='map'):
            t = TransformStamped()
            t.header.stamp = tf_timestamp
            t.header.frame_id = parent_frame
            t.child_frame_id = child_frame
            t.transform.translation.x = float(x)
            t.transform.translation.y = float(y)
            t.transform.translation.z = 0.0
            q = self.euler_to_quaternion(0, 0, theta)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

        for i in range(len(self.trailers)):
            idx_dolly = 3 + 3*i
            idx_axle = 4 + 3*i
            p_dolly = coords[idx_dolly]
            p_axle = coords[idx_axle]
            theta_drawbar = self.state[3 + 2*i]
            theta_trailer = self.state[3 + 2*i + 1]
            
            pass
        
        v_tr_rear = self.v0
        self.wheel_phi[0] += (v_tr_rear / (self.wheel_diam/2.0)) * self.dt
        v_tr_front = self.v0 / math.cos(self.delta) if math.cos(self.delta) != 0 else self.v0
        self.wheel_phi[1] += (v_tr_front / (self.wheel_diam/2.0)) * self.dt
        
        for i in range(len(self.trailers)):
            self.wheel_phi[2+i] += (self.v0 / (self.wheel_diam/2.0)) * self.dt

        marker_array = MarkerArray()
        marker_id = 0
        
        def create_mesh_marker(x, y, z, roll, pitch, yaw, mesh_path, frame_id='map'):
            nonlocal marker_id
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = timestamp
            m.ns = "meshes"
            m.id = marker_id
            marker_id += 1
            m.type = Marker.MESH_RESOURCE
            m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = float(z)
            q = self.euler_to_quaternion(roll, pitch, yaw)
            m.pose.orientation.x = q[0]
            m.pose.orientation.y = q[1]
            m.pose.orientation.z = q[2]
            m.pose.orientation.w = q[3]
            m.scale.x = 0.01
            m.scale.y = 0.01
            m.scale.z = 0.01
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 1.0
            m.color.a = 1.0
            m.mesh_resource = mesh_path
            m.mesh_use_embedded_materials = True
            return m

        # CAD Offsets for meshes
        tr_cad = (0.850064, -0.001778, 0.0)
        db_cad = (3.08664, -0.001778, 0.0)
        tl_cad = (5.08157, -0.001778, 0.0)
        mesh_rpy = (1.570796, 0.0, 0.0)
        
        map_z = getattr(self, 'map_elevation', 0.0)
        
        p0 = coords[0]
        p0_f = coords[1]
        theta0 = self.state[2]
        
        # Helper to transform local coordinates to map coordinates
        def to_map(cx, cy, px, py, theta):
            mx = px + cx * math.cos(theta) - cy * math.sin(theta)
            my = py + cx * math.sin(theta) + cy * math.cos(theta)
            return mx, my

        # Tractor Chassis (Map frame)
        tr_x, tr_y = to_map(tr_cad[0], tr_cad[1], p0[0], p0[1], theta0)
        marker_array.markers.append(create_mesh_marker(
            tr_x, tr_y, map_z, 
            mesh_rpy[0], mesh_rpy[1], theta0, 
            'package://tractor_description/mesh/TowTractor_Chassis.obj'
        ))
        
        # Tractor Wheels (Map frame)
        wb = self.L0
        tw = self.W
        wr = self.wheel_diam / 2.0
        
        # We also need to apply steering angle to front wheels
        steer = self.delta
        
        # Front Left
        fl_x, fl_y = to_map(wb, tw/2, p0[0], p0[1], theta0)
        fl_mx, fl_my = to_map(tr_cad[0] - wb, tr_cad[1] - tw/2, fl_x, fl_y, theta0 + steer)
        marker_array.markers.append(create_mesh_marker(
            fl_mx, fl_my, map_z, 
            mesh_rpy[0], mesh_rpy[1] + self.wheel_phi[1], theta0 + steer, 
            'package://tractor_description/mesh/TowTractor_FL_Wheel.obj'
        ))
        
        # Front Right
        fr_x, fr_y = to_map(wb, -tw/2, p0[0], p0[1], theta0)
        fr_mx, fr_my = to_map(tr_cad[0] - wb, tr_cad[1] + tw/2, fr_x, fr_y, theta0 + steer)
        marker_array.markers.append(create_mesh_marker(
            fr_mx, fr_my, map_z, 
            mesh_rpy[0], mesh_rpy[1] + self.wheel_phi[1], theta0 + steer, 
            'package://tractor_description/mesh/TowTractor_FR_Wheel.obj'
        ))
        
        # Rear Left
        rl_x, rl_y = to_map(0, tw/2, p0[0], p0[1], theta0)
        rl_mx, rl_my = to_map(tr_cad[0], tr_cad[1] - tw/2, rl_x, rl_y, theta0)
        marker_array.markers.append(create_mesh_marker(
            rl_mx, rl_my, map_z, 
            mesh_rpy[0], mesh_rpy[1] + self.wheel_phi[0], theta0, 
            'package://tractor_description/mesh/TowTractor_RL_Wheel.obj'
        ))
        
        # Rear Right
        rr_x, rr_y = to_map(0, -tw/2, p0[0], p0[1], theta0)
        rr_mx, rr_my = to_map(tr_cad[0], tr_cad[1] + tw/2, rr_x, rr_y, theta0)
        marker_array.markers.append(create_mesh_marker(
            rr_mx, rr_my, map_z, 
            mesh_rpy[0], mesh_rpy[1] + self.wheel_phi[0], theta0, 
            'package://tractor_description/mesh/TowTractor_RR_Wheel.obj'
        ))

        # Trailers and Drawbars
        db_tw = self.get_parameter('drawbar_track_width').value
        tl_tw = self.get_parameter('trailer_track_width').value
        tl_wr = self.trailer_wheel_radius
        
        for k in range(len(self.trailers)):
            p_dolly = coords[3 + 3*k]
            p_axle = coords[4 + 3*k]
            theta_drawbar = self.state[3 + 2*k]
            theta_trailer = self.state[3 + 2*k + 1]
            
            # Drawbar Chassis
            db_x, db_y = to_map(db_cad[0], db_cad[1], p_dolly[0], p_dolly[1], theta_drawbar)
            marker_array.markers.append(create_mesh_marker(
                db_x, db_y, map_z, 
                mesh_rpy[0], mesh_rpy[1], theta_drawbar, 
                'package://tractor_description/mesh/Drawbar.obj'
            ))
            
            # Drawbar Wheels
            dl_x, dl_y = to_map(0, db_tw/2, p_dolly[0], p_dolly[1], theta_drawbar)
            dl_mx, dl_my = to_map(db_cad[0], db_cad[1] - db_tw/2, dl_x, dl_y, theta_drawbar)
            marker_array.markers.append(create_mesh_marker(
                dl_mx, dl_my, map_z, 
                mesh_rpy[0], mesh_rpy[1] + self.wheel_phi[2+k], theta_drawbar, 
                'package://tractor_description/mesh/DL_Wheel.obj'
            ))
            
            dr_x, dr_y = to_map(0, -db_tw/2, p_dolly[0], p_dolly[1], theta_drawbar)
            dr_mx, dr_my = to_map(db_cad[0], db_cad[1] + db_tw/2, dr_x, dr_y, theta_drawbar)
            marker_array.markers.append(create_mesh_marker(
                dr_mx, dr_my, map_z, 
                mesh_rpy[0], mesh_rpy[1] + self.wheel_phi[2+k], theta_drawbar, 
                'package://tractor_description/mesh/DR_Wheel.obj'
            ))
            
            # Trailer Chassis
            tl_x, tl_y = to_map(tl_cad[0], tl_cad[1], p_axle[0], p_axle[1], theta_trailer)
            marker_array.markers.append(create_mesh_marker(
                tl_x, tl_y, map_z, 
                mesh_rpy[0], mesh_rpy[1], theta_trailer, 
                'package://tractor_description/mesh/Trailerl.obj'
            ))
            
            # Trailer Wheels
            tl_wl_x, tl_wl_y = to_map(0, tl_tw/2, p_axle[0], p_axle[1], theta_trailer)
            tl_mx, tl_my = to_map(tl_cad[0], tl_cad[1] - tl_tw/2, tl_wl_x, tl_wl_y, theta_trailer)
            marker_array.markers.append(create_mesh_marker(
                tl_mx, tl_my, map_z, 
                mesh_rpy[0], mesh_rpy[1] + self.wheel_phi[2+k], theta_trailer, 
                'package://tractor_description/mesh/TL_Wheel.obj'
            ))
            
            tr_wl_x, tr_wl_y = to_map(0, -tl_tw/2, p_axle[0], p_axle[1], theta_trailer)
            tr_mx, tr_my = to_map(tl_cad[0], tl_cad[1] + tl_tw/2, tr_wl_x, tr_wl_y, theta_trailer)
            marker_array.markers.append(create_mesh_marker(
                tr_mx, tr_my, map_z, 
                mesh_rpy[0], mesh_rpy[1] + self.wheel_phi[2+k], theta_trailer, 
                'package://tractor_description/mesh/TR_Wheel.obj'
            ))

        self.pub_markers.publish(marker_array)
        
        # Prepare empty JointState message (only needed if there are revolute joints in URDF in the future)
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = timestamp

        def broadcast_hitch_tf(roll, pitch, yaw, child_frame, parent_frame):
            t = TransformStamped()
            t.header.stamp = tf_timestamp
            t.header.frame_id = parent_frame
            t.child_frame_id = child_frame
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            q = self.euler_to_quaternion(roll, pitch, yaw)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

        # Add Trailer Joints if they exist
        for i in range(len(self.trailers)):
            idx_db = 3 + 2*i
            idx_prev = 2 + 2*i
            angle_db = self.state[idx_db] - self.state[idx_prev]
            
            idx_tr = 3 + 2*i + 1
            angle_tr = self.state[idx_tr] - self.state[idx_db]
            
            idx = i + 1
            parent_hitch = 'hitch_link' if i == 0 else f'trailer_rear_hitch_link_{idx-1}'
            
            # Tractor/Prev Trailer to drawbar (Explicit TF Broadcast)
            broadcast_hitch_tf(0.0, 0.0, float(angle_db), f'drawbar_hitch_link_{idx}', parent_hitch)
            
            # Drawbar to trailer (Explicit TF Broadcast)
            broadcast_hitch_tf(0.0, 0.0, float(angle_tr), f'trailer_front_hitch_link_{idx}', f'drawbar_top_hitch_link_{idx}')
                
        self.pub_joints.publish(msg)
        self.pub_joints_veh.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NMBOdometer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
