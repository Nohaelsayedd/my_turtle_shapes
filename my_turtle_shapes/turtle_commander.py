#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import SetPen
from std_srvs.srv import Empty  # Correct import for Clear service
import math
import threading
import time

def normalize_angle(a):
    while a > math.pi:
        a -= 2*math.pi
    while a <= -math.pi:
        a += 2*math.pi
    return a

class TurtleCommander(Node):
    def __init__(self):
        super().__init__('turtle_commander')
        self.shape_sub = self.create_subscription(String, 'shape_selection', self.shape_cb, 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.teleport_cli = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.set_pen_cli = self.create_client(SetPen, '/turtle1/set_pen')
        self.clear_cli = self.create_client(Empty, '/clear')  

        self.pose = None
        self.drawing = False
        self.stop_requested = False
        self.target_point = None  # (x, y) target for move_to
        self.current_shape = None  # Track current shape for speed
        self.timer = self.create_timer(0.02, self.timer_cb)  # 50Hz control loop

        self.get_logger().info('turtleCommander ready. Waiting for shape_selection topic messages.')

    def pose_cb(self, msg: Pose):
        self.pose = msg

    def shape_cb(self, msg: String):
        shape = msg.data.strip().lower()
        self.get_logger().info(f'Received shape: {shape}')
        if shape == 'stop':
            self.stop_requested = True
            self._stop_turtle()
            return
        if shape == 'clear':
            self._clear_screen()
            return
        if shape not in ('heart', 'star', 'flower'):
            self.get_logger().warn('Unknown shape. Use: heart, star, flower, or stop.')
            return
        if self.drawing:
            self.get_logger().info('Currently drawing — ignoring new request until finished.')
            return
        self.current_shape = shape  # Set current shape
        t = threading.Thread(target=self.draw_shape, args=(shape,), daemon=True)
        t.start()

    def _stop_turtle(self):
        self.drawing = False
        self.target_point = None
        self.current_shape = None
        twist = Twist()
        self.vel_pub.publish(twist)
        self.get_logger().info('Stop requested: turtle stopped.')

    def _set_pen(self, pen_on=True):
        if not self.set_pen_cli.wait_for_service(timeout_sec=5.0):
            return False
        req = SetPen.Request()
        req.r = 255
        req.g = 255
        req.b = 255
        req.width = 3
        req.off = not pen_on
        future = self.set_pen_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done() and future.result() is not None:
            return True
        return False

    def _teleport_turtle(self, x, y, theta=0.0):
        retries = 5
        for attempt in range(retries):
            if not self.teleport_cli.wait_for_service(timeout_sec=5.0):
                time.sleep(1.0)
                continue
            req = TeleportAbsolute.Request()
            req.x = float(x)
            req.y = float(y)
            req.theta = float(theta)
            future = self.teleport_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.done() and future.result() is not None:
                return True
            time.sleep(1.0)
        return False

    def _clear_screen(self):
        if not self.clear_cli.wait_for_service(timeout_sec=5.0):
            return
        future = self.clear_cli.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done() and future.result() is not None:
            self.get_logger().info('Screen cleared.')
        else:
            self.get_logger().error('Failed to clear screen.')

    # ---------- shape generators (centered & bigger) ----------
    def heart_points(self, n=300, scale=0.1, cx=5.5, cy=5.0):
        pts = []
        for i in range(n+1):
            t = 2*math.pi * i / n
            x = 16 * math.sin(t)**3
            y = 13*math.cos(t) - 5*math.cos(2*t) - 2*math.cos(3*t) - math.cos(4*t)
            pts.append((cx + scale*x, cy + scale*y))
        pts.append(pts[0])
        return pts

    def star_points(self, points=5, steps_per_edge=50, r_out=3.0, r_in=1.5, cx=5.5, cy=5.5):
        verts = []
        for i in range(points):
            angle_out = 2*math.pi * i / points
            verts.append((cx + r_out*math.cos(angle_out), cy + r_out*math.sin(angle_out)))
            angle_in = 2*math.pi * (i + 0.5) / points
            verts.append((cx + r_in*math.cos(angle_in), cy + r_in*math.sin(angle_in)))
        pts = []
        for i in range(len(verts)):
            x0,y0 = verts[i]
            x1,y1 = verts[(i+1) % len(verts)]
            for s in range(steps_per_edge + 1):  # Include endpoint
                alpha = s / float(steps_per_edge)
                pts.append((x0*(1-alpha) + x1*alpha, y0*(1-alpha) + y1*alpha))
        pts.append(verts[0])
        return pts

    def flower_paths(self, num_squares=5, steps_per_edge=100, r=3.0, cx=5.5, cy=5.5):
        paths = []
        for square_idx in range(num_squares):
            rotation_angle = square_idx * 2 * math.pi / (num_squares * 2)  # Keep original rotation
            verts = []
            for vertex_idx in range(4):
                angle = (vertex_idx * math.pi / 2) + rotation_angle
                x = cx + r * math.cos(angle)
                y = cy + r * math.sin(angle)
                verts.append((x, y))
            verts.append(verts[0])  # Close the square for interpolation
            # Interpolate edges
            pts = []
            for i in range(4):  # 4 edges
                x0, y0 = verts[i]
                x1, y1 = verts[i+1]
                for s in range(steps_per_edge + 1):
                    alpha = s / float(steps_per_edge)
                    pts.append((x0 * (1 - alpha) + x1 * alpha, y0 * (1 - alpha) + y1 * alpha))
            paths.append(pts)
        return paths

    # ---------- motion controller ----------
    def timer_cb(self):
        if not self.drawing or self.stop_requested or self.target_point is None or self.pose is None:
            self.vel_pub.publish(Twist())
            return

        tx, ty = self.target_point
        tol = 0.01
        K_lin = 22.5  
        K_ang = 6.0   
        vel_cap = 60.0  

        dx = tx - self.pose.x
        dy = ty - self.pose.y
        dist = math.hypot(dx, dy)
        if dist < tol:
            self.vel_pub.publish(Twist())
            self.target_point = None  # Signal that we reached the target
            return

        desired_yaw = math.atan2(dy, dx)
        yaw_err = normalize_angle(desired_yaw - self.pose.theta)

        twist = Twist()
        if abs(yaw_err) > 0.3:
            twist.linear.x = 0.0
            twist.angular.z = K_ang * yaw_err
        else:
            twist.angular.z = K_ang * yaw_err
            twist.linear.x = min(vel_cap, K_lin * dist)
        self.vel_pub.publish(twist)

    def move_to(self, tx, ty):
        if self.pose is None:
            start_time = time.time()
            while self.pose is None and rclpy.ok() and not self.stop_requested:
                time.sleep(0.02)  # Wait briefly instead of spinning
                if time.time() - start_time > 5.0:  # Timeout after 5s
                    return False
            if self.pose is None or self.stop_requested:
                return False

        self.target_point = (tx, ty)
        start_time = time.time()
        while self.target_point is not None and rclpy.ok() and not self.stop_requested:
            time.sleep(0.02)  # Let timer_cb handle movement
            if time.time() - start_time > 10.0:  # Timeout after 10s
                self.target_point = None
                return False
        return not self.stop_requested

    def draw_shape(self, shape_name):
        self.drawing = True
        self.stop_requested = False
        self.get_logger().info(f'Starting to draw: {shape_name}')

        if shape_name == 'heart':
            pts = self.heart_points()
            paths = [pts]  # Single path
        elif shape_name == 'star':
            pts = self.star_points()
            paths = [pts]  # Single path
        elif shape_name == 'flower':
            paths = self.flower_paths()
        else:
            paths = []

        if paths:
            for path_idx, path in enumerate(paths):
                if self.stop_requested:
                    break
                # Lift pen before teleporting
                if not self._set_pen(pen_on=False):
                    self.drawing = False
                    return
                # Teleport to first point
                if not self._teleport_turtle(path[0][0], path[0][1]):
                    self.drawing = False
                    return
                # Set pen down after teleporting
                if not self._set_pen(pen_on=True):
                    self.drawing = False
                    return
                # Draw the path starting from the second point (since already at first)
                for i in range(1, len(path)):
                    if self.stop_requested:
                        break
                    x, y = path[i]
                    if not self.move_to(x, y):
                        break

        self.vel_pub.publish(Twist())
        self.get_logger().info(f'Finished drawing: {shape_name}')
        self.drawing = False
        return

def main(args=None):
    rclpy.init(args=args)
    node = TurtleCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()