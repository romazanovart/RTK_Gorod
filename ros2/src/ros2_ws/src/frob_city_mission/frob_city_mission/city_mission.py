import json
import math
import threading
import time
from collections import Counter, deque
from http import server

import cv2
import numpy as np
import rclpy
from frob_interfaces.srv import Forward, Turn
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger


class DebugStreamState:
    def __init__(self):
        self.condition = threading.Condition()
        self.jpeg = None

    def update(self, jpeg):
        with self.condition:
            self.jpeg = jpeg
            self.condition.notify_all()


class DebugStreamHandler(server.BaseHTTPRequestHandler):
    state = None

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            page = '''<!doctype html><html lang="ru"><head><meta charset="utf-8">
<title>Frob — автономная миссия</title>
<style>body{margin:0;background:#111;color:#eee;font-family:sans-serif;text-align:center}
img{max-width:96vw;max-height:86vh;margin-top:10px}.hint{padding:8px;color:#bbb}</style></head>
<body><h2>Frob: линия, знаки и состояние миссии</h2><img src="/stream.mjpg">
<div class="hint">зелёный — выбранная линия · жёлтый — зона поиска перекрёстка</div>
</body></html>'''.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(page)))
            self.end_headers()
            self.wfile.write(page)
            return
        if self.path == '/snapshot.jpg':
            with self.state.condition:
                jpeg = self.state.jpeg
            if jpeg is None:
                self.send_error(503, 'Camera frame is not ready')
                return
            self.send_response(200)
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(jpeg)))
            self.end_headers()
            self.wfile.write(jpeg)
            return
        if self.path != '/stream.mjpg':
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Cache-Control', 'no-cache, private')
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        try:
            while True:
                with self.state.condition:
                    self.state.condition.wait(timeout=2.0)
                    jpeg = self.state.jpeg
                if jpeg is None:
                    continue
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n')
                self.wfile.write(f'Content-Length: {len(jpeg)}\r\n\r\n'.encode())
                self.wfile.write(jpeg)
                self.wfile.write(b'\r\n')
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, _format, *_args):
        return


class CityMission(Node):
    """Follow the white road marking and apply signs at cell centres."""

    def __init__(self):
        super().__init__('city_mission')
        defaults = {
            'linear_speed': 0.07, 'steering_gain': 0.65,
            'max_angular_speed': 0.40, 'white_value_min': 165,
            'white_saturation_max': 85, 'max_white_fraction': 0.55,
            'line_lost_timeout': 1.0,
            'intersection_confirm_frames': 3, 'intersection_cooldown': 1.5,
            'minimum_follow_time': 1.0, 'maximum_follow_time': 15.0,
            'center_advance_mm': 170, 'center_advance_speed': 70,
            'turn_speed': 100, 'observe_seconds': 1.2,
            'sign_window_seconds': 1.5, 'sign_min_confidence': 0.55,
            'sign_min_confirmations': 2, 'bus_stop_seconds': 2.2,
            'expected_bus_stops': 1, 'obstacle_stop_distance': 0.25,
            'http_port': 8081,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/city_mission/state', 10)
        self.debug_pub = self.create_publisher(
            CompressedImage, '/city_mission/line_debug/compressed', 1)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, 1)
        self.create_subscription(String, '/sign_detector/detections', self.on_signs, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        self.forward = self.create_client(Forward, '/forward_robot')
        self.rotate = self.create_client(Turn, '/rotate_robot')
        self.create_service(Trigger, '/city_mission/test_cell', self.test_cell)
        self.create_service(Trigger, '/city_mission/start', self.start)
        self.create_service(Trigger, '/city_mission/stop', self.stop)

        self.state = 'IDLE'
        self.full_mission = False
        self.line_error = None
        self.last_line = 0.0
        self.follow_started = 0.0
        self.ignore_cross_until = 0.0
        self.cross_frames = 0
        self.observe_until = 0.0
        self.wait_until = 0.0
        self.signs = deque()
        self.bus_stops = 0
        self.last_sign = None
        self.obstacle = False
        self.latest_detections = []

        self.stream_state = DebugStreamState()
        DebugStreamHandler.state = self.stream_state
        port = int(self.p['http_port'])
        self.http_server = server.ThreadingHTTPServer(('0.0.0.0', port), DebugStreamHandler)
        self.http_thread = threading.Thread(target=self.http_server.serve_forever, daemon=True)
        self.http_thread.start()
        self.create_timer(0.1, self.control)
        self.get_logger().info(
            f'Ready: call /city_mission/test_cell first; debug stream http://0.0.0.0:{port}/')

    def test_cell(self, _request, response):
        return self._start_request(response, full=False)

    def start(self, _request, response):
        self.bus_stops = 0
        return self._start_request(response, full=True)

    def _start_request(self, response, full):
        if self.state not in ('IDLE', 'PARKED', 'ERROR'):
            response.success = False
            response.message = f'Busy: {self.state}'
            return response
        self.full_mission = full
        self.signs.clear()
        self._follow_line()
        response.success = True
        response.message = 'Full mission started' if full else 'One-cell test started'
        return response

    def stop(self, _request, response):
        self.full_mission = False
        self.state = 'IDLE'
        self._stop_robot()
        response.success = True
        response.message = 'Stopped'
        return response

    def on_image(self, msg):
        if msg.encoding not in ('bgr8', 'rgb8') or msg.step < msg.width * 3:
            return
        raw = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        frame = raw[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv, (0, 0, int(self.p['white_value_min'])),
            (180, int(self.p['white_saturation_max']), 255))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        height, width = mask.shape
        white_fraction = cv2.countNonZero(mask) / float(mask.size)
        valid_white_scene = white_fraction <= float(self.p['max_white_fraction'])

        # Bottom ROI: choose the white contour closest to the camera centre.
        lane_y = int(height * 0.62)
        lane = mask[lane_y:int(height * 0.96), :]
        contours, _ = cv2.findContours(lane, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if cv2.contourArea(c) > 35]
        selected_cx = None
        if valid_white_scene and contours:
            middle = width / 2.0
            contour = min(
                contours,
                key=lambda c: abs(cv2.boundingRect(c)[0] + cv2.boundingRect(c)[2] / 2 - middle))
            m = cv2.moments(contour)
            if m['m00']:
                cx = m['m10'] / m['m00']
                selected_cx = int(cx)
                self.line_error = (cx - middle) / middle
                self.last_line = time.monotonic()

        # A long horizontal white segment marks a crossing/intersection.
        cross_y = int(height * 0.42)
        cross = mask[cross_y:int(height * 0.78), :]
        lines = cv2.HoughLinesP(
            cross, 1, np.pi / 180, 45,
            minLineLength=int(width * 0.38), maxLineGap=35)
        horizontal = False
        if lines is not None:
            horizontal = any(
                abs(y2 - y1) < 0.12 * max(1, abs(x2 - x1))
                for x1, y1, x2, y2 in lines[:, 0])
        horizontal = horizontal and valid_white_scene
        if (self.state == 'FOLLOW_LINE' and horizontal
                and time.monotonic() >= self.ignore_cross_until):
            self.cross_frames += 1
        else:
            self.cross_frames = 0

        debug = frame.copy()
        overlay = np.zeros_like(debug)
        overlay[mask > 0] = (90, 90, 90)
        debug = cv2.addWeighted(debug, 0.78, overlay, 0.35, 0)
        cv2.line(debug, (0, lane_y), (width, lane_y), (255, 0, 0), 2)
        cv2.line(debug, (0, cross_y), (width, cross_y), (0, 255, 255), 2)
        if selected_cx is not None:
            cv2.line(debug, (width // 2, lane_y), (width // 2, height - 1), (0, 255, 0), 2)
            cv2.circle(debug, (selected_cx, int(height * 0.79)), 8, (0, 255, 0), -1)
        for item in self.latest_detections:
            box = item.get('box_xyxy', [])
            if len(box) != 4:
                continue
            x1, y1, x2, y2 = (int(value) for value in box)
            label = f"{item.get('class', '?')} {float(item.get('confidence', 0.0)):.2f}"
            cv2.rectangle(debug, (x1, y1), (x2, y2), (255, 120, 0), 2)
            cv2.putText(debug, label, (x1, max(18, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 120, 0), 2)
        error_text = 'none' if self.line_error is None else f'{self.line_error:+.3f}'
        cv2.putText(debug, f'{self.state} line={error_text} white={white_fraction:.2f}',
                    (8, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 0) if valid_white_scene else (0, 0, 255), 2)
        cv2.putText(debug, f'sign={self._stable_sign(time.monotonic()) or "none"}', (8, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 120, 0), 2)
        ok, encoded = cv2.imencode('.jpg', debug, [cv2.IMWRITE_JPEG_QUALITY, 65])
        if ok:
            jpeg = encoded.tobytes()
            out = CompressedImage(header=msg.header, format='jpeg', data=jpeg)
            self.debug_pub.publish(out)
            self.stream_state.update(jpeg)

    def on_signs(self, msg):
        try:
            items = json.loads(msg.data).get('detections', [])
        except (ValueError, TypeError, AttributeError):
            return
        self.latest_detections = items
        now = time.monotonic()
        for item in items:
            if float(item.get('confidence', 0.0)) >= float(self.p['sign_min_confidence']):
                self.signs.append((now, str(item.get('class', ''))))
        self._trim_signs(now)

    def on_scan(self, msg):
        distances = []
        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) <= math.radians(18) and math.isfinite(distance):
                distances.append(distance)
        self.obstacle = bool(distances) and min(distances) < float(
            self.p['obstacle_stop_distance'])

    def control(self):
        now = time.monotonic()
        self._publish_state(now)
        if self.state == 'FOLLOW_LINE':
            if self.obstacle:
                return self._fail('Obstacle inside safety distance')
            if now - self.follow_started > float(self.p['maximum_follow_time']):
                return self._fail('Cell-centre detection timeout')
            if now - self.last_line > float(self.p['line_lost_timeout']):
                self._stop_robot()
                return
            if (self.cross_frames >= int(self.p['intersection_confirm_frames'])
                    and now - self.follow_started >= float(self.p['minimum_follow_time'])):
                self.cross_frames = 0
                self._stop_robot()
                self._advance_to_centre()
                return
            command = Twist()
            command.linear.x = float(self.p['linear_speed'])
            angular = -float(self.p['steering_gain']) * float(self.line_error or 0.0)
            limit = float(self.p['max_angular_speed'])
            command.angular.z = max(-limit, min(limit, angular))
            self.cmd_pub.publish(command)
        elif self.state == 'OBSERVE_SIGN' and now >= self.observe_until:
            self._apply_sign(self._stable_sign(now))
        elif self.state == 'BUS_STOP_WAIT' and now >= self.wait_until:
            self._follow_line()

    def _advance_to_centre(self):
        if not self.forward.wait_for_service(timeout_sec=0.3):
            return self._fail('/forward_robot unavailable')
        self.state = 'ADVANCE_TO_CENTER'
        request = Forward.Request()
        request.dist = int(self.p['center_advance_mm'])
        request.speed = int(self.p['center_advance_speed'])
        self.forward.call_async(request).add_done_callback(self._centre_done)

    def _centre_done(self, future):
        try:
            success = future.result().success
        except Exception as exc:
            return self._fail(f'Forward service exception: {exc}')
        if not success:
            return self._fail('Forward service failed')
        if not self.full_mission:
            self.state = 'IDLE'
            self._stop_robot()
            self.get_logger().info('One-cell test complete')
        else:
            self.state = 'OBSERVE_SIGN'
            self.observe_until = time.monotonic() + float(self.p['observe_seconds'])

    def _apply_sign(self, sign):
        self.last_sign = sign or 'none'
        self.signs.clear()
        self.get_logger().info(f'Cell decision: {self.last_sign}')
        if sign == 'left':
            self._turn(90)
        elif sign == 'right':
            self._turn(-90)
        elif sign == 'no_left':
            self._turn(-90)  # First-version rule for a T-intersection.
        elif sign == 'no_right':
            self._turn(90)   # First-version rule for a T-intersection.
        elif sign == 'bus_stop':
            self.bus_stops += 1
            self.state = 'BUS_STOP_WAIT'
            self.wait_until = time.monotonic() + float(self.p['bus_stop_seconds'])
        elif sign == 'parking' and self.bus_stops >= int(self.p['expected_bus_stops']):
            self.state = 'PARKED'
            self._stop_robot()
            self.get_logger().info('Mission complete')
        elif sign == 'obstacle':
            self._fail('Danger sign: route planner not implemented in version 1')
        else:
            self._follow_line()  # straight, early parking, or no stable sign

    def _turn(self, angle):
        if not self.rotate.wait_for_service(timeout_sec=0.3):
            return self._fail('/rotate_robot unavailable')
        self.state = 'TURNING'
        request = Turn.Request(angle=int(angle), speed=int(self.p['turn_speed']))
        self.rotate.call_async(request).add_done_callback(self._turn_done)

    def _turn_done(self, future):
        try:
            success = future.result().success
        except Exception as exc:
            return self._fail(f'Turn service exception: {exc}')
        self._follow_line() if success else self._fail('Turn service failed')

    def _follow_line(self):
        now = time.monotonic()
        self.state = 'FOLLOW_LINE'
        self.follow_started = now
        self.ignore_cross_until = now + float(self.p['intersection_cooldown'])
        self.cross_frames = 0
        self.line_error = None
        self.last_line = 0.0

    def _trim_signs(self, now):
        cutoff = now - float(self.p['sign_window_seconds'])
        while self.signs and self.signs[0][0] < cutoff:
            self.signs.popleft()

    def _stable_sign(self, now):
        self._trim_signs(now)
        counts = Counter(label for _, label in self.signs if label)
        if not counts:
            return None
        label, count = counts.most_common(1)[0]
        return label if count >= int(self.p['sign_min_confirmations']) else None

    def _publish_state(self, now):
        state = {
            'state': self.state, 'full_mission': self.full_mission,
            'line_error': self.line_error, 'cross_frames': self.cross_frames,
            'stable_sign': self._stable_sign(now), 'last_sign': self.last_sign,
            'bus_stops': self.bus_stops, 'obstacle': self.obstacle,
        }
        self.state_pub.publish(String(data=json.dumps(state, ensure_ascii=False)))

    def _stop_robot(self):
        self.cmd_pub.publish(Twist())

    def _fail(self, reason):
        self.state = 'ERROR'
        self._stop_robot()
        self.get_logger().error(reason)

    def destroy_node(self):
        self._stop_robot()
        self.http_server.shutdown()
        self.http_server.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CityMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
