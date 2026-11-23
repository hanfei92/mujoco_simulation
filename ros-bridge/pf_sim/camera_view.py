#!/usr/bin/env python3
"""Live view of the robot camera with the ground-truth keypoints overlaid.

Serves an MJPEG stream so the camera can be watched in a browser (no RViz, no
X11):   http://<host>:8090/

Overlays:
  * neck / ankle keypoints from /pose_estimator/pose (what the tracker sees)
  * the number of live tracks and the nearest track id
"""

import threading

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from monocular_people_tracking.msg import TrackArray
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tfpose_ros.msg import Persons
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

NECK, LANKLE, RANKLE = 1, 13, 10
COLORS = {NECK: (0, 255, 0), LANKLE: (0, 200, 255), RANKLE: (0, 200, 255)}


class CameraView(object):
    def __init__(self):
        self.port = rospy.get_param('~port', 8090)
        self.jpeg_quality = rospy.get_param('~jpeg_quality', 70)
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frame = None
        self.frame_id = 0
        self.persons = []
        self.tracks = 0
        self.state = ''
        self.command = (0.0, 0.0)

        rospy.Subscriber('/pf_cam/image_raw', Image, self.image_cb, queue_size=1)
        rospy.Subscriber('/pose_estimator/pose', Persons, self.pose_cb,
                         queue_size=1)
        rospy.Subscriber('/monocular_people_tracking/tracks', TrackArray,
                         self.tracks_cb, queue_size=1)
        rospy.Subscriber('/observe_manager/state', String, self.state_cb,
                         queue_size=1)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_cb, queue_size=1)

        self.server = ThreadingHTTPServer(('0.0.0.0', self.port),
                                          self._make_handler())
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        rospy.loginfo('camera view on http://0.0.0.0:%d/', self.port)

    def image_cb(self, message):
        try:
            frame = self.bridge.imgmsg_to_cv2(message, 'bgr8')
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn_throttle(5.0, 'camera_view: %s', exc)
            return
        frame = np.ascontiguousarray(frame)
        self._draw(frame)
        with self.lock:
            self.frame = frame
            self.frame_id += 1

    def pose_cb(self, message):
        self.persons = message

    def tracks_cb(self, message):
        self.tracks = len(message.tracks)

    def state_cb(self, message):
        self.state = message.data

    def cmd_cb(self, message):
        self.command = (message.linear.x, message.angular.z)

    def _draw(self, frame):
        height, width = frame.shape[:2]
        if self.persons is not None:
            for person in self.persons.persons:
                for part in person.body_part:
                    if part.part_id not in COLORS:
                        continue
                    u = int(round(part.x * width))
                    v = int(round(part.y * height))
                    cv2.circle(frame, (u, v), 4, COLORS[part.part_id], -1)
                    cv2.putText(frame, str(part.part_id), (u + 6, v - 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                                COLORS[part.part_id], 1)
        label = 'tracks {} | {} | cmd({:.2f},{:.2f})'.format(
            self.tracks, self.state or '-', self.command[0], self.command[1])
        cv2.putText(frame, label, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1, cv2.LINE_AA)

    def jpeg(self):
        with self.lock:
            frame = None if self.frame is None else self.frame.copy()
        if frame is None:
            return None
        ok, buffer = cv2.imencode('.jpg', frame,
                                  [int(cv2.IMWRITE_JPEG_QUALITY),
                                   self.jpeg_quality])
        return buffer.tobytes() if ok else None

    def _make_handler(self):
        view = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                if self.path in ('/', '/index.html'):
                    body = (b'<html><body style="margin:0;background:#111">'
                            b'<img src="/stream" style="width:100%">'
                            b'</body></html>')
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html')
                    self.send_header('Content-Length', str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if self.path == '/stream':
                    self.send_response(200)
                    self.send_header('Content-Type',
                                     'multipart/x-mixed-replace; boundary=frame')
                    self.end_headers()
                    last = -1
                    while not rospy.is_shutdown():
                        with view.lock:
                            frame_id = view.frame_id
                        if frame_id == last:
                            threading.Event().wait(0.02)
                            continue
                        data = view.jpeg()
                        if data is None:
                            continue
                        last = frame_id
                        try:
                            self.wfile.write(b'--frame\r\n')
                            self.wfile.write(b'Content-Type: image/jpeg\r\n')
                            self.wfile.write(
                                b'Content-Length: ' + str(len(data)).encode()
                                + b'\r\n\r\n')
                            self.wfile.write(data)
                            self.wfile.write(b'\r\n')
                        except (BrokenPipeError, ConnectionResetError):
                            return
                    return
                self.send_error(404)

        return Handler


def main():
    rospy.init_node('pf_sim_camera_view')
    CameraView()
    rospy.spin()


if __name__ == '__main__':
    main()
