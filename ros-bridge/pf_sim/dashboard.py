#!/usr/bin/env python3
"""Web dashboard: mujoco overview, sensors, status and replay controls.

http://<host>:8090/
  * large third-person overview of the room (mujoco pf_overview camera)
  * side column: robot camera + lidar top-down
  * status: manager state, tracks, command, latest decision
  * sessions: coverage heatmap, summary text, replay button (needs the docker
    socket mounted; falls back to showing the command otherwise)
"""

import json
import math
import os
import shutil
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rospy
import zmq
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
from monocular_people_tracking.msg import TrackArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String

import common_pb2
import lidar_pb2

PAGE = b"""<!doctype html>
<html><head><meta charset="utf-8"><title>person following sim</title>
<style>
 body{margin:0;background:#14161a;color:#dfe3ea;font:13px/1.5 monospace}
 .wrap{display:flex;height:100vh}
 .main{flex:1;background:#000;display:flex;align-items:center;justify-content:center}
 .main img{width:100%;height:100%;object-fit:contain}
 .side{width:360px;padding:8px;box-sizing:border-box;overflow:auto}
 .card{background:#1d2026;border-radius:6px;margin-bottom:8px;padding:8px}
 .card h3{margin:0 0 6px;font-size:12px;color:#8fa3bf;font-weight:600}
 img.sensor{width:100%;border-radius:4px;background:#000}
 #status div{display:flex;justify-content:space-between}
 .bad{color:#ff6b6b}.good{color:#51cf66}
 button{background:#2b3a55;color:#dfe3ea;border:0;border-radius:4px;padding:4px 8px;cursor:pointer}
 button:hover{background:#3b4f73}
 select{background:#2b3a55;color:#dfe3ea;border:0;padding:4px;width:100%}
 #summary{white-space:pre-wrap;font-size:11px;color:#a9b7c9;max-height:180px;overflow:auto}
</style></head><body><div class="wrap">
<div class="main"><img id="overview" src="/stream/overview"></div>
<div class="side">
  <div class="card"><h3>ROBOT CAMERA</h3><img class="sensor" src="/stream/camera"></div>
  <div class="card"><h3>LIDAR 3D (drag to orbit, wheel to zoom)</h3><img id="lidar" class="sensor" src="/stream/lidar"></div>
  <div class="card" id="status"><h3>STATUS</h3><div>loading...</div></div>
  <div class="card"><h3>VALIDATION</h3>
    <select id="session"></select>
    <div style="margin-top:6px"><button onclick="loadSession()">load</button>
    <button onclick="replay()">replay</button></div>
    <div id="summary"></div>
    <img id="heat" class="sensor" style="margin-top:6px;display:none">
  </div>
</div></div>
<script>
async function poll(){
  try{
    const r = await fetch('/api/status'); const s = await r.json();
    document.getElementById('status').innerHTML =
      '<h3>STATUS</h3>' +
      row('state', s.state) + row('tracks', s.tracks) +
      row('cmd', s.cmd) + row('person', s.person) + row('reason', s.reason);
  }catch(e){}
}
function row(k,v){return '<div><span>'+k+'</span><span>'+v+'</span></div>'}
async function sessions(){
  const r = await fetch('/api/sessions'); const d = await r.json();
  const sel = document.getElementById('session');
  sel.innerHTML = d.sessions.map(s=>'<option>'+s+'</option>').join('');
  if(d.sessions.length) loadSession();
}
async function loadSession(){
  const name = document.getElementById('session').value;
  const r = await fetch('/api/summary?name='+encodeURIComponent(name));
  document.getElementById('summary').textContent = await r.text();
  const h = document.getElementById('heat');
  h.style.display='block'; h.src='/api/heatmap?name='+encodeURIComponent(name)+'&t='+Date.now();
}
async function replay(){
  const name = document.getElementById('session').value;
  if(!name) return;
  const r = await fetch('/api/replay?name='+encodeURIComponent(name), {method:'POST'});
  document.getElementById('summary').textContent = await r.text();
}
function orbit(id, url){
  const img = document.getElementById(id);
  let dragging=false, lx=0, ly=0;
  img.addEventListener('mousedown', e=>{dragging=true;lx=e.clientX;ly=e.clientY;e.preventDefault();});
  window.addEventListener('mouseup', ()=>dragging=false);
  window.addEventListener('mousemove', e=>{
    if(!dragging) return;
    const dx=e.clientX-lx, dy=e.clientY-ly; lx=e.clientX; ly=e.clientY;
    fetch(url+'?daz='+(-dx*0.5)+'&del='+(dy*0.4));
  });
  img.addEventListener('wheel', e=>{
    e.preventDefault();
    fetch(url+'?dist='+(e.deltaY>0?1.2:-1.2));
  }, {passive:false});
}
orbit('overview','/api/view');
orbit('lidar','/api/lidar_view');
poll(); setInterval(poll, 1000); sessions();
</script></body></html>"""


class Dashboard(object):
    def __init__(self):
        self.port = rospy.get_param('~port', 8090)
        self.sessions_dir = rospy.get_param('~sessions_dir', '/opt/pf_tmp/sim_sessions')
        self.scripts_dir = rospy.get_param('~scripts_dir', '/opt/pf_scripts')
        self.image_dir = rospy.get_param('~image_dir', '/opt/pf_tmp')
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frames = {'overview': None, 'camera': None, 'lidar': None}
        self.counters = {'overview': 0, 'camera': 0, 'lidar': 0}
        self.status = {'state': '-', 'tracks': 0, 'cmd': '-', 'person': '-',
                       'reason': '-'}
        self.robot = (0.0, 0.0, 0.0)
        self.view = {'azimuth': 90.0, 'elevation': -60.0, 'distance': 9.0}
        self.lidar_view = {'azimuth': 60.0, 'elevation': -25.0, 'distance': 14.0}
        endpoint = rospy.get_param('~zmq_endpoint', 'tcp://mujoco-wrapper:5555')
        self.cmd_endpoint = rospy.get_param('~zmq_command_endpoint',
                                            'tcp://mujoco-wrapper:5556')
        self.view_context = zmq.Context()
        self.view_push = self.view_context.socket(zmq.PUSH)
        self.view_push.setsockopt(zmq.LINGER, 0)
        self.view_push.connect(self.cmd_endpoint)
        self._send_view()
        self.points = np.zeros((0, 3))

        rospy.Subscriber('/pf_cam/overview', Image, self._overview_cb, queue_size=1)
        rospy.Subscriber('/pf_cam/image_raw', Image, self._camera_cb, queue_size=1)
        rospy.Subscriber('/observe_manager/state', String, self._state_cb,
                         queue_size=1)
        rospy.Subscriber('/monocular_people_tracking/tracks', TrackArray,
                         self._tracks_cb, queue_size=1)
        rospy.Subscriber('/cmd_vel', Twist, self._cmd_cb, queue_size=1)
        rospy.Subscriber('/person_pose_stamped', PoseStamped, self._person_cb,
                         queue_size=1)
        rospy.Subscriber('/odom', Odometry, self._odom_cb, queue_size=1)
        threading.Thread(target=self._lidar_loop, daemon=True).start()
        threading.Thread(target=self._lidar_render_loop, daemon=True).start()

        self.server = ThreadingHTTPServer(('0.0.0.0', self.port),
                                          self._make_handler())
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        rospy.loginfo('dashboard on http://0.0.0.0:%d/', self.port)

    def _send_view(self):
        import command_pb2
        command = command_pb2.Command()
        command.kind = 'view'
        command.x = self.robot[0]
        command.y = self.robot[1]
        command.z = 0.0
        command.vx = self.view['distance']
        command.vy = self.view['azimuth']
        command.omega = self.view['elevation']
        try:
            self.view_push.send(command.SerializeToString(), zmq.NOBLOCK)
        except zmq.ZMQError:
            pass

    def lookat_person(self):
        person = self.status.get('person') or '-'
        if person != '-':
            try:
                x, y = [float(part) for part in person.split(',')]
                self.view['lookat'] = (x, y)
            except ValueError:
                pass

    # ---------------- inputs ----------------
    def _store(self, key, message, draw=None):
        try:
            frame = self.bridge.imgmsg_to_cv2(message, 'bgr8')
        except Exception:
            return
        frame = np.ascontiguousarray(frame)
        if draw:
            draw(frame)
        with self.lock:
            self.frames[key] = frame
            self.counters[key] += 1

    def _overview_cb(self, message):
        self._store('overview', message, self._draw_overview)

    def _camera_cb(self, message):
        self._store('camera', message)

    def _state_cb(self, message):
        self.status['state'] = message.data

    def _tracks_cb(self, message):
        self.status['tracks'] = len(message.tracks)
        if message.tracks:
            track = message.tracks[0]
            self.status['reason'] = 'track {} ({:.1f}, {:.1f})'.format(
                track.id, track.pos.x, track.pos.y)

    def _cmd_cb(self, message):
        self.status['cmd'] = '{:.2f},{:.2f}'.format(message.linear.x,
                                                    message.angular.z)

    def _person_cb(self, message):
        self.status['person'] = '{:.2f},{:.2f}'.format(
            message.pose.position.x, message.pose.position.y)

    def _odom_cb(self, message):
        quaternion = message.pose.pose.orientation
        yaw = math.atan2(2 * quaternion.w * quaternion.z,
                         1 - 2 * quaternion.z * quaternion.z)
        self.robot = (message.pose.pose.position.x,
                      message.pose.pose.position.y, yaw)

    def _lidar_loop(self):
        endpoint = rospy.get_param('~zmq_endpoint', 'tcp://mujoco-wrapper:5555')
        context = zmq.Context()
        sub = context.socket(zmq.SUB)
        sub.connect(endpoint)
        sub.setsockopt(zmq.SUBSCRIBE, b'')
        sub.setsockopt(zmq.RCVTIMEO, 500)
        while not rospy.is_shutdown():
            try:
                raw = sub.recv()
            except zmq.Again:
                continue
            except zmq.ZMQError:
                break
            envelope = common_pb2.ZmqEnvelope()
            try:
                envelope.ParseFromString(raw)
            except Exception:
                continue
            if envelope.topic != 'lidar':
                continue
            cloud = lidar_pb2.PointCloud2()
            try:
                cloud.ParseFromString(envelope.payload)
            except Exception:
                continue
            points = np.frombuffer(cloud.data, dtype=np.float32)
            if points.size % 3:
                continue
            self.points = points.reshape(-1, 3)

    def _lidar_render_loop(self):
        """RViz-like 3D point cloud: orbit angles come from the page."""
        width, height = 560, 480
        focal = 420.0
        while not rospy.is_shutdown():
            canvas = np.zeros((height, width, 3), dtype=np.uint8)
            canvas[:] = (16, 18, 22)
            points = self.points
            if len(points):
                step = max(1, len(points) // 25000)
                sample = np.asarray(points[::step], dtype=np.float64)
                sample = sample - np.array([self.robot[0], self.robot[1], 0.0])
                azimuth = math.radians(self.lidar_view['azimuth'])
                elevation = math.radians(self.lidar_view['elevation'])
                ca, sa = math.cos(azimuth), math.sin(azimuth)
                ce, se = math.cos(elevation), math.sin(elevation)
                # world -> view: rotate around z (yaw), then around x (pitch)
                x = sample[:, 0] * ca + sample[:, 1] * sa
                y = -sample[:, 0] * sa + sample[:, 1] * ca
                z = sample[:, 2]
                y2 = y * ce - z * se
                z2 = y * se + z * ce
                depth = x + self.lidar_view['distance']
                mask = depth > 0.5
                u = (focal * y2[mask] / depth[mask] + width / 2)
                v = (height / 2 - focal * z2[mask] / depth[mask])
                height_norm = np.clip((sample[mask, 2] + 0.5) / 3.0, 0, 1)
                colours = np.stack([np.full_like(height_norm, 255),
                                    np.clip(255 * (1 - height_norm), 0, 255),
                                    np.clip(255 * height_norm, 0, 255)],
                                   axis=1).astype(np.uint8)
                ui = np.clip(u.astype(np.int32), 0, width - 1)
                vi = np.clip(v.astype(np.int32), 0, height - 1)
                inside = (u >= 0) & (u < width) & (v >= 0) & (v < height)
                for dy, dx in ((0, 0), (0, 1), (1, 0)):
                    yy = np.clip(vi + dy, 0, height - 1)
                    xx = np.clip(ui + dx, 0, width - 1)
                    canvas[yy[inside], xx[inside]] = colours[inside]
                # robot marker at the origin of the view
                origin = np.zeros((1, 3))
                ox = 0.0
                oy = 0.0
                oz = 1.0
                y2o = oy * ce - oz * se
                z2o = oy * se + oz * ce
                depth_o = ox + self.lidar_view['distance']
                cu = int(focal * y2o / depth_o + width / 2)
                cv2_ = int(height / 2 - focal * z2o / depth_o)
                cv2.drawMarker(canvas, (cu, cv2_), (80, 160, 255),
                               cv2.MARKER_TRIANGLE_UP, 14, 2)
            cv2.putText(canvas, 'az %.0f  el %.0f  d %.1f' % (
                self.lidar_view['azimuth'], self.lidar_view['elevation'],
                self.lidar_view['distance']), (8, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 170, 200), 1)
            with self.lock:
                self.frames['lidar'] = canvas
                self.counters['lidar'] += 1
            time.sleep(0.2)

    def _draw_overview(self, frame):
        for key, colour in (('person', (80, 220, 120)), ('robot', (80, 160, 255))):
            text = self.status.get(key)
            if text and text != '-':
                cv2.putText(frame, '{} {}'.format(key, text), (10, 26 + 22 * (
                    0 if key == 'person' else 1)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, colour, 2)

    # ---------------- outputs ----------------
    def jpeg(self, key, quality=None):
        if quality is None:
            quality = 88 if key == 'overview' else 78
        with self.lock:
            frame = self.frames.get(key)
            frame = None if frame is None else frame.copy()
        if frame is None:
            return None
        ok, buffer = cv2.imencode('.jpg', frame,
                                  [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        return buffer.tobytes() if ok else None

    def adjust_view(self):
        from urllib.parse import parse_qs, urlparse
        query = parse_qs(urlparse(self.path_query).query)
        for key, field in (('daz', 'azimuth'), ('del', 'elevation'),
                           ('dist', 'distance')):
            if key in query:
                self.view[field] += float(query[key][0])
        self.view['elevation'] = max(-89.0, min(-5.0, self.view['elevation']))
        self.view['distance'] = max(3.0, min(25.0, self.view['distance']))
        self.view['azimuth'] %= 360.0
        self._send_view()
        return self.view

    def adjust_lidar_view(self):
        from urllib.parse import parse_qs, urlparse
        query = parse_qs(urlparse(self.path_query).query)
        for key, field in (('daz', 'azimuth'), ('del', 'elevation'),
                           ('dist', 'distance')):
            if key in query:
                self.lidar_view[field] += float(query[key][0])
        self.lidar_view['elevation'] = max(-80.0, min(80.0,
                                                      self.lidar_view['elevation']))
        self.lidar_view['distance'] = max(4.0, min(40.0,
                                                   self.lidar_view['distance']))
        return self.lidar_view

    def sessions(self):
        try:
            names = [name for name in os.listdir(self.sessions_dir)
                     if os.path.isdir(os.path.join(self.sessions_dir, name))]
        except OSError:
            return []
        return sorted(names)

    def _make_handler(self):
        view = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def _send(self, code, body, content_type='application/json'):
                data = body.encode() if isinstance(body, str) else body
                self.send_response(code)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                path = self.path.split('?')[0]
                view.path_query = self.path
                if path in ('/', '/index.html'):
                    return self._send(200, PAGE, 'text/html; charset=utf-8')
                if path.startswith('/stream/'):
                    return self._stream(path.rsplit('/', 1)[-1])
                if path == '/api/status':
                    payload = dict(view.status)
                    payload['robot'] = '{:.2f},{:.2f}'.format(view.robot[0],
                                                              view.robot[1])
                    return self._send(200, json.dumps(payload))
                if path == '/api/view':
                    return self._send(200, json.dumps(view.adjust_view()))
                if path == '/api/lidar_view':
                    return self._send(200, json.dumps(view.adjust_lidar_view()))
                if path == '/api/sessions':
                    return self._send(200, json.dumps({'sessions': view.sessions()}))
                if path == '/api/summary':
                    name = self._query('name')
                    return self._send(200, view.summary(name), 'text/plain')
                if path == '/api/heatmap':
                    name = self._query('name')
                    return self._heatmap(name)
                return self._send(404, '{"error":"not found"}')

            def do_POST(self):
                path = self.path.split('?')[0]
                if path == '/api/replay':
                    name = self._query('name')
                    return self._send(200, view.replay(name), 'text/plain')
                return self._send(404, '{"error":"not found"}')

            def _query(self, key):
                from urllib.parse import parse_qs, urlparse
                values = parse_qs(urlparse(self.path).query).get(key)
                return values[0] if values else ''

            def _stream(self, key):
                if key not in ('overview', 'camera', 'lidar'):
                    return self._send(404, 'no stream')
                self.send_response(200)
                self.send_header('Content-Type',
                                 'multipart/x-mixed-replace; boundary=frame')
                self.end_headers()
                last = -1
                while not rospy.is_shutdown():
                    with view.lock:
                        counter = view.counters.get(key, 0)
                    if counter == last:
                        time.sleep(0.02)
                        continue
                    data = view.jpeg(key)
                    if data is None:
                        continue
                    last = counter
                    try:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(b'Content-Length: '
                                         + str(len(data)).encode() + b'\r\n\r\n')
                        self.wfile.write(data)
                        self.wfile.write(b'\r\n')
                    except (BrokenPipeError, ConnectionResetError):
                        return

            def _heatmap(self, name):
                png = os.path.join(view.sessions_dir, name, 'coverage.png')
                if not os.path.isfile(png):
                    return self._send(404, 'no heatmap')
                with open(png, 'rb') as handle:
                    return self._send(200, handle.read(), 'image/png')

        return Handler

    def summary(self, name):
        directory = os.path.join(self.sessions_dir, name)
        script = os.path.join(self.scripts_dir, 'summarize_pf_session.py')
        try:
            result = subprocess.run(['python3', script, directory],
                                    capture_output=True, text=True, timeout=30)
            return result.stdout or result.stderr
        except Exception as exc:  # noqa: BLE001
            return 'summary failed: {}'.format(exc)

    def replay(self, name):
        directory = os.path.join(self.sessions_dir, name)
        if not os.path.isdir(directory):
            return 'unknown session {}'.format(name)
        script = os.path.join(self.scripts_dir, 'replay_pf_session.sh')
        if not shutil.which('docker') or not os.path.exists('/var/run/docker.sock'):
            return ('replay needs the docker socket; run on the host:\n  '
                    'scripts/replay_pf_session.sh {}'.format(directory))
        try:
            result = subprocess.run(['bash', script, directory],
                                    capture_output=True, text=True, timeout=300)
            return (result.stdout or '') + (result.stderr or '')
        except Exception as exc:  # noqa: BLE001
            return 'replay failed: {}\n\nrun on the host:\n  ' \
                   'scripts/replay_pf_session.sh {}'.format(exc, directory)


def main():
    rospy.init_node('pf_sim_dashboard')
    Dashboard()
    rospy.spin()


if __name__ == '__main__':
    main()
