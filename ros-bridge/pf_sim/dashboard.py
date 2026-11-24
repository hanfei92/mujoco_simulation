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
from nav_msgs.msg import Odometry, Path
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
  <div class="card"><h3>ROUTE EDITOR (click the map, then send)</h3>
    <div style="position:relative">
      <img id="mapimg" src="/api/map.png" style="width:100%;display:block;border-radius:4px">
      <canvas id="draw" style="position:absolute;left:0;top:0;width:100%;height:100%"></canvas>
    </div>
    <div style="margin-top:6px">
      <button onclick="addPoint()">add point</button>
      <button onclick="undoPoint()">undo</button>
      <button onclick="clearPoints()">clear</button>
      <button onclick="sendRoute()">send to G1</button>
      <span id="routemsg" style="color:#8fa3bf"></span>
    </div>
  </div>
  <div class="card"><h3>TIMELINE (candidates / FOV / decisions)</h3>
    <iframe id="timeline" style="width:100%;height:260px;border:0;border-radius:4px;background:#0d0f12"></iframe>
  </div>
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
    if (s.robot) { robotWorld = s.robot.split(',').map(Number); }
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
let mapMeta = null, routePts = [], hoverWorld = null, robotWorld = [0, 0];
async function loadMapMeta(){
  try { mapMeta = await (await fetch('/api/map_meta')).json(); } catch(e) {}
}
function syncCanvas(){
  const img = document.getElementById('mapimg');
  const canvas = document.getElementById('draw');
  const width = img.naturalWidth || 480;
  const height = img.naturalHeight || 480;
  canvas.width = width;
  canvas.height = height;
  canvas.style.width = img.clientWidth + 'px';
  canvas.style.height = img.clientHeight + 'px';
}
function pixelToWorld(ev){
  const img = document.getElementById('mapimg');
  const rect = img.getBoundingClientRect();
  const px = (ev.clientX - rect.left) / rect.width * img.naturalWidth;
  const py = (ev.clientY - rect.top) / rect.height * img.naturalHeight;
  return [mapMeta.origin_x + px / mapMeta.px_per_m,
          mapMeta.origin_y + (img.naturalHeight - py) / mapMeta.px_per_m];
}
function addPoint(){
  const box = document.getElementById('routemsg');
  if (!mapMeta) { box.textContent = 'map not loaded'; return; }
  const point = hoverWorld || robotWorld;
  routePts.push(point);
  redrawRoute();
  box.textContent = routePts.length + ' pts (' + point[0].toFixed(1) + ', ' +
                    point[1].toFixed(1) + ')';
}
function mapToPixel(x, y){
  const img = document.getElementById('mapimg');
  return [(x - mapMeta.origin_x) * mapMeta.px_per_m,
          img.naturalHeight - (y - mapMeta.origin_y) * mapMeta.px_per_m];
}
function redrawRoute(){
  const img = document.getElementById('mapimg');
  const canvas = document.getElementById('draw');
  if (!img.naturalWidth) return;
  syncCanvas();
  canvas.width = img.naturalWidth;
  canvas.height = img.naturalHeight;
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  if (!mapMeta) return;
  ctx.strokeStyle = '#5ac8ff'; ctx.lineWidth = 3;
  ctx.beginPath();
  routePts.forEach(function(pt, i){
    const p = mapToPixel(pt[0], pt[1]);
    if (i) ctx.lineTo(p[0], p[1]); else ctx.moveTo(p[0], p[1]);
  });
  ctx.stroke();
  ctx.fillStyle = '#5ac8ff';
  routePts.forEach(function(pt){
    const p = mapToPixel(pt[0], pt[1]);
    ctx.beginPath(); ctx.arc(p[0], p[1], 4, 0, 6.283); ctx.fill();
  });
}
const mapImg = document.getElementById('mapimg');
mapImg.addEventListener('load', syncCanvas);
window.addEventListener('resize', syncCanvas);
mapImg.addEventListener('mousemove', function(ev){
  if (!mapMeta) return;
  hoverWorld = pixelToWorld(ev);
});
mapImg.addEventListener('click', function(ev){
  if (!mapMeta) return;
  hoverWorld = pixelToWorld(ev);
  addPoint();
});
function undoPoint(){ routePts.pop(); redrawRoute();
  document.getElementById('routemsg').textContent = routePts.length + ' pts'; }
function clearPoints(){ routePts = []; redrawRoute();
  document.getElementById('routemsg').textContent = '0 pts'; }
async function sendRoute(){
  const box = document.getElementById('routemsg');
  if (routePts.length < 2) { box.textContent = 'need >= 2 points'; return; }
  const body = JSON.stringify({points: routePts});
  const response = await fetch('/api/route', {method: 'POST',
      headers: {'Content-Type': 'application/json'}, body: body});
  const result = await response.json();
  box.textContent = result.ok
      ? ('sent ' + result.points + ' pts / ' + result.length + ' m')
      : ('rejected: ' + result.message);
  document.getElementById('mapimg').src = '/api/map.png?t=' + Date.now();
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
document.getElementById('timeline').src =
  'http://' + location.hostname + ':8781/?file=observe-latest.jsonl';
loadMapMeta();
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
        self.route_pub = rospy.Publisher('/person_route/set', Path,
                                         queue_size=1, latch=True)
        self.map_data = self._load_map(
            rospy.get_param('~map_file', '/opt/pf_tmp/pf_map.json'))
        self.route_points = []
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

    def _load_map(self, path):
        try:
            with open(path) as handle:
                return json.load(handle)
        except OSError:
            rospy.logwarn('dashboard: map file %s not found', path)
            return None

    def map_meta(self):
        if not self.map_data:
            return {}
        resolution = float(self.map_data['resolution'])
        scale = 24.0  # pixels per metre
        return {
            'origin_x': float(self.map_data['origin_x']),
            'origin_y': float(self.map_data['origin_y']),
            'resolution': resolution,
            'width': int(self.map_data['width']),
            'height': int(self.map_data['height']),
            'px_per_m': scale,
        }

    def render_map(self):
        """Occupancy map + current route + robot/person, for the route editor."""
        meta = self.map_meta()
        if not meta:
            return None
        scale = meta['px_per_m']
        width = int(meta['width'] * meta['resolution'] * scale)
        height = int(meta['height'] * meta['resolution'] * scale)
        canvas = np.full((height, width, 3), 46, dtype=np.uint8)
        grid = np.array(self.map_data['data'], dtype=np.int8).reshape(
            meta['height'], meta['width'])
        cell = meta['resolution'] * scale
        for iy in range(meta['height']):
            for ix in range(meta['width']):
                if grid[iy, ix] < 50:
                    continue
                x0 = int(ix * cell)
                y0 = int(height - (iy + 1) * cell)
                canvas[y0:y0 + int(cell) + 1, x0:x0 + int(cell) + 1] = (12, 12, 14)

        def to_px_grid(metre):
            # metres -> pixels: the scale already is pixels per metre
            return int((metre - meta['origin_x']) * scale)

        # grid so the user can judge distances while drawing
        for metre in range(-10, 11, 2):
            offset = to_px_grid(metre)
            cv2.line(canvas, (offset, 0), (offset, height), (58, 62, 70), 1)
            cv2.line(canvas, (0, offset), (width, offset), (58, 62, 70), 1)

        def to_px(x, y):
            # clamp: a single bogus odometry sample must not kill the request
            if not (math.isfinite(x) and math.isfinite(y)):
                return (0, 0)
            px = int((x - meta['origin_x']) * scale)
            py = int(height - (y - meta['origin_y']) * scale)
            return (max(0, min(width - 1, px)), max(0, min(height - 1, py)))

        points = self.route_points or []
        for a, b in zip(points, points[1:]):
            cv2.line(canvas, to_px(*a), to_px(*b), (90, 200, 255), 2)
        for point in points:
            cv2.circle(canvas, to_px(*point), 3, (90, 200, 255), -1)

        person = self.status.get('person') or '-'
        if person != '-':
            try:
                x, y = [float(part) for part in person.split(',')]
                cv2.circle(canvas, to_px(x, y), 5, (80, 220, 120), -1)
            except ValueError:
                pass
        cv2.drawMarker(canvas, to_px(self.robot[0], self.robot[1]),
                       (80, 160, 255), cv2.MARKER_TRIANGLE_UP, 10, 2)
        return canvas

    def set_route(self, points):
        if len(points) < 2:
            return {'ok': False, 'message': 'need at least 2 points'}
        path = Path()
        path.header.frame_id = 'world'
        path.header.stamp = rospy.Time.now()
        total = 0.0
        previous = None
        for x, y in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
            if previous is not None:
                total += math.hypot(x - previous[0], y - previous[1])
            previous = (x, y)
        self.route_points = list(points)
        self.route_pub.publish(path)
        return {'ok': True, 'points': len(points), 'length': round(total, 1)}

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
                    payload['route_points'] = len(view.route_points)
                    return self._send(200, json.dumps(payload))
                if path == '/api/view':
                    return self._send(200, json.dumps(view.adjust_view()))
                if path == '/api/lidar_view':
                    return self._send(200, json.dumps(view.adjust_lidar_view()))
                if path == '/api/map_meta':
                    return self._send(200, json.dumps(view.map_meta()))
                if path == '/api/map.png':
                    canvas = view.render_map()
                    if canvas is None:
                        return self._send(404, 'no map')
                    ok, buffer = cv2.imencode('.png', canvas)
                    return self._send(200, buffer.tobytes(), 'image/png')
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
                if path == '/api/route':
                    length = int(self.headers.get('Content-Length') or 0)
                    try:
                        payload = json.loads(self.rfile.read(length) or b'{}')
                        result = view.set_route(payload.get('points') or [])
                        return self._send(200, json.dumps(result))
                    except Exception as exc:  # noqa: BLE001
                        return self._send(400, json.dumps({'ok': False,
                                                           'message': str(exc)}))
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
