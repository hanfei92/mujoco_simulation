import numpy as np
from scipy.spatial.transform import Rotation as R
import trimesh

# ============ 配置 ============
OBJ_PATH = "/home/amigo/Documents/ament_ws/src/aidea_mujoco/mjcf/assets/full/room-732/Scaniverse_2026_11_02_094439.obj"

# 取最低的百分之几的点作为"地面候选"
GROUND_PERCENTILE = 2.0   # 先试 2%，如果地面点太少就调大

# ============ 1. 读 mesh ============
mesh = trimesh.load(OBJ_PATH, force='mesh')
V = mesh.vertices
print(f"顶点数: {len(V)}")
print(f"X 范围: [{V[:,0].min():.3f}, {V[:,0].max():.3f}]")
print(f"Y 范围: [{V[:,1].min():.3f}, {V[:,1].max():.3f}]")
print(f"Z 范围: [{V[:,2].min():.3f}, {V[:,2].max():.3f}]")

import numpy as np
from scipy.spatial.transform import Rotation as R
import trimesh

OBJ_PATH = "/home/amigo/Documents/ament_ws/src/aidea_mujoco/mjcf/assets/full/room-732/Scaniverse_2026_11_02_094439.obj"

mesh = trimesh.load(OBJ_PATH, force='mesh')
V = mesh.vertices

print(f"顶点数: {len(V)}")
for i, name in enumerate("XYZ"):
    print(f"{name}: [{V[:,i].min():.3f}, {V[:,i].max():.3f}], 跨度 {V[:,i].max()-V[:,i].min():.3f}")


def fit_plane_at_extreme(V, axis, high=True, percentile=2.0):
    """取 axis 方向最高（或最低）的 percentile% 点，PCA 拟合平面。
    返回 (normal, centroid, n_pts, flatness)
    """
    vals = V[:, axis]
    if high:
        thresh = np.percentile(vals, 100 - percentile)
        mask = vals >= thresh
    else:
        thresh = np.percentile(vals, percentile)
        mask = vals <= thresh

    pts = V[mask]
    if len(pts) < 10:
        return None, None, 0, None

    centroid = pts.mean(axis=0)
    centered = pts - centroid
    cov = centered.T @ centered
    eigvals, eigvecs = np.linalg.eigh(cov)

    normal = eigvecs[:, 0]
    normal = normal / np.linalg.norm(normal)

    # 让法向指向 high 方向
    if high and normal[axis] < 0:
        normal = -normal
    if not high and normal[axis] > 0:
        normal = -normal

    flatness = eigvals[0] / eigvals[-1] if eigvals[-1] > 1e-12 else np.inf
    return normal, centroid, len(pts), flatness


print("\n=== 分别试三个轴的最高点，找屋顶 ===")
roof_results = {}
for axis, name in [(0, 'X'), (1, 'Y'), (2, 'Z')]:
    normal, centroid, n_pts, flatness = fit_plane_at_extreme(V, axis, high=True, percentile=2.0)
    if normal is None:
        continue
    angle = np.degrees(np.arccos(np.clip(abs(normal[axis]), 0, 1)))
    roof_results[name] = (normal, centroid, n_pts, flatness, angle)
    print(f"up={name}: 点数={n_pts}, flatness={flatness:.6f}, "
          f"法向={normal.round(4)}, 与{name}夹角={angle:.3f}°, "
          f"质心={centroid.round(3)}")

best_name, (roof_normal, roof_centroid, n_pts, flatness, angle) = min(
    roof_results.items(), key=lambda kv: kv[1][3]
)
print(f"\n>>> 最可能的屋顶法向方向: {best_name}")
print(f">>> 屋顶法向: {roof_normal}")
print(f">>> 屋顶质心: {roof_centroid}")
print(f">>> 屋顶质心的 {best_name} 坐标: {roof_centroid[{'X':0,'Y':1,'Z':2}[best_name]]:.3f}")


# ============ 算旋转：让屋顶法向对齐世界 +Z ============
# 屋顶朝上，所以屋顶法向应该指向 +Z
world_z = np.array([0.0, 0.0, 1.0])
n = roof_normal

if np.allclose(n, world_z, atol=1e-6):
    rot = R.identity()
    axis = np.zeros(3)
    angle_rad = 0.0
else:
    axis = np.cross(n, world_z)
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-8:
        axis = np.array([1.0, 0.0, 0.0])
    else:
        axis = axis / axis_norm
    angle_rad = np.arccos(np.clip(np.dot(n, world_z), -1, 1))
    rot = R.from_rotvec(axis * angle_rad)

quat_xyzw = rot.as_quat()
quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])

print(f"\n旋转轴: {axis}")
print(f"旋转角: {np.degrees(angle_rad):.3f}°")
print(f'MuJoCo quat: "{quat_wxyz[0]:.6f} {quat_wxyz[1]:.6f} {quat_wxyz[2]:.6f} {quat_wxyz[3]:.6f}"')

# ============ 验证 + 旋转后包围盒 ============
n_rot = rot.apply(n)
print(f"\n验证：旋转后屋顶法向 = {n_rot}")

V_rot = rot.apply(V)
print(f"\n旋转后顶点范围:")
for i, name in enumerate("XYZ"):
    print(f"  {name}: [{V_rot[:,i].min():.3f}, {V_rot[:,i].max():.3f}], 跨度 {V_rot[:,i].max()-V_rot[:,i].min():.3f}")

roof_centroid_rot = rot.apply(roof_centroid)
print(f"\n旋转后屋顶质心: {roof_centroid_rot}")

# ============ 估算房间高度 ============
# 旋转后，竖直方向是 Z。屋顶在 Z 高处，地面（镂空）大概在 Z 最低点附近。
# 用旋转后 Z 的跨度作为房间高度的粗略估计
z_min_rot = V_rot[:,2].min()
z_max_rot = V_rot[:,2].max()
room_height_est = z_max_rot - z_min_rot
print(f"\n旋转后 Z 跨度（估计房间高度）: {room_height_est:.3f} m")
print(f"旋转后 Z 最低点: {z_min_rot:.3f}")
print(f"旋转后 Z 最高点（屋顶）: {z_max_rot:.3f}")
print(f"旋转后屋顶质心 Z: {roof_centroid_rot[2]:.3f}")

# ============ 选一个 pos Z ============
# 方案 A：让屋顶落在 Z = room_height_est，地面自然在 Z=0
# pos_z_A = room_height_est - roof_centroid_rot[2]

# 方案 B：让旋转后最低点落到 Z=0（墙根/家具腿落地）
pos_z_B = -z_min_rot

# 方案 C：让屋顶质心落到某个指定高度（比如 2.8 m）
target_roof_z = 2.8
pos_z_C = target_roof_z - roof_centroid_rot[2]

print(f"\n--- 三种 pos Z 方案 ---")
# print(f"A. 让屋顶在 Z={room_height_est:.3f}: pos Z = {pos_z_A:.6f}")
print(f"B. 让最低点落到 Z=0:        pos Z = {pos_z_B:.6f}")
print(f"C. 让屋顶质心到 Z={target_roof_z}: pos Z = {pos_z_C:.6f}")

print(f"\n推荐先用 B：")
print(f'<body name="Scaniverse_2026_11_02_094439" '
      f'pos="0 0 {pos_z_B:.6f}" '
      f'quat="{quat_wxyz[0]:.6f} {quat_wxyz[1]:.6f} {quat_wxyz[2]:.6f} {quat_wxyz[3]:.6f}">')

# ============ 额外：检查旋转后地面点的 XY 覆盖 ============
# 地面镂空，所以不找地面点，而是看最低 5cm 内的点分布在哪些 XY 位置
ground_mask = V_rot[:,2] <= z_min_rot + 0.05
ground_pts = V_rot[ground_mask]
print(f"\n旋转后最低 5cm 内的点: {len(ground_pts)} 个")
print(f"  X: [{ground_pts[:,0].min():.3f}, {ground_pts[:,0].max():.3f}]")
print(f"  Y: [{ground_pts[:,1].min():.3f}, {ground_pts[:,1].max():.3f}]")