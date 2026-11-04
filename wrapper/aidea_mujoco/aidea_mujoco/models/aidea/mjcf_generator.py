import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import yourdfpy

from urdf2mjcf.convert import convert_urdf_to_mjcf
from urdf2mjcf.model import ConversionMetadata


def generate_mjcf(urdf_path: str, out_mjcf_path: str=None):
    """
    Generates a Stretch 4 MJCF from the provided URDF using urdf2mjcf and programmatic DOM manipulations.
    Note, this function only works for stretch 4 urdf -> mjcf, and is based on a manual process that used to happen to convert a URDF to an mjcf.
    """
    
    print(f"Generating mjcf from {urdf_path}")
    urdf_path = Path(urdf_path)
    if out_mjcf_path is None: 
        out_mjcf_path = urdf_path.with_suffix(".mjcf")
    else: 
        out_mjcf_path = Path(out_mjcf_path)

    # 1. Update default settings

    metadata = ConversionMetadata(
        # Adjust the height so the wheels do not start below the floor (measure from the base_link)
        height_offset=0.02734,
        # Allow the root link to be "unwelded"
        freejoint=True,
        # Do not anchor the root link to a surface
        floating_base=True,
        # Avoid the default "front_camera" and "side_camera" artifacts
        cameras=[],
    )

    convert_urdf_to_mjcf(urdf_path, out_mjcf_path, metadata=metadata)

    # 2. Parse the raw MJCF
    tree = ET.parse(out_mjcf_path)
    root = tree.getroot()

    # Create a new root holding only asset and worldbody
    new_root = ET.Element("mujoco", model="aidea")
    asset = root.find("asset")
    worldbody = root.find("worldbody")
    if asset is None or worldbody is None:
        raise ValueError("urdf2mjcf output is missing 'asset' or 'worldbody' tags.")

    new_root.append(asset)
    new_root.append(worldbody)

    # Helper function to find a body by name
    def find_body(name):
        return worldbody.find(f".//body[@name='{name}']")



    # 4. Rename root body to stretch4 and update its geoms
    base_body = worldbody.find("body")
    if base_body is not None:
        if base_body.get("name") != "aidea":
            base_body.set("name", "aidea")
        if "childclass" in base_body.attrib:
            del base_body.attrib["childclass"]
            
        ET.SubElement(base_body, "site", name="imu", size="0.01", pos="0 0 0")


    # 6. Add lidars, cameras, and laser dynamically from URDF
    urdf = yourdfpy.URDF.load(urdf_path)

    def get_cam_pos_quat(link_name, parent_name, post_rotation:np.ndarray|None = None):
        T_parent_link = urdf.get_transform(link_name, parent_name)
        T_parent_cam = T_parent_link
        if post_rotation is not None:
            T_parent_cam = T_parent_link @ post_rotation

        pos = T_parent_cam[:3, 3]
        quat = Rotation.from_matrix(T_parent_cam[:3, :3]).as_quat() # xyzw
        # MuJoCo quat format: w x y z
        quat_mj = f"{quat[3]} {quat[0]} {quat[1]} {quat[2]}"
        pos_mj = f"{pos[0]} {pos[1]} {pos[2]}"
        return pos_mj, quat_mj

    T_opt_cam = np.eye(4)
    # Note that Mujoco's camera frame coordinate system is different from ROS's: " Forward corresponds to the negative Z axis of the camera frame, while up corresponds to the positive Y axis. " , so in creating the Camera lement in the mjcf, we rotate by 180 degrees. https://mujoco.readthedocs.io/en/stable/programming/visualization.html
    T_opt_cam[:3, :3] = Rotation.from_euler('xyz', [180, 0, 0], degrees=True).as_matrix()

    # 鱼眼相机
    fisheye_body = find_body("fisheye_link")
    if fisheye_body is not None:
        pos, quat = get_cam_pos_quat("fisheye_link", "base_link", post_rotation=T_opt_cam)
        ET.SubElement(fisheye_body, "camera", name="fisheye_camera", pos=pos, quat=quat)

    # 夹爪相机
    gripper_cam_body = find_body("camera_link")
    if gripper_cam_body is not None:
        pos, quat = get_cam_pos_quat("camera_link", "gripper_camera_mount_link", post_rotation=T_opt_cam)
        ET.SubElement(gripper_cam_body, "camera", name="gripper_camera", pos=pos, quat=quat)    

    # 激光雷达
    # 在 MJCF 里加一个 <camera>，不是为了模拟 lidar 扫描，而是为了：
    # 可视化：在 MuJoCo viewer 里能看到"lidar 视角"的画面；
    # 调试：检查 lidar 安装位置和朝向。
    # 真正的 lidar 数据不是从 <camera> 来的，而是从 <sensor> 里的 rangefinder 或外部插件。
    airy_body = find_body("airy_link")
    if airy_body is not None:
        pos, quat = get_cam_pos_quat("airy_link", "base_link", post_rotation=T_opt_cam)
        ET.SubElement(airy_body, "camera", name="airy_camera", pos=pos, quat=quat)

    # if base_body is not None:
    #     old_laser = worldbody.find(".//body[@name='laser']")
    #     if old_laser is not None:
    #         parent_map = {c: p for p in worldbody.iter() for c in p}
    #         p = parent_map.get(old_laser)
    #         if p is not None:
    #             p.remove(old_laser)

    #     laser = ET.SubElement(base_body, "body", name="laser", pos="0.0 0.0 0.2", quat="0 0 0 1")
    #     rep = ET.SubElement(laser, "replicate", count="360", euler="0 0 0.0174533")
    #     ET.SubElement(rep, "site", name="lidar", zaxis="1 0 0")

    # 8. Modify Mast to use appropriate collision mass
    # mast = find_body("mast_link")
    # if mast is not None:
    #     for geom in mast.findall("geom"):
    #         mast.remove(geom)
    #     ET.SubElement(mast, "geom", type="mesh", mesh="mast_link", **{"class": "visualgeom"})
    #     ET.SubElement(
    #         mast, "geom", mesh="mast_collision_link", **{"class": "collision"}, mass="4.25"
    #     )

    # 9. Update arm_l0_link if needed and add gravcomp to lift, arm, wrist, gripper bodies
    # for b in worldbody.findall(".//body"):
    #     name = b.get("name", "")
    #     if "lift_link" in name or "arm_link" in name or "wrist_link" in name or "gripper" in name:
    #         b.set("gravcomp", "1")
    # for b in worldbody.findall(".//body"):
    #     name = b.get("name", "")
    #     # 6 轴臂
    #     if name.startswith("joint") and name.endswith("_link"):
    #         b.set("gravcomp", "1")
    #     # 夹爪手指
    #     elif name.startswith("gripper_") and ("l1" in name or "l2" in name or "l3" in name or "r1" in name or "r2" in name or "r3" in name):
    #         b.set("gravcomp", "1")
    for b in worldbody.findall(".//body"):
        name = b.get("name", "")
        # 只给 6 轴臂的前 6 节加
        if name in ["joint1_link", "joint2_link", "joint3_link", "joint4_link", "joint5_link", "joint6_link"]:
            b.set("gravcomp", "1")

    # 10. Encapsulate gripper fingers into gripper_slider_link
    # f_right = find_body("gripper_finger_right_link")
    # f_left = find_body("gripper_finger_left_link")
    # if f_right is not None and f_left is not None:
    #     parent_map = {c: p for p in worldbody.iter() for c in p}
    #     p = parent_map.get(f_right)
    #     if p is not None:
    #         slider = ET.Element("body", name="gripper_slider_link", pos="0 0 0", gravcomp="1")
    #         ET.SubElement(
    #             slider,
    #             "geom",
    #             type="box",
    #             size=".03 .005 .005",
    #             mass=".05",
    #             **{"class": "visualgeom"},
    #             rgba="0 0 0 0",
    #         )
    #         p.remove(f_right)
    #         p.remove(f_left)
    #         slider.append(f_right)
    #         slider.append(f_left)
    #         p.append(slider)

    # 11. Add "rubber" class to gripper fingers and fix thin-shell mesh issues
    for geom in worldbody.findall(".//geom"):
        mesh = geom.get("mesh", "")
        if "gripper_l1" in mesh or "gripper_r1" in mesh:
            if geom.get("class") != "visualgeom":
                geom.set("class", "rubber")

    # Add compliant passive joints to fingertips to allow surface alignment
    # for body in worldbody.findall(".//body"):
    #     name = body.get("name", "")
    #     if name in ["gripper_fingertip_right_link", "gripper_fingertip_left_link"]:
    #         ET.SubElement(
    #             body,
    #             "joint",
    #             name=f"{name}_compliant_x",
    #             type="hinge",
    #             axis="1 0 0",
    #             stiffness="0.1",
    #             damping="0.002",
    #             springref="0",
    #             limited="true",
    #             range="-0.15 0.15",
    #         )
    #         ET.SubElement(
    #             body,
    #             "joint",
    #             name=f"{name}_compliant_y",
    #             type="hinge",
    #             axis="0 1 0",
    #             stiffness="0.1",
    #             damping="0.002",
    #             springref="0",
    #             limited="true",
    #             range="-0.15 0.15",
    #         )

    # 12. Update Joint Classes
    # for j in worldbody.findall(".//joint"):
    #     name = j.get("name", "")
    #     if "lift_joint" in name:
    #         j.set("class", "lift_stretch4")
    #     elif "arm_l" in name and "_joint" in name:
    #         j.set("class", "telescope")
    #     elif "wrist_yaw_joint" in name:
    #         j.set("class", "wrist_yaw_stretch4")
    #     elif "wrist_pitch_joint" in name:
    #         j.set("class", "wrist_pitch_stretch4")
    #     elif "wrist_roll_joint" in name:
    #         j.set("class", "wrist_roll_stretch4")
    #     elif "gripper_finger_left_joint" in name:
    #         j.set("class", "gripper_left_finger")
    #     elif "gripper_finger_right_joint" in name:
    #         j.set("class", "gripper_right_finger")
    #     else:
    #         # urdf2mjcf assigns 'motor' by default. If it wasn't remapped above, it's invalid.
    #         if j.get("class") == "motor":
    #             raise ValueError(f"Unexpected joint '{name}' retains invalid default class 'motor'. Please add a dedicated class mapping in mjcf_generator.py")

    # Clean up the generated string and save
    ET.indent(new_root, space="  ", level=0)
    tree = ET.ElementTree(new_root)
    tree.write(out_mjcf_path, encoding="unicode", xml_declaration=True)

    # Inject ctrlrange into actuator_sensor.xml based on the extracted joint ranges
    try:
        import os

        actuator_xml_path = os.path.join(os.path.dirname(out_mjcf_path), "actuator_sensor.xml")
        out_actuator_xml_path = os.path.join(
            os.path.dirname(out_mjcf_path), "autogenerated_actuator_sensor.xml"
        )
        if os.path.exists(actuator_xml_path):
            act_tree = ET.parse(actuator_xml_path)
            act_root = act_tree.getroot()

            ranges_joint = {}
            for j in worldbody.findall(".//joint"):
                name = j.get("name")
                r = j.get("range")
                if name and r:
                    ranges_joint[name] = r

            # The arm tendon "extend" represents the sum of 4 telescope joints
            # if "arm_l4_joint" in ranges_joint:
            #     try:
            #         l_min, l_max = map(float, ranges_joint["arm_l4_joint"].split())
            #         ranges_joint["tendon_extend"] = f"{l_min * 4} {l_max * 4}"
            #     except ValueError:
            #         pass

            for pos in act_root.findall(".//position"):
                j_name = pos.get("joint")
                t_name = pos.get("tendon")

                r = None
                if j_name and j_name in ranges_joint:
                    r = ranges_joint[j_name]
                elif t_name == "extend" and "tendon_extend" in ranges_joint:
                    r = ranges_joint["tendon_extend"]

                if r:
                    pos.set("ctrlrange", r)
                    pos.set("ctrllimited", "true")

            ET.indent(act_root, space="  ", level=0)
            act_tree.write(out_actuator_xml_path, encoding="unicode", xml_declaration=True)
    except Exception as e:
        print(f"Failed to generate autogenerated_actuator_sensor.xml: {e}")

if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python mjcf_generator.py <urdf_path> [out_mjcf_path]")
        sys.exit(1)

    urdf_path = sys.argv[1]
    out_path = sys.argv[2] if len(sys.argv) > 2 else None
    generate_mjcf(urdf_path, out_path)