#!/usr/bin/env python3
"""
Generate MJCF for aidea from URDF, based on Stretch4's mjcf_generator.py.
"""

import os
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yourdfpy

from urdf2mjcf.convert import convert_urdf_to_mjcf
from urdf2mjcf.model import ConversionMetadata


def generate_mjcf(urdf_path: str, out_mjcf_path: str = None):
    """
    Generate aidea MJCF from URDF.
    """
    print(f"Generating mjcf from {urdf_path}")
    urdf_path = Path(urdf_path)
    if out_mjcf_path is None:
        out_mjcf_path = urdf_path.with_suffix(".mjcf")
    else:
        out_mjcf_path = Path(out_mjcf_path)

    # ==================================================================
    # 1. urdf2mjcf 初步转换
    # ==================================================================
    metadata = ConversionMetadata(
        height_offset=0.056,      # TODO: 按 aidea 底盘高度改
        freejoint=True,
        floating_base=True,
        cameras=[],
    )
    convert_urdf_to_mjcf(urdf_path, out_mjcf_path, metadata=metadata)

    # ==================================================================
    # 2. 解析 MJCF，重建根节点
    # ==================================================================
    tree = ET.parse(out_mjcf_path)
    root = tree.getroot()

    new_root = ET.Element("mujoco", model="aidea")
    asset = root.find("asset")
    worldbody = root.find("worldbody")
    if asset is None or worldbody is None:
        raise ValueError("urdf2mjcf output is missing 'asset' or 'worldbody' tags.")

    new_root.append(asset)
    new_root.append(worldbody)

    def find_body(name):
        return worldbody.find(f".//body[@name='{name}']")

    # ==================================================================
    # 3. 清理 mesh 名和材质
    # ==================================================================
    for mesh in asset.findall("mesh"):
        name = mesh.get("name")
        if name and (name.endswith(".STL") or name.endswith(".stl")):
            mesh.set("name", name[:-4])
        elif name and name.endswith(".dae"):
            mesh.set("name", name[:-4])

        file_attr = mesh.get("file")
        if file_attr and file_attr.startswith("file://"):
            mesh.set("file", file_attr[7:])

    for mat in asset.findall("material"):
        if mat.get("name") == "":
            asset.remove(mat)

    # ==================================================================
    # 4. geom class 和颜色（按 aidea 的 mesh 名改规则）
    # ==================================================================
    for geom in worldbody.findall(".//geom"):
        mesh_attr = geom.get("mesh")
        if mesh_attr and (mesh_attr.endswith(".STL") or mesh_attr.endswith(".dae")):
            geom.set("mesh", mesh_attr.rsplit(".", 1)[0])

        geom_class = geom.get("class")
        if geom_class == "visual":
            geom.set("class", "visualgeom")
            geom.set("density", "0")
            geom.set("group", "1")

            mesh_name = geom.get("mesh", "")
            # TODO: 按 aidea 的 mesh 名改颜色规则
            if "base" in mesh_name:
                geom.set("rgba", "0.5 0.5 0.5 1")
            elif "wheel" in mesh_name:
                geom.set("rgba", "0.15 0.15 0.15 1")
            elif "joint" in mesh_name:
                geom.set("rgba", "0.9 0.9 0.88 1")
            elif "gripper" in mesh_name:
                geom.set("rgba", "0.15 0.15 0.15 1")
            else:
                geom.set("rgba", "0.9 0.9 0.88 1")

        elif geom_class == "collision":
            # TODO: 按 aidea 的碰撞体名改规则
            geom_name = geom.get("name", "")
            if "gripper_l1" in geom_name or "gripper_r1" in geom_name:
                geom.set("class", "rubber")

        if "material" in geom.attrib and geom.attrib["material"] == "":
            del geom.attrib["material"]

    # ==================================================================
    # 5. 重命名 root body，加 imu site
    # ==================================================================
    base_body = worldbody.find("body")
    if base_body is not None:
        if base_body.get("name") != "aidea":
            base_body.set("name", "aidea")
        if "childclass" in base_body.attrib:
            del base_body.attrib["childclass"]
        ET.SubElement(base_body, "site", name="imu", size="0.01", pos="0 0 0")

    # ==================================================================
    # 6. 轮子（LIMO 的 4 个轮子，不需要替换，只改 class）
    # ==================================================================
    # urdf2mjcf 已经生成轮子 body，你只需要确保它们引用正确的 class
    for wheel_name in [
        "left_behind_wheel_link",
        "right_behind_wheel_link",
        "left_front_wheel_link",
        "right_front_wheel_link",
    ]:
        w_body = find_body(wheel_name)
        if w_body is not None:
            for geom in w_body.findall("geom"):
                # 视觉 geom 保持
                if geom.get("class") == "visualgeom":
                    continue
                # 碰撞 geom 改成 limo_wheel_collision
                geom.set("class", "limo_wheel_collision")

    # ==================================================================
    # 7. 从 URDF 添加相机（如果你的 aidea 有相机）
    # ==================================================================
    urdf = yourdfpy.URDF.load(urdf_path)

    def get_cam_pos_quat(link_name, parent_name, post_rotation=None):
        T_parent_link = urdf.get_transform(link_name, parent_name)
        T_parent_cam = T_parent_link
        if post_rotation is not None:
            T_parent_cam = T_parent_link @ post_rotation

        pos = T_parent_cam[:3, 3]
        quat = Rotation.from_matrix(T_parent_cam[:3, :3]).as_quat()
        quat_mj = f"{quat[3]} {quat[0]} {quat[1]} {quat[2]}"
        pos_mj = f"{pos[0]} {pos[1]} {pos[2]}"
        return pos_mj, quat_mj

    T_opt_cam = np.eye(4)
    T_opt_cam[:3, :3] = Rotation.from_euler("xyz", [180, 0, 0], degrees=True).as_matrix()

    # TODO: 按 aidea 的相机 link 名加
    # 例如：
    # camera_body = find_body("camera_link")
    # if camera_body is not None:
    #     pos, quat = get_cam_pos_quat("camera_link", "gripper_camera_mount_link", post_rotation=T_opt_cam)
    #     ET.SubElement(camera_body, "camera", name="camera", pos=pos, quat=quat)

    # ==================================================================
    # 8. gravcomp：给臂和夹爪加
    # ==================================================================
    for b in worldbody.findall(".//body"):
        name = b.get("name", "")
        if any(k in name for k in ["joint", "gripper"]):
            b.set("gravcomp", "1")

    # ==================================================================
    # 9. 关节 class 重映射
    # ==================================================================
    joint_class_map = {
        "joint2_to_joint1_joint": "mycobot_j1",
        "joint3_to_joint2_joint": "mycobot_j2",
        "joint4_to_joint3_joint": "mycobot_j3",
        "joint5_to_joint4_joint": "mycobot_j4",
        "joint6_to_joint5_joint": "mycobot_j5",
        "joint7_to_joint6_joint": "mycobot_j6",
        "gripper_controller_joint": "gripper_controller",
        "gripper_base_to_gripper_l2_joint": "gripper_mimic",
        "gripper_l3_to_gripper_l1_joint": "gripper_mimic",
        "gripper_base_to_gripper_r3_joint": "gripper_mimic",
        "gripper_base_to_gripper_r2_joint": "gripper_mimic",
        "gripper_r3_to_gripper_r1_joint": "gripper_mimic",
    }

    for j in worldbody.findall(".//joint"):
        name = j.get("name", "")
        if name in joint_class_map:
            j.set("class", joint_class_map[name])
        elif name.endswith("_wheel_joint"):
            j.set("class", "wheel")
        else:
            # 保持原 class，或报错
            pass

    # ==================================================================
    # 10. 保存 MJCF
    # ==================================================================
    ET.indent(new_root, space="  ", level=0)
    tree = ET.ElementTree(new_root)
    tree.write(out_mjcf_path, encoding="unicode", xml_declaration=True)

    # ==================================================================
    # 11. 注入 ctrlrange（可选，如果你用 actuator_sensor.xml）
    # ==================================================================
    try:
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

            for pos in act_root.findall(".//position"):
                j_name = pos.get("joint")
                if j_name and j_name in ranges_joint:
                    pos.set("ctrlrange", ranges_joint[j_name])
                    pos.set("ctrllimited", "true")

            ET.indent(act_root, space="  ", level=0)
            act_tree.write(out_actuator_xml_path, encoding="unicode", xml_declaration=True)
    except Exception as e:
        print(f"Failed to generate autogenerated_actuator_sensor.xml: {e}")


if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python mjcf_generator_aidea.py <urdf_path> [out_mjcf_path]")
        sys.exit(1)

    urdf_path = sys.argv[1]
    out_path = sys.argv[2] if len(sys.argv) > 2 else None
    generate_mjcf(urdf_path, out_path)