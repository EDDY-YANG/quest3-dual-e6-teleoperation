"""Construct the manufacturer's nominal flange chain for offline software verification.

The source is the distributed E6 URDF. No measured poses, saved start positions,
tool calibration, device model or sensor samples are used for this model.
"""
import xml.etree.ElementTree as ET
import numpy as np
from controller_kinematics import PoseModel, pose_matrix


def from_urdf(path):
    robot = ET.parse(path).getroot()
    joints = {j.find("parent").attrib["link"]: j for j in robot.findall("joint")}
    children = {j.find("child").attrib["link"] for j in joints.values()}
    link, = set(joints) - children
    transform = np.eye(4)
    axes, points = [], []
    for _ in range(len(joints)):
        joint = joints[link]
        origin = joint.find("origin")
        local = origin.attrib if origin is not None else {}
        xyz = np.array([float(v) for v in local.get("xyz", "0 0 0").split()]) * 1000
        angles = np.degrees([float(v) for v in local.get("rpy", "0 0 0").split()])
        transform = transform @ pose_matrix([*xyz, *angles])
        if joint.attrib["type"] == "revolute":
            axis = np.array([float(v) for v in joint.find("axis").attrib["xyz"].split()])
            axes.append(transform[:3, :3] @ axis)
            points.append(transform[:3, 3].copy())
        elif joint.attrib["type"] != "fixed":
            raise ValueError("Unsupported joint type in nominal E6 URDF")
        link = joint.find("child").attrib["link"]
    return PoseModel(transform, axes, points)
