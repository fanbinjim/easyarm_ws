"""Shared utilities for vision-based joint zero calibration."""

from pathlib import Path
import math
import re
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


CAMERA_MODEL = "MER2-301-125U3M"
CAMERA_SERIAL = "FCZ21070977"
CAMERA_INTRINSICS = Path(
    "data/camera_calibration/MER2-301-125U3M_FCZ21070977/20260526_171222/camera_calibration.yaml"
)
URDF_PATH = Path("src/easyarm_description/urdf/easyarm_a1_h0521.urdf")
ROS2_CONTROL_XACRO = Path("src/easyarm_a1_moveit_config/config/EasyARM-A1.ros2_control.xacro")

CHESSBOARD_COLS = 11
CHESSBOARD_ROWS = 8
SQUARE_SIZE_M = 0.01
BOARD_TO_FLANGE_DISTANCE_M = 0.0315

JOINT_NAMES = [f"Joint{i}" for i in range(1, 7)]
OPTIMIZED_JOINTS = ["Joint2", "Joint3", "Joint4", "Joint5"]

CAMERA_POSITION_BASE_M = np.array([0.700, 0.300, 0.080], dtype=float)
CAMERA_LOOK_AT_BASE_M = np.array([0.0, 0.0, 0.0], dtype=float)


def rpy_to_matrix(rpy):
    return Rotation.from_euler("xyz", rpy).as_matrix()


def transform_from_xyz_rpy(xyz, rpy):
    transform = np.eye(4)
    transform[:3, :3] = rpy_to_matrix(rpy)
    transform[:3, 3] = xyz
    return transform


def transform_from_rotvec_t(rotvec, translation):
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix()
    transform[:3, 3] = translation
    return transform


def rotvec_t_from_transform(transform):
    return Rotation.from_matrix(transform[:3, :3]).as_rotvec(), transform[:3, 3].copy()


def axis_angle_transform(axis, angle):
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_rotvec(np.asarray(axis, dtype=float) * angle).as_matrix()
    return transform


def make_chessboard_points():
    points = np.zeros((CHESSBOARD_ROWS * CHESSBOARD_COLS, 3), np.float64)
    grid = np.mgrid[0:CHESSBOARD_COLS, 0:CHESSBOARD_ROWS].T.reshape(-1, 2)
    points[:, :2] = grid * SQUARE_SIZE_M
    return points


def find_chessboard_corners(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, (CHESSBOARD_COLS, CHESSBOARD_ROWS), flags)
    if not found:
        return False, None

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, corners.reshape(-1, 2)


def load_camera_intrinsics(path=CAMERA_INTRINSICS):
    text = Path(path).read_text(encoding="utf-8")

    def read_list(name):
        match = re.search(rf"{name}:\s*\n(?:\s+.*\n)*?\s+data:\s*\[([^\]]+)\]", text)
        if not match:
            raise RuntimeError(f"Cannot find {name} in {path}")
        return [float(v.strip()) for v in match.group(1).split(",")]

    width = int(re.search(r"image_width:\s*(\d+)", text).group(1))
    height = int(re.search(r"image_height:\s*(\d+)", text).group(1))
    camera_matrix = np.asarray(read_list("camera_matrix"), dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(read_list("distortion_coefficients"), dtype=np.float64).reshape(-1)
    return camera_matrix, distortion, (width, height)


def parse_urdf_chain(path=URDF_PATH):
    root = ET.parse(path).getroot()
    joints = []
    for joint_name in JOINT_NAMES:
        joint = root.find(f"joint[@name='{joint_name}']")
        if joint is None:
            raise RuntimeError(f"Cannot find {joint_name} in {path}")
        origin = joint.find("origin")
        xyz = np.fromstring(origin.attrib.get("xyz", "0 0 0"), sep=" ")
        rpy = np.fromstring(origin.attrib.get("rpy", "0 0 0"), sep=" ")
        axis = np.fromstring(joint.find("axis").attrib["xyz"], sep=" ")
        axis = axis / np.linalg.norm(axis)
        joints.append({"name": joint_name, "origin": transform_from_xyz_rpy(xyz, rpy), "axis": axis})
    return joints


def fk_base_link6(joint_positions, chain):
    transform = np.eye(4)
    for joint in chain:
        q = joint_positions[joint["name"]]
        transform = transform @ joint["origin"] @ axis_angle_transform(joint["axis"], q)
    return transform


def initial_camera_base_transform():
    camera_pos = CAMERA_POSITION_BASE_M
    forward = CAMERA_LOOK_AT_BASE_M - camera_pos
    forward = forward / np.linalg.norm(forward)
    world_up = np.array([0.0, 0.0, 1.0])
    right = np.cross(forward, world_up)
    if np.linalg.norm(right) < 1e-6:
        right = np.array([1.0, 0.0, 0.0])
    right = right / np.linalg.norm(right)
    down = np.cross(forward, right)
    down = down / np.linalg.norm(down)

    rotation_base_camera = np.column_stack((right, down, forward))
    rotation_camera_base = rotation_base_camera.T
    transform = np.eye(4)
    transform[:3, :3] = rotation_camera_base
    transform[:3, 3] = -rotation_camera_base @ camera_pos
    return transform


def initial_link6_board_transform():
    board_width = (CHESSBOARD_COLS - 1) * SQUARE_SIZE_M
    board_height = (CHESSBOARD_ROWS - 1) * SQUARE_SIZE_M
    transform = np.eye(4)
    transform[:3, 3] = np.array([-board_width / 2.0, -board_height / 2.0, BOARD_TO_FLANGE_DISTANCE_M])
    return transform


def parse_xacro_joint_offsets(path=ROS2_CONTROL_XACRO):
    text = Path(path).read_text(encoding="utf-8")
    values = {}
    for joint_name in JOINT_NAMES:
        match = re.search(rf'<joint name="{joint_name}">(.*?)</joint>', text, re.S)
        if not match:
            raise RuntimeError(f"Cannot find {joint_name} in {path}")
        block = match.group(1)
        direction = float(re.search(r'<param name="direction">([^<]+)</param>', block).group(1))
        offset = float(re.search(r'<param name="position_offset">([^<]+)</param>', block).group(1))
        values[joint_name] = {"direction": 1.0 if direction >= 0.0 else -1.0, "position_offset": offset}
    return values


def project_points(points_camera, camera_matrix, distortion):
    projected, _ = cv2.projectPoints(
        points_camera.reshape(-1, 1, 3),
        np.zeros(3),
        np.zeros(3),
        camera_matrix,
        distortion,
    )
    return projected.reshape(-1, 2)


def transform_points(transform, points):
    hom = np.c_[points, np.ones(len(points))]
    transformed = (transform @ hom.T).T
    return transformed[:, :3]


def radians_to_degrees(values):
    return {name: math.degrees(value) for name, value in values.items()}
