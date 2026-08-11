#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import threading
import glob
import re
import csv
from datetime import datetime

import cv2
import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pybullet as p
import pybullet_data
import pyzed.sl as sl

# ====== ROS2 ======
import rclpy
from rclpy.node import Node
from tm_msgs.srv import SendScript, SetEvent
from tm_msgs.msg import FeedbackState


# =========================
# Path settings
# =========================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

def rel_path(*parts):
    return os.path.join(BASE_DIR, *parts)

FLOOR_TEXTURE_PATH = rel_path("images.png")
TM5_URDF_PATH = rel_path("tm_description", "urdf", "tm5-900.urdf")

# TCP speed 紀錄輸出資料夾
OUTPUT_DIR = rel_path("output")


# =========================
# ZED Body Tracking settings
# =========================
TARGET_FPS = 30.0

ZED_RESOLUTION = sl.RESOLUTION.HD720
ZED_FPS = 30
ZED_DEPTH_MODE = sl.DEPTH_MODE.NEURAL
ZED_COORD_UNIT = sl.UNIT.MILLIMETER

ZED_BODY_FORMAT = sl.BODY_FORMAT.BODY_34
ZED_DETECTION_CONFIDENCE = 40
ZED_KEYPOINT_CONFIDENCE = 20
ZED_ENABLE_BODY_FITTING = True

MAX_PERSONS = 10
NUM_ZED_KPTS = 34


# =========================
# ZED stereo calibration folder settings
# =========================
# 以本程式所在資料夾為基準，避免綁定特定使用者或工作區路徑
CALIB_ROOT = rel_path("zed_chessboard")

CALIB_LEFT_DIR_CANDIDATES = [
    os.path.join(CALIB_ROOT, "img_l"),
    os.path.join(CALIB_ROOT, "img_left"),
]

CALIB_RIGHT_DIR_CANDIDATES = [
    os.path.join(CALIB_ROOT, "img_r"),
    os.path.join(CALIB_ROOT, "img_right"),
]

# 拍攝棋盤格照片時的 ZED 解析度，要和照片解析度一致
CALIB_ZED_RESOLUTION = sl.RESOLUTION.HD1080
CALIB_ZED_FPS = 30

# 棋盤格內角點數量，不是格子數
CHECKERBOARD_SIZE = (4, 3)

# 棋盤格單格尺寸，單位 mm
SQUARE_SIZE_MM = 71.0

# 優先使用指定編號的左右照片，例如 left_0.png / right_0.png
# 若設為 None，則使用第 CALIB_VALID_IMAGE_INDEX 組有效照片
CALIB_PAIR_ID = 0
CALIB_VALID_IMAGE_INDEX = 0

# 使用 sl.VIEW.LEFT / sl.VIEW.RIGHT 拍出來通常是 rectified image
USE_RECTIFIED_ZED_IMAGES = True


# ============================================================
# T_BASE_CHESS_OLD_MM = base <- chessboard
#
# 這裡填入「原始」棋盤格座標系到機械手臂 base 座標系的矩陣。
# 程式會自動執行：
#
#     T_BASE_CHESS_MM = T_BASE_CHESS_OLD_MM @ Ry(180°)
#
# 因此不需要再手動改矩陣正負號。
# ============================================================
T_BASE_CHESS_OLD_MM = np.array([[-0.0000, -0.0000, -1.0000, 355.0000],
 [1.0000, 0.0000, -0.0000, 106.5000],
 [0.0000, -1.0000, 0.0000, 771.0000],
 [0.0000, 0.0000, 0.0000, 1.0000]], dtype=np.float64)

# Ry(180°) 的 4x4 齊次旋轉矩陣
RY_180 = np.array([
    [-1.0,  0.0,  0.0, 0.0],
    [ 0.0,  1.0,  0.0, 0.0],
    [ 0.0,  0.0, -1.0, 0.0],
    [ 0.0,  0.0,  0.0, 1.0],
], dtype=np.float64)

# 自動套用棋盤格座標修正
T_BASE_CHESS_MM = T_BASE_CHESS_OLD_MM @ RY_180

print("\n[CALIB] T_BASE_CHESS_OLD_MM =")
print(T_BASE_CHESS_OLD_MM)
print("\n[CALIB] T_BASE_CHESS_MM = T_BASE_CHESS_OLD_MM @ Ry(180°)")
print(T_BASE_CHESS_MM)


# =========================
# ZED BODY_34 skeleton edges
# =========================
SKELETON_EDGES = [
    (0, 1), (1, 2), (2, 3), (3, 26),

    (3, 4), (4, 5), (5, 6), (6, 7),
    (7, 8), (8, 9), (7, 10),

    (3, 11), (11, 12), (12, 13), (13, 14),
    (14, 15), (15, 16), (14, 17),

    (0, 18), (18, 19), (19, 20), (20, 21), (20, 32),

    (0, 22), (22, 23), (23, 24), (24, 25), (24, 33),

    (26, 27), (27, 28), (28, 29),
    (27, 30), (30, 31),
]

COLORS3D = [(0.0, 0.0, 0.0)] * MAX_PERSONS


# =========================
# Safety settings
# =========================
PLATFORM_SIZE_M    = 0.20
PLATFORM_HEIGHT_M  = 0.69
FLOOR_Z            = -0.69

AABB_MARGIN_M      = 0.05
EMA_ALPHA          = 0.25
AABB_SCALE         = 1.5

SLOW_BOX_CENTER        = [0.0, 0.0, 0.0]
SLOW_BOX_HALF_EXTENT_M = 2.0
SLOW_BOX_COLOR         = [0, 0, 1]
SLOW_BOX_ALERT         = [1.0, 0.5, 0.0]
SLOW_BOX_LINE_WIDTH    = 4

OVERLAY_LINE_WIDTH = 15
BBOX_LINE_WIDTH    = 2

USE_XY_SQUARE_MASK = True
XY_MASK_CENTER_M   = (0.0, 0.0)
XY_MASK_HALF_M     = 2.5
XY_MASK_Z_RANGE_M  = None

# 額外平移補償，單位 m
BIAS_XY_M = (+0., +0.3)
BIAS_MM   = np.array(
    [BIAS_XY_M[0] * 1000.0, BIAS_XY_M[1] * 1000.0, 0.0],
    dtype=np.float64
)

RESUME_COOLDOWN_SEC = 0.5


# =========================
# Transform helpers
# =========================
def inverse_T(T):
    T = np.asarray(T, dtype=np.float64)
    Rm = T[:3, :3]
    t = T[:3, 3]

    Ti = np.eye(4, dtype=np.float64)
    Ti[:3, :3] = Rm.T
    Ti[:3, 3] = -Rm.T @ t
    return Ti


def reorthonormalize_transform(T):
    T = np.asarray(T, dtype=np.float64).copy()
    Rm = T[:3, :3]

    U, _, Vt = np.linalg.svd(Rm)
    Rn = U @ Vt

    if np.linalg.det(Rn) < 0:
        U[:, -1] *= -1
        Rn = U @ Vt

    T[:3, :3] = Rn
    return T


def to_homogeneous(Rm, tvec):
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(Rm, dtype=np.float64)
    T[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    return T


def transform_points_mm(T, pts_mm):
    pts_mm = np.asarray(pts_mm, dtype=np.float64)
    out = np.full_like(pts_mm, np.nan, dtype=np.float64)

    if pts_mm.ndim != 2 or pts_mm.shape[1] != 3:
        return out

    valid = np.isfinite(pts_mm).all(axis=1)
    if not np.any(valid):
        return out

    ph = np.ones((np.count_nonzero(valid), 4), dtype=np.float64)
    ph[:, :3] = pts_mm[valid]

    transformed = (T @ ph.T).T[:, :3]
    out[valid] = transformed
    return out


def transform_point_mm(T, pt_mm):
    pt_mm = np.asarray(pt_mm, dtype=np.float64).reshape(3)
    if not np.isfinite(pt_mm).all():
        return None

    ph = np.array([pt_mm[0], pt_mm[1], pt_mm[2], 1.0], dtype=np.float64)
    out = T @ ph
    return out[:3]


# =========================
# Stereo calibration helpers
# =========================
def extract_number(path):
    name = os.path.basename(path)
    m = re.search(r"(\d+)", name)
    return int(m.group(1)) if m else -1


def find_existing_dir(candidates, name):
    for d in candidates:
        if os.path.isdir(d):
            return d

    raise FileNotFoundError(
        f"\n找不到 {name} 資料夾，已搜尋：\n" +
        "\n".join([f"  {d}" for d in candidates])
    )


def find_stereo_calibration_pairs():
    left_dir = find_existing_dir(CALIB_LEFT_DIR_CANDIDATES, "左影像")
    right_dir = find_existing_dir(CALIB_RIGHT_DIR_CANDIDATES, "右影像")

    left_patterns = [
        os.path.join(left_dir, "left_*.png"),
        os.path.join(left_dir, "zed_left_*.png"),
        os.path.join(left_dir, "*.png"),
        os.path.join(left_dir, "*.jpg"),
        os.path.join(left_dir, "*.jpeg"),
    ]

    right_patterns = [
        os.path.join(right_dir, "right_*.png"),
        os.path.join(right_dir, "zed_right_*.png"),
        os.path.join(right_dir, "*.png"),
        os.path.join(right_dir, "*.jpg"),
        os.path.join(right_dir, "*.jpeg"),
    ]

    left_paths = []
    right_paths = []

    for pat in left_patterns:
        left_paths.extend(glob.glob(pat))

    for pat in right_patterns:
        right_paths.extend(glob.glob(pat))

    left_paths = sorted(set(left_paths), key=extract_number)
    right_paths = sorted(set(right_paths), key=extract_number)

    left_by_id = {}
    right_by_id = {}

    for path in left_paths:
        idx = extract_number(path)
        if idx >= 0:
            left_by_id[idx] = path

    for path in right_paths:
        idx = extract_number(path)
        if idx >= 0:
            right_by_id[idx] = path

    common_ids = sorted(set(left_by_id.keys()) & set(right_by_id.keys()))

    pairs = []
    for idx in common_ids:
        pairs.append((idx, left_by_id[idx], right_by_id[idx]))

    if not pairs:
        raise FileNotFoundError(
            f"\n找不到可配對的 ZED 左右棋盤格照片。\n"
            f"左資料夾：{left_dir}\n"
            f"右資料夾：{right_dir}\n"
            f"請確認檔名有相同編號，例如 left_0.png / right_0.png。\n"
        )

    print("\n[CALIB] 找到左右校正照片配對：")
    for idx, lp, rp in pairs:
        print(f"  id={idx}:")
        print(f"    L: {lp}")
        print(f"    R: {rp}")

    return pairs


def make_chessboard_object_points():
    objp = np.zeros(
        (CHECKERBOARD_SIZE[0] * CHECKERBOARD_SIZE[1], 3),
        dtype=np.float64
    )

    objp[:, :2] = SQUARE_SIZE_MM * np.mgrid[
        0:CHECKERBOARD_SIZE[0],
        0:CHECKERBOARD_SIZE[1]
    ].T.reshape(-1, 2)

    return objp


def find_chessboard_corners(image_path):
    img = cv2.imread(image_path)

    if img is None:
        raise RuntimeError(f"讀取影像失敗：{image_path}")

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        + cv2.CALIB_CB_NORMALIZE_IMAGE
    )

    ok, corners = cv2.findChessboardCorners(
        gray,
        CHECKERBOARD_SIZE,
        flags
    )

    if not ok:
        return None

    criteria = (
        cv2.TERM_CRITERIA_EPS
        + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001
    )

    corners = cv2.cornerSubPix(
        gray,
        corners,
        (11, 11),
        (-1, -1),
        criteria
    )

    return corners.reshape(-1, 2).astype(np.float64)


def rigid_transform_3d(A, B):
    """
    求剛體轉換：
        B ≈ R @ A + t

    A: chessboard object points, Nx3
    B: stereo triangulated points in left optical frame, Nx3

    return:
        T_leftopt_chess = left optical <- chess
    """
    A = np.asarray(A, dtype=np.float64)
    B = np.asarray(B, dtype=np.float64)

    if A.shape != B.shape:
        raise ValueError(f"A/B shape 不一致：A={A.shape}, B={B.shape}")

    centroid_A = A.mean(axis=0)
    centroid_B = B.mean(axis=0)

    AA = A - centroid_A
    BB = B - centroid_B

    H = AA.T @ BB
    U, _, Vt = np.linalg.svd(H)

    Rm = Vt.T @ U.T

    if np.linalg.det(Rm) < 0:
        Vt[-1, :] *= -1
        Rm = Vt.T @ U.T

    t = centroid_B - Rm @ centroid_A

    T = to_homogeneous(Rm, t)
    return reorthonormalize_transform(T)


def _translation_to_numpy_mm(tr):
    """
    盡量相容不同 pyzed 版本的 Translation 型態。
    """
    arr = None

    try:
        arr = np.array(tr.get(), dtype=np.float64).reshape(3)
    except Exception:
        pass

    if arr is None:
        try:
            arr = np.array([tr.x, tr.y, tr.z], dtype=np.float64)
        except Exception:
            pass

    if arr is None:
        try:
            arr = np.array(tr, dtype=np.float64).reshape(3)
        except Exception:
            pass

    return arr


def get_zed_stereo_calibration_from_camera():
    """
    從目前接上的 ZED 讀取雙目校正參數。
    不讀 zed_left_intrinsics.json。
    """
    zed_tmp = sl.Camera()

    init_params = sl.InitParameters()
    init_params.camera_resolution = CALIB_ZED_RESOLUTION
    init_params.camera_fps = CALIB_ZED_FPS
    init_params.coordinate_units = sl.UNIT.MILLIMETER

    try:
        init_params.depth_mode = sl.DEPTH_MODE.NONE
    except Exception:
        pass

    err = zed_tmp.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"讀取 ZED 雙目校正資料時，ZED 開啟失敗：{err}")

    cam_info = zed_tmp.get_camera_information()
    calib = cam_info.camera_configuration.calibration_parameters

    left = calib.left_cam
    right = calib.right_cam

    K1 = np.array([
        [float(left.fx), 0.0, float(left.cx)],
        [0.0, float(left.fy), float(left.cy)],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)

    K2 = np.array([
        [float(right.fx), 0.0, float(right.cx)],
        [0.0, float(right.fy), float(right.cy)],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)

    if USE_RECTIFIED_ZED_IMAGES:
        D1 = np.zeros((5, 1), dtype=np.float64)
        D2 = np.zeros((5, 1), dtype=np.float64)
        R_lr = np.eye(3, dtype=np.float64)
    else:
        try:
            D1 = np.asarray(left.disto, dtype=np.float64).reshape(-1, 1)
        except Exception:
            D1 = np.zeros((5, 1), dtype=np.float64)

        try:
            D2 = np.asarray(right.disto, dtype=np.float64).reshape(-1, 1)
        except Exception:
            D2 = np.zeros((5, 1), dtype=np.float64)

        R_lr = np.eye(3, dtype=np.float64)

    t_lr = None

    try:
        baseline = float(calib.get_camera_baseline())
        if abs(baseline) < 10.0:
            baseline *= 1000.0
        t_lr = np.array([baseline, 0.0, 0.0], dtype=np.float64)
    except Exception:
        pass

    if t_lr is None:
        try:
            st = calib.stereo_transform
            tr = st.get_translation()
            t_lr = _translation_to_numpy_mm(tr)
            if t_lr is not None and np.linalg.norm(t_lr) < 10.0:
                t_lr *= 1000.0
        except Exception:
            pass

    if t_lr is None:
        # ZED / ZED2 常見 baseline 約 120 mm
        t_lr = np.array([120.0, 0.0, 0.0], dtype=np.float64)

    zed_tmp.close()

    print("\n[CALIB] ZED stereo calibration loaded from SDK")
    print("[CALIB] K_left =")
    print(K1)
    print("[CALIB] K_right =")
    print(K2)
    print("[CALIB] R_left_to_right =")
    print(R_lr)
    print("[CALIB] t_left_to_right mm =")
    print(t_lr)

    return K1, D1, K2, D2, R_lr, t_lr


def triangulate_chessboard_from_stereo(corners_l, corners_r, K1, D1, K2, D2, R_lr, t_lr):
    """
    使用左右棋盤角點三角化，得到每個棋盤角點在 left optical frame 下的 3D 位置。
    """
    pts_l = corners_l.reshape(-1, 1, 2).astype(np.float64)
    pts_r = corners_r.reshape(-1, 1, 2).astype(np.float64)

    pts_l_norm = cv2.undistortPoints(pts_l, K1, D1).reshape(-1, 2)
    pts_r_norm = cv2.undistortPoints(pts_r, K2, D2).reshape(-1, 2)

    P1 = np.hstack([
        np.eye(3, dtype=np.float64),
        np.zeros((3, 1), dtype=np.float64)
    ])

    P2 = np.hstack([
        R_lr.astype(np.float64),
        t_lr.reshape(3, 1).astype(np.float64)
    ])

    ph = cv2.triangulatePoints(
        P1,
        P2,
        pts_l_norm.T,
        pts_r_norm.T
    )

    X = (ph[:3] / (ph[3] + 1e-12)).T

    # 若深度為負，代表 baseline 方向可能相反，嘗試翻轉 t
    if np.nanmedian(X[:, 2]) < 0:
        P2_flip = np.hstack([
            R_lr.astype(np.float64),
            (-t_lr).reshape(3, 1).astype(np.float64)
        ])

        ph_flip = cv2.triangulatePoints(
            P1,
            P2_flip,
            pts_l_norm.T,
            pts_r_norm.T
        )

        X_flip = (ph_flip[:3] / (ph_flip[3] + 1e-12)).T

        if np.nanmedian(X_flip[:, 2]) > np.nanmedian(X[:, 2]):
            X = X_flip

    return X


def compute_T_zed_to_base_from_stereo_folder():
    pairs = find_stereo_calibration_pairs()
    K1, D1, K2, D2, R_lr, t_lr = get_zed_stereo_calibration_from_camera()

    objp = make_chessboard_object_points()

    valid_count = 0
    selected = None

    for idx, left_path, right_path in pairs:
        if CALIB_PAIR_ID is not None and idx != CALIB_PAIR_ID:
            continue

        corners_l = find_chessboard_corners(left_path)
        corners_r = find_chessboard_corners(right_path)

        if corners_l is None:
            print(f"[CALIB] skip left chessboard not found: {left_path}")
            continue

        if corners_r is None:
            print(f"[CALIB] skip right chessboard not found: {right_path}")
            continue

        X_leftopt = triangulate_chessboard_from_stereo(
            corners_l,
            corners_r,
            K1,
            D1,
            K2,
            D2,
            R_lr,
            t_lr
        )

        valid = np.isfinite(X_leftopt).all(axis=1)

        if np.count_nonzero(valid) < 6:
            print(f"[CALIB] skip triangulation invalid: pair id={idx}")
            continue

        T_leftopt_chess = rigid_transform_3d(
            objp[valid],
            X_leftopt[valid]
        )

        X_fit = (
            T_leftopt_chess[:3, :3] @ objp[valid].T
            + T_leftopt_chess[:3, 3:4]
        ).T

        fit_err_mm = float(
            np.mean(np.linalg.norm(X_fit - X_leftopt[valid], axis=1))
        )

        selected = {
            "pair_id": idx,
            "left_path": left_path,
            "right_path": right_path,
            "T_leftopt_chess": T_leftopt_chess,
            "fit_err_mm": fit_err_mm,
            "median_depth_mm": float(np.nanmedian(X_leftopt[:, 2])),
        }
        break

    if selected is None and CALIB_PAIR_ID is not None:
        raise RuntimeError(
            f"\n找不到指定編號 CALIB_PAIR_ID={CALIB_PAIR_ID} 的有效左右棋盤格照片。\n"
            f"請確認 left_{CALIB_PAIR_ID}.png / right_{CALIB_PAIR_ID}.png 皆存在且能偵測棋盤格。\n"
        )

    if selected is None:
        valid_count = 0

        for idx, left_path, right_path in pairs:
            corners_l = find_chessboard_corners(left_path)
            corners_r = find_chessboard_corners(right_path)

            if corners_l is None or corners_r is None:
                continue

            X_leftopt = triangulate_chessboard_from_stereo(
                corners_l,
                corners_r,
                K1,
                D1,
                K2,
                D2,
                R_lr,
                t_lr
            )

            valid = np.isfinite(X_leftopt).all(axis=1)

            if np.count_nonzero(valid) < 6:
                continue

            if valid_count == CALIB_VALID_IMAGE_INDEX:
                T_leftopt_chess = rigid_transform_3d(
                    objp[valid],
                    X_leftopt[valid]
                )

                X_fit = (
                    T_leftopt_chess[:3, :3] @ objp[valid].T
                    + T_leftopt_chess[:3, 3:4]
                ).T

                fit_err_mm = float(
                    np.mean(np.linalg.norm(X_fit - X_leftopt[valid], axis=1))
                )

                selected = {
                    "pair_id": idx,
                    "left_path": left_path,
                    "right_path": right_path,
                    "T_leftopt_chess": T_leftopt_chess,
                    "fit_err_mm": fit_err_mm,
                    "median_depth_mm": float(np.nanmedian(X_leftopt[:, 2])),
                }
                break

            valid_count += 1

    if selected is None:
        raise RuntimeError(
            f"\n校正資料夾中找不到第 {CALIB_VALID_IMAGE_INDEX} 組有效左右棋盤格照片。\n"
        )

    T_leftopt_chess = selected["T_leftopt_chess"]

    # T_leftopt_chess = left optical <- chess
    T_chess_leftopt = inverse_T(T_leftopt_chess)

    # T_BASE_CHESS_MM = base <- chess
    T_base_leftopt = T_BASE_CHESS_MM @ T_chess_leftopt

    # ZED Body Tracking 使用 RIGHT_HANDED_Z_UP_X_FWD：
    # X forward, Y left, Z up
    #
    # OpenCV left optical frame：
    # x right, y down, z forward
    #
    # x_opt = -Y_zed
    # y_opt = -Z_zed
    # z_opt =  X_zed
    T_leftopt_zed = np.eye(4, dtype=np.float64)
    T_leftopt_zed[:3, :3] = np.array([
        [0.0, -1.0,  0.0],
        [0.0,  0.0, -1.0],
        [1.0,  0.0,  0.0],
    ], dtype=np.float64)

    T_zed_to_base = T_base_leftopt @ T_leftopt_zed
    T_zed_to_base = reorthonormalize_transform(T_zed_to_base)

    np.set_printoptions(suppress=True, precision=4)

    print("\n================ ZED stereo hand-eye calibration ================")
    print(f"[CALIB] CALIB_ROOT = {CALIB_ROOT}")
    print(f"[CALIB] pair id = {selected['pair_id']}")
    print(f"[CALIB] left  = {selected['left_path']}")
    print(f"[CALIB] right = {selected['right_path']}")
    print(f"[CALIB] stereo 3D fitting error = {selected['fit_err_mm']:.4f} mm")
    print(f"[CALIB] median chessboard depth = {selected['median_depth_mm']:.2f} mm")

    print("\n[CALIB] T_BASE_CHESS_MM = base <- chess")
    print(T_BASE_CHESS_MM)

    print("\n[CALIB] T_leftopt_chess = left optical <- chess")
    print(T_leftopt_chess)

    print("\n[CALIB] T_ZED_TO_BASE_MM = base <- zed body coordinate")
    print(T_zed_to_base)
    print("=================================================================\n")

    return T_zed_to_base


# 真正給後面 ZED 骨架轉換用的矩陣
T_ZED_TO_BASE_MM = compute_T_zed_to_base_from_stereo_folder()


class Human3D:
    def __init__(self, body_id, center_mm, X_full_mm):
        self.id = int(body_id)
        self.center = np.asarray(center_mm, dtype=np.float64)
        self.X_full_mm = np.asarray(X_full_mm, dtype=np.float64)


class TMSimulator:
    def __init__(self):
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

        self.joint_indices = [1, 2, 3, 4, 5, 6]

        self.last_green_exit = None
        self.last_slow_exit = None

        self._add_floor_and_platform()

        self.tm5_id = p.loadURDF(
            TM5_URDF_PATH,
            basePosition=[0, 0, 0],
            useFixedBase=True
        )

        nj = p.getNumJoints(self.tm5_id)
        p.changeVisualShape(self.tm5_id, -1, rgbaColor=[1, 1, 1, 1])
        for link_idx in range(nj):
            p.changeVisualShape(self.tm5_id, link_idx, rgbaColor=[1, 1, 1, 1])

        self._bbox_lock = threading.Lock()
        self.current_bounds = [-1.1, 1.1, -1.1, 1.1, FLOOR_Z, 1.3]

        self._ema_c = None
        self._ema_r = None
        self.ema_alpha = EMA_ALPHA
        self.aabb_scale = AABB_SCALE

        self.bbox_ids = None
        self.overlay_ids = None
        self.set_bbox_bounds(
            *self.current_bounds,
            color=[0, 1, 0],
            line_width=BBOX_LINE_WIDTH
        )

        self.slow_box_ids = None
        self.slow_bounds = None

        cx, cy, cz = SLOW_BOX_CENTER
        r = float(SLOW_BOX_HALF_EXTENT_M)
        self.set_slow_bbox_bounds(
            cx - r, cx + r,
            cy - r, cy + r,
            cz - r, cz + r,
            color=SLOW_BOX_COLOR,
            line_width=SLOW_BOX_LINE_WIDTH
        )

        self.skel_line_ids = []
        for _ in range(MAX_PERSONS):
            line_set = []
            for _edge in SKELETON_EDGES:
                lid = p.addUserDebugLine(
                    [0, 0, 0],
                    [0, 0, 0],
                    [0, 0, 0],
                    6,
                    0
                )
                line_set.append(lid)
            self.skel_line_ids.append(line_set)

    def _add_floor_and_platform(self):
        plane_id = p.loadURDF("plane.urdf")

        try:
            if os.path.isfile(FLOOR_TEXTURE_PATH):
                tex = p.loadTexture(FLOOR_TEXTURE_PATH)
                p.changeVisualShape(plane_id, -1, textureUniqueId=tex)
        except Exception:
            pass

        p.resetBasePositionAndOrientation(
            plane_id,
            [0, 0, FLOOR_Z],
            [0, 0, 0, 1]
        )

        size = float(PLATFORM_SIZE_M)
        height = float(PLATFORM_HEIGHT_M)
        half = [size / 2, size / 2, height / 2]

        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
        vis = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=half,
            rgbaColor=[0.6, 0.6, 0.6, 1]
        )

        base_pos = [0, 0, FLOOR_Z + height / 2]

        p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=base_pos
        )

    def _edges_from_bounds(self, xm, xM, ym, yM, zm, zM):
        return [
            ([xm, ym, zm], [xM, ym, zm]),
            ([xM, ym, zm], [xM, yM, zm]),
            ([xM, yM, zm], [xm, yM, zm]),
            ([xm, yM, zm], [xm, ym, zm]),

            ([xm, ym, zM], [xM, ym, zM]),
            ([xM, ym, zM], [xM, yM, zM]),
            ([xM, yM, zM], [xm, yM, zM]),
            ([xm, yM, zM], [xm, ym, zM]),

            ([xm, ym, zm], [xm, ym, zM]),
            ([xM, ym, zm], [xM, ym, zM]),
            ([xM, yM, zm], [xM, yM, zM]),
            ([xm, yM, zm], [xm, yM, zM]),
        ]

    def set_bbox_bounds(self, xm, xM, ym, yM, zm, zM,
                        color=[0, 1, 0],
                        line_width=BBOX_LINE_WIDTH):
        edges = self._edges_from_bounds(xm, xM, ym, yM, zm, zM)

        with self._bbox_lock:
            self.current_bounds = [xm, xM, ym, yM, zm, zM]

            if self.bbox_ids is None:
                self.bbox_ids = []
                for s, e in edges:
                    lid = p.addUserDebugLine(s, e, color, line_width, 0)
                    self.bbox_ids.append(lid)
            else:
                for idx, (s, e) in enumerate(edges):
                    p.addUserDebugLine(
                        s, e, color, line_width, 0,
                        replaceItemUniqueId=self.bbox_ids[idx]
                    )

    def update_bounding_box_color(self, color, line_width):
        with self._bbox_lock:
            xm, xM, ym, yM, zm, zM = self.current_bounds

        edges = self._edges_from_bounds(xm, xM, ym, yM, zm, zM)

        for idx, (s, e) in enumerate(edges):
            p.addUserDebugLine(
                s, e, color, line_width, 0,
                replaceItemUniqueId=self.bbox_ids[idx]
            )

    def show_overlay_from_current(self,
                                  color=[1, 0, 0],
                                  line_width=OVERLAY_LINE_WIDTH):
        with self._bbox_lock:
            xm, xM, ym, yM, zm, zM = self.current_bounds
            edges = self._edges_from_bounds(xm, xM, ym, yM, zm, zM)

            if self.overlay_ids is None:
                self.overlay_ids = []
                for s, e in edges:
                    lid = p.addUserDebugLine(s, e, color, line_width, 0)
                    self.overlay_ids.append(lid)
            else:
                for idx, (s, e) in enumerate(edges):
                    p.addUserDebugLine(
                        s, e, color, line_width, 0,
                        replaceItemUniqueId=self.overlay_ids[idx]
                    )

    def hide_overlay(self):
        if self.overlay_ids is not None:
            for lid in self.overlay_ids:
                p.addUserDebugLine(
                    [0, 0, 0],
                    [0, 0, 0],
                    [0, 0, 0],
                    1,
                    0,
                    replaceItemUniqueId=lid
                )
            self.overlay_ids = None

    def set_slow_bbox_bounds(self, xm, xM, ym, yM, zm, zM,
                             color=SLOW_BOX_COLOR,
                             line_width=SLOW_BOX_LINE_WIDTH):
        edges = self._edges_from_bounds(xm, xM, ym, yM, zm, zM)

        with self._bbox_lock:
            self.slow_bounds = [xm, xM, ym, yM, zm, zM]

            if self.slow_box_ids is None:
                self.slow_box_ids = []
                for s, e in edges:
                    lid = p.addUserDebugLine(s, e, color, line_width, 0)
                    self.slow_box_ids.append(lid)
            else:
                for idx, (s, e) in enumerate(edges):
                    p.addUserDebugLine(
                        s, e, color, line_width, 0,
                        replaceItemUniqueId=self.slow_box_ids[idx]
                    )

    def update_slow_box_color(self,
                              color=SLOW_BOX_COLOR,
                              line_width=SLOW_BOX_LINE_WIDTH):
        with self._bbox_lock:
            if self.slow_bounds is None or self.slow_box_ids is None:
                return
            xm, xM, ym, yM, zm, zM = self.slow_bounds

        edges = self._edges_from_bounds(xm, xM, ym, yM, zm, zM)

        for idx, (s, e) in enumerate(edges):
            p.addUserDebugLine(
                s, e, color, line_width, 0,
                replaceItemUniqueId=self.slow_box_ids[idx]
            )

    def get_slow_bounds(self):
        with self._bbox_lock:
            return tuple(self.slow_bounds) if self.slow_bounds else None

    def update_joint_angles(self, joint_angles):
        for i, angle in enumerate(joint_angles):
            p.resetJointState(self.tm5_id, self.joint_indices[i], angle)

        num_links = p.getNumJoints(self.tm5_id)

        mn = np.array(p.getAABB(self.tm5_id, -1)[0])
        mx = np.array(p.getAABB(self.tm5_id, -1)[1])

        for link_idx in range(num_links):
            aabb = p.getAABB(self.tm5_id, link_idx)
            mn = np.minimum(mn, aabb[0])
            mx = np.maximum(mx, aabb[1])

        mn -= AABB_MARGIN_M
        mx += AABB_MARGIN_M

        c_raw = (mn + mx) / 2.0
        r_raw = np.max((mx - mn) / 2.0) * float(self.aabb_scale)

        if self._ema_c is None:
            self._ema_c = c_raw
            self._ema_r = r_raw
        else:
            a = self.ema_alpha
            self._ema_c = (1 - a) * self._ema_c + a * c_raw
            self._ema_r = (1 - a) * self._ema_r + a * r_raw

        mn2 = self._ema_c - self._ema_r
        mx2 = self._ema_c + self._ema_r

        xm, ym, zm = mn2.tolist()
        xM, yM, zM = mx2.tolist()

        self.set_bbox_bounds(xm, xM, ym, yM, zm, zM)

    def draw_skeleton_slot(self, slot_idx, pts3d_m, valid_mask, color_rgb):
        if not (0 <= slot_idx < len(self.skel_line_ids)):
            return

        if pts3d_m is None or valid_mask is None:
            self.clear_skeleton_slot(slot_idx)
            return

        line_set = self.skel_line_ids[slot_idx]

        for k, (i, j) in enumerate(SKELETON_EDGES):
            is_valid = (
                i < len(valid_mask)
                and j < len(valid_mask)
                and valid_mask[i]
                and valid_mask[j]
            )

            if is_valid:
                p1 = pts3d_m[i].tolist()
                p2 = pts3d_m[j].tolist()

                p.addUserDebugLine(
                    p1,
                    p2,
                    lineColorRGB=color_rgb,
                    lineWidth=4,
                    lifeTime=0,
                    replaceItemUniqueId=line_set[k]
                )
            else:
                p.addUserDebugLine(
                    [0, 0, 0],
                    [0, 0, 0],
                    lineColorRGB=color_rgb,
                    lineWidth=1,
                    lifeTime=0,
                    replaceItemUniqueId=line_set[k]
                )

    def clear_skeleton_slot(self, slot_idx):
        if not (0 <= slot_idx < len(self.skel_line_ids)):
            return

        for lid in self.skel_line_ids[slot_idx]:
            p.addUserDebugLine(
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
                1,
                0,
                replaceItemUniqueId=lid
            )

    def step(self):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)


class ZEDHumanSafetyNode(Node):
    def __init__(self, sim):
        super().__init__("zed_human_safety_node")

        self.sim = sim
        self._shutdown = False
        self.lock = threading.Lock()

        self.paused = False
        self.slow_mode = False

        # =========================
        # TCP speed recording
        # =========================
        self.speed_lock = threading.Lock()
        self.speed_records = []
        self.speed_start_monotonic = None
        self.speed_session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._tcp_speed_warned = False

        # =========================
        # Emergency-stop trigger latency recording
        # =========================
        # 主要評估：
        #   camera_read_to_pause_ms
        #   = 從呼叫 ZED grab() 開始，到送出 PAUSE request 前的時間
        #
        # 另外也紀錄：
        #   frame_acquired_to_pause_ms
        #   = grab() 成功回傳後，到送出 PAUSE request 前的時間
        #
        # 注意：這裡量測的是軟體端「觸發 PAUSE」延遲，
        # 不包含 TM 機械手臂實體完全停止所需的機械/通訊反應時間。
        self.latency_lock = threading.Lock()
        self.latency_records = []
        self.camera_frame_index = 0

        self.send_cli = self.create_client(SendScript, "/send_script")
        while not self.send_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("Waiting for /send_script...")

        self.event_cli = self.create_client(SetEvent, "/set_event")
        while not self.event_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("Waiting for /set_event...")

        self.create_subscription(
            FeedbackState,
            "/feedback_states",
            self.feedback_cb,
            10
        )

        self.tcp_points_fast = [
            'PTP("JPP",0,0,60,-10,90,0,100,100,100,false)',
            'PTP("JPP",0,0,120,-10,90,0,100,100,100,false)',
            'PTP("JPP",0,0,60,-10,90,0,100,100,100,false)',
            'PTP("JPP",0,0,120,-10,90,0,100,100,100,false)'
        ]
        self.tcp_points_slow = [
            'PTP("JPP",0,0,60,-10,90,0,40,100,100,false)',
            'PTP("JPP",0,0,120,-10,90,0,40,100,100,false)',
            'PTP("JPP",0,0,60,-10,90,0,40,100,100,false)',
            'PTP("JPP",0,0,120,-10,90,0,40,100,100,false)'
        ]

        self.tcp_points = self.tcp_points_fast

        self.zed = None
        self.runtime_params = None
        self.body_runtime_params = None
        self.bodies = None

        self._open_zed()

        sb = self.sim.get_slow_bounds()
        if sb:
            xm, xM, ym, yM, zm, zM = sb
            self.get_logger().info(
                f"固定減速盒：中心(0,0,0)，半徑={SLOW_BOX_HALF_EXTENT_M} m，"
                f"x∈[{xm:.2f},{xM:.2f}], y∈[{ym:.2f},{yM:.2f}], z∈[{zm:.2f},{zM:.2f}]"
            )

        threading.Thread(target=self.detect_loop, daemon=True).start()
        threading.Thread(target=self.run_loop, daemon=True).start()

    def _open_zed(self):
        self.zed = sl.Camera()

        init_params = sl.InitParameters()
        init_params.camera_resolution = ZED_RESOLUTION
        init_params.camera_fps = ZED_FPS
        init_params.depth_mode = ZED_DEPTH_MODE
        init_params.coordinate_units = ZED_COORD_UNIT
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD

        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise SystemExit(f"[FATAL] ZED open failed: {err}")

        self.get_logger().info("[ZED] Camera opened")

        positional_tracking_params = sl.PositionalTrackingParameters()

        try:
            positional_tracking_params.set_as_static = True
        except Exception:
            pass

        err = self.zed.enable_positional_tracking(positional_tracking_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.zed.close()
            raise SystemExit(f"[FATAL] enable_positional_tracking failed: {err}")

        body_params = sl.BodyTrackingParameters()
        body_params.enable_tracking = True
        body_params.enable_body_fitting = ZED_ENABLE_BODY_FITTING
        body_params.body_format = ZED_BODY_FORMAT

        try:
            body_params.detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_MEDIUM
        except Exception:
            pass

        err = self.zed.enable_body_tracking(body_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.zed.disable_positional_tracking()
            self.zed.close()
            raise SystemExit(f"[FATAL] enable_body_tracking failed: {err}")

        self.runtime_params = sl.RuntimeParameters()

        self.body_runtime_params = sl.BodyTrackingRuntimeParameters()
        try:
            self.body_runtime_params.detection_confidence_threshold = ZED_DETECTION_CONFIDENCE
        except Exception:
            pass

        self.bodies = sl.Bodies()

        self.get_logger().info("[ZED] Body Tracking enabled, format=BODY_34")

    def close_zed(self):
        if self.zed is not None:
            try:
                self.zed.disable_body_tracking()
            except Exception:
                pass

            try:
                self.zed.disable_positional_tracking()
            except Exception:
                pass

            try:
                self.zed.close()
            except Exception:
                pass

            self.zed = None

    def _apply_xy_square_mask(self, center_mm):
        if not USE_XY_SQUARE_MASK:
            return True

        x_m = center_mm[0] / 1000.0
        y_m = center_mm[1] / 1000.0
        z_m = center_mm[2] / 1000.0

        cx, cy = XY_MASK_CENTER_M
        half = float(XY_MASK_HALF_M)

        inside_xy = (
            cx - half <= x_m <= cx + half
            and cy - half <= y_m <= cy + half
        )

        if not inside_xy:
            return False

        if XY_MASK_Z_RANGE_M is not None:
            zmin, zmax = XY_MASK_Z_RANGE_M
            if not (zmin <= z_m <= zmax):
                return False

        return True

    def _estimate_body_center(self, X_full_mm):
        valid = np.isfinite(X_full_mm).all(axis=1)

        torso_candidates = [5, 12, 18, 22]
        pts = []

        for idx in torso_candidates:
            if idx < len(valid) and valid[idx]:
                pts.append(X_full_mm[idx])

        if len(pts) > 0:
            return np.mean(np.asarray(pts, dtype=np.float64), axis=0)

        if len(valid) > 0 and valid[0]:
            return X_full_mm[0].copy()

        if np.any(valid):
            return np.mean(X_full_mm[valid], axis=0)

        return None

    def _zed_body_to_human(self, body):
        if not hasattr(body, "keypoint"):
            return None

        kp = np.asarray(body.keypoint, dtype=np.float64)

        if kp.ndim != 2 or kp.shape[1] != 3:
            return None

        X_zed_mm = np.full((NUM_ZED_KPTS, 3), np.nan, dtype=np.float64)
        n = min(NUM_ZED_KPTS, kp.shape[0])
        X_zed_mm[:n] = kp[:n]

        valid = np.isfinite(X_zed_mm).all(axis=1)

        if hasattr(body, "keypoint_confidence"):
            try:
                conf = np.asarray(body.keypoint_confidence, dtype=np.float64)
                nc = min(NUM_ZED_KPTS, conf.shape[0])
                valid[:nc] &= conf[:nc] >= ZED_KEYPOINT_CONFIDENCE
            except Exception:
                pass

        X_zed_mm[~valid] = np.nan

        X_base_mm = transform_points_mm(T_ZED_TO_BASE_MM, X_zed_mm)

        good = np.isfinite(X_base_mm).all(axis=1)
        X_base_mm[good] += BIAS_MM

        center_mm = self._estimate_body_center(X_base_mm)

        if center_mm is None:
            if hasattr(body, "position"):
                pos_zed = np.asarray(body.position, dtype=np.float64).reshape(3)
                pos_base = transform_point_mm(T_ZED_TO_BASE_MM, pos_zed)
                if pos_base is not None:
                    center_mm = pos_base + BIAS_MM

        if center_mm is None or not np.isfinite(center_mm).all():
            return None

        body_id = getattr(body, "id", -1)

        return Human3D(
            body_id=body_id,
            center_mm=center_mm,
            X_full_mm=X_base_mm
        )

    def _record_emergency_latency(self, frame_timing, pause_trigger_perf, pause_trigger_wall):
        """紀錄由相機讀取到 PAUSE request 觸發的軟體延遲。"""
        if not frame_timing:
            return

        grab_start_perf = frame_timing.get("grab_start_perf")
        frame_acquired_perf = frame_timing.get("frame_acquired_perf")
        bodies_ready_perf = frame_timing.get("bodies_ready_perf")
        humans_ready_perf = frame_timing.get("humans_ready_perf")
        visualization_done_perf = frame_timing.get("visualization_done_perf")

        if grab_start_perf is None or frame_acquired_perf is None:
            return

        def elapsed_ms(t1, t0):
            if t1 is None or t0 is None:
                return float("nan")
            return float((t1 - t0) * 1000.0)

        camera_read_to_pause_ms = elapsed_ms(pause_trigger_perf, grab_start_perf)
        frame_acquired_to_pause_ms = elapsed_ms(pause_trigger_perf, frame_acquired_perf)

        record = {
            "event_index": 0,
            "frame_index": int(frame_timing.get("frame_index", -1)),
            "timestamp_unix_s": float(pause_trigger_wall),
            "timestamp_iso": datetime.fromtimestamp(pause_trigger_wall).isoformat(
                timespec="milliseconds"
            ),
            "camera_grab_ms": elapsed_ms(frame_acquired_perf, grab_start_perf),
            "body_tracking_ms": elapsed_ms(bodies_ready_perf, frame_acquired_perf),
            "human_processing_ms": elapsed_ms(humans_ready_perf, bodies_ready_perf),
            "visualization_ms": elapsed_ms(visualization_done_perf, humans_ready_perf),
            "post_visualization_to_pause_ms": elapsed_ms(
                pause_trigger_perf, visualization_done_perf
            ),
            "frame_acquired_to_pause_ms": frame_acquired_to_pause_ms,
            "camera_read_to_pause_ms": camera_read_to_pause_ms,
        }

        with self.latency_lock:
            record["event_index"] = len(self.latency_records)
            self.latency_records.append(record)

        self.get_logger().warning(
            "[LATENCY] emergency event=%d, frame=%d, "
            "camera read -> PAUSE = %.3f ms, "
            "frame acquired -> PAUSE = %.3f ms"
            % (
                record["event_index"],
                record["frame_index"],
                camera_read_to_pause_ms,
                frame_acquired_to_pause_ms,
            )
        )

    def _send_pause(self, frame_timing=None):
        if not self.event_cli.service_is_ready():
            self.get_logger().warning("SetEvent service not ready, Pause skipped")
            return

        req = SetEvent.Request()
        req.func = SetEvent.Request.PAUSE
        req.arg0 = 0
        req.arg1 = 0

        # 觸發時間定義：送出非同步 PAUSE service request 的當下
        pause_trigger_perf = time.perf_counter()
        pause_trigger_wall = time.time()

        self.event_cli.call_async(req)

        self._record_emergency_latency(
            frame_timing,
            pause_trigger_perf,
            pause_trigger_wall,
        )

        self.get_logger().warning("[SAFETY] Send Pause")

    def _send_resume(self):
        if not self.event_cli.service_is_ready():
            self.get_logger().warning("SetEvent service not ready, Resume skipped")
            return

        req = SetEvent.Request()
        req.func = SetEvent.Request.RESUME
        req.arg0 = 0
        req.arg1 = 0

        self.event_cli.call_async(req)
        self.get_logger().info("[SAFETY] Send Resume")

    def _update_safety_state(self, humans, frame_timing=None):
        now = time.time()

        any_in_green = False
        any_in_slow = False

        try:
            xm, xM, ym, yM, zm, zM = self.sim.current_bounds
        except Exception:
            xm = xM = ym = yM = zm = zM = None

        slow_bounds = self.sim.get_slow_bounds()

        if slow_bounds:
            sxm, sxM, sym, syM, szm, szM = slow_bounds
        else:
            sxm = sxM = sym = syM = szm = szM = None

        for human in humans:
            X_full_mm = human.X_full_mm

            if X_full_mm is None:
                continue

            pts3d_m = X_full_mm.astype(np.float64) / 1000.0
            valid = np.isfinite(pts3d_m).all(axis=1)
            valid_pts = pts3d_m[valid]

            if xm is not None and not any_in_green:
                for x, y, z in valid_pts:
                    if xm <= x <= xM and ym <= y <= yM and zm <= z <= zM:
                        any_in_green = True
                        break

            if slow_bounds and not any_in_slow:
                for x, y, z in valid_pts:
                    if sxm <= x <= sxM and sym <= y <= syM and szm <= z <= szM:
                        any_in_slow = True
                        break

        if any_in_green:
            self.sim.last_green_exit = None

            with self.lock:
                if not self.paused:
                    self.paused = True
                    self._send_pause(frame_timing)
                    self.sim.update_bounding_box_color(
                        color=[1, 0, 0],
                        line_width=BBOX_LINE_WIDTH
                    )
                    self.sim.show_overlay_from_current(
                        color=[1, 0, 0],
                        line_width=OVERLAY_LINE_WIDTH
                    )
        else:
            if self.sim.last_green_exit is None:
                self.sim.last_green_exit = now

            with self.lock:
                if self.paused and (
                    now - self.sim.last_green_exit >= RESUME_COOLDOWN_SEC
                ):
                    self.paused = False
                    self._send_resume()
                    self.sim.update_bounding_box_color(
                        color=[0, 1, 0],
                        line_width=BBOX_LINE_WIDTH
                    )
                    self.sim.hide_overlay()

        if any_in_slow:
            self.sim.last_slow_exit = None

            with self.lock:
                if not self.slow_mode:
                    self.slow_mode = True
                    self.tcp_points = self.tcp_points_slow
                    self.sim.update_slow_box_color(
                        color=SLOW_BOX_ALERT,
                        line_width=SLOW_BOX_LINE_WIDTH
                    )
                    self.get_logger().info("[SAFETY] Enter SLOW mode")
        else:
            with self.lock:
                if self.slow_mode:
                    if self.sim.last_slow_exit is None:
                        self.sim.last_slow_exit = now

                    if now - self.sim.last_slow_exit >= RESUME_COOLDOWN_SEC:
                        self.slow_mode = False
                        self.tcp_points = self.tcp_points_fast
                        self.sim.update_slow_box_color(
                            color=SLOW_BOX_COLOR,
                            line_width=SLOW_BOX_LINE_WIDTH
                        )
                        self.get_logger().info("[SAFETY] Back to FAST mode")
                        self.sim.last_slow_exit = None

    def feedback_cb(self, msg: FeedbackState):
        # 更新 PyBullet 中的機械手臂關節角度
        try:
            angles = list(msg.joint_angle[:6])
        except AttributeError:
            angles = list(msg.joint_pos[:6])

        self.sim.update_joint_angles(angles)

        # =====================================================
        # 記錄 TM Robot 回授的 TCP 速度
        #
        # msg.tcp_speed:
        # [Vx, Vy, Vz, Rx, Ry, Rz]
        #
        # Vx, Vy, Vz 單位：mm/s
        # Rx, Ry, Rz 單位：deg/s
        # =====================================================
        try:
            tcp_speed = np.asarray(msg.tcp_speed, dtype=np.float64).reshape(-1)
        except Exception:
            tcp_speed = np.array([], dtype=np.float64)

        if tcp_speed.size < 6:
            if not self._tcp_speed_warned:
                self.get_logger().warning(
                    "[TCP SPEED] msg.tcp_speed 資料不足。"
                    "請確認 TMflow Ethernet Slave Data Table 已啟用 "
                    "TCP_Speed 與 TCP_Speed3D。"
                )
                self._tcp_speed_warned = True
            return

        vx, vy, vz, rx, ry, rz = tcp_speed[:6]

        linear_speed = float(np.linalg.norm([vx, vy, vz]))
        angular_speed = float(np.linalg.norm([rx, ry, rz]))

        now_monotonic = time.perf_counter()
        now_wall = time.time()

        with self.speed_lock:
            if self.speed_start_monotonic is None:
                self.speed_start_monotonic = now_monotonic

            elapsed_s = now_monotonic - self.speed_start_monotonic

            self.speed_records.append({
                "sample_index": len(self.speed_records),
                "timestamp_unix_s": now_wall,
                "timestamp_iso": datetime.fromtimestamp(now_wall).isoformat(
                    timespec="milliseconds"
                ),
                "time_s": elapsed_s,
                "vx_mm_s": float(vx),
                "vy_mm_s": float(vy),
                "vz_mm_s": float(vz),
                "linear_speed_mm_s": linear_speed,
                "rx_deg_s": float(rx),
                "ry_deg_s": float(ry),
                "rz_deg_s": float(rz),
                "angular_speed_deg_s": angular_speed,
                "paused": int(bool(self.paused)),
                "slow_mode": int(bool(self.slow_mode)),
            })

    def save_tcp_speed_results(self):
        """將 TCP 速度紀錄儲存成 CSV，並輸出一張 V-T 圖。"""
        with self.speed_lock:
            records = list(self.speed_records)

        if not records:
            self.get_logger().warning(
                "[TCP SPEED] 沒有可儲存的 TCP 速度資料。"
            )
            return

        os.makedirs(OUTPUT_DIR, exist_ok=True)

        csv_path = os.path.join(
            OUTPUT_DIR,
            f"tcp_speed_{self.speed_session_id}.csv"
        )

        vt_path = os.path.join(
            OUTPUT_DIR,
            f"VT_{self.speed_session_id}.png"
        )

        fieldnames = [
            "sample_index",
            "timestamp_unix_s",
            "timestamp_iso",
            "time_s",
            "vx_mm_s",
            "vy_mm_s",
            "vz_mm_s",
            "linear_speed_mm_s",
            "rx_deg_s",
            "ry_deg_s",
            "rz_deg_s",
            "angular_speed_deg_s",
            "paused",
            "slow_mode",
        ]

        with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(records)

        time_values = np.asarray(
            [row["time_s"] for row in records],
            dtype=np.float64
        )

        speed_values = np.asarray(
            [row["linear_speed_mm_s"] for row in records],
            dtype=np.float64
        )

        valid = np.isfinite(time_values) & np.isfinite(speed_values)

        plt.figure(figsize=(10, 6))
        plt.plot(
            time_values[valid],
            speed_values[valid],
            linewidth=1.5,
            label="TCP linear speed"
        )
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (mm/s)")
        plt.title("TCP Velocity-Time (V-T)")
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.savefig(vt_path, dpi=300)
        plt.close()

        self.get_logger().info(
            f"[TCP SPEED] CSV saved: {csv_path}"
        )
        self.get_logger().info(
            f"[TCP SPEED] V-T plot saved: {vt_path}"
        )
        self.get_logger().info(
            f"[TCP SPEED] samples: {len(records)}"
        )

    def save_latency_results(self):
        """儲存緊停觸發延遲 CSV、延遲圖與統計結果。"""
        with self.latency_lock:
            records = list(self.latency_records)

        if not records:
            self.get_logger().warning(
                "[LATENCY] 沒有緊停觸發事件，因此沒有延遲資料可儲存。"
            )
            return

        os.makedirs(OUTPUT_DIR, exist_ok=True)

        csv_path = os.path.join(
            OUTPUT_DIR,
            f"emergency_latency_{self.speed_session_id}.csv"
        )

        plot_path = os.path.join(
            OUTPUT_DIR,
            f"Latency_{self.speed_session_id}.png"
        )

        fieldnames = [
            "event_index",
            "frame_index",
            "timestamp_unix_s",
            "timestamp_iso",
            "camera_grab_ms",
            "body_tracking_ms",
            "human_processing_ms",
            "visualization_ms",
            "post_visualization_to_pause_ms",
            "frame_acquired_to_pause_ms",
            "camera_read_to_pause_ms",
        ]

        with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(records)

        event_indices = np.asarray(
            [row["event_index"] for row in records],
            dtype=np.int64
        )

        total_latency_ms = np.asarray(
            [row["camera_read_to_pause_ms"] for row in records],
            dtype=np.float64
        )

        acquired_latency_ms = np.asarray(
            [row["frame_acquired_to_pause_ms"] for row in records],
            dtype=np.float64
        )

        valid_total = np.isfinite(total_latency_ms)
        valid_acquired = np.isfinite(acquired_latency_ms)

        plt.figure(figsize=(10, 6))

        if np.any(valid_total):
            plt.plot(
                event_indices[valid_total],
                total_latency_ms[valid_total],
                marker="o",
                linewidth=1.5,
                label="Camera read start to PAUSE"
            )

        if np.any(valid_acquired):
            plt.plot(
                event_indices[valid_acquired],
                acquired_latency_ms[valid_acquired],
                marker="s",
                linewidth=1.5,
                label="Frame acquired to PAUSE"
            )

        plt.xlabel("Emergency event index")
        plt.ylabel("Latency (ms)")
        plt.title("Emergency Stop Trigger Latency")
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.savefig(plot_path, dpi=300)
        plt.close()

        total_valid_values = total_latency_ms[valid_total]

        if total_valid_values.size > 0:
            mean_ms = float(np.mean(total_valid_values))
            median_ms = float(np.median(total_valid_values))
            std_ms = float(np.std(total_valid_values))
            min_ms = float(np.min(total_valid_values))
            max_ms = float(np.max(total_valid_values))

            self.get_logger().info(
                "[LATENCY] camera read -> PAUSE statistics: "
                f"n={total_valid_values.size}, "
                f"mean={mean_ms:.3f} ms, "
                f"median={median_ms:.3f} ms, "
                f"std={std_ms:.3f} ms, "
                f"min={min_ms:.3f} ms, "
                f"max={max_ms:.3f} ms"
            )

        self.get_logger().info(
            f"[LATENCY] CSV saved: {csv_path}"
        )
        self.get_logger().info(
            f"[LATENCY] plot saved: {plot_path}"
        )

    def send_ptp(self, script):
        if not self.send_cli.service_is_ready():
            self.get_logger().warning("/send_script not ready")

            if not self.send_cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("無法連接 /send_script，跳過本次 PTP")
                return

        req = SendScript.Request()

        if hasattr(req, "id"):
            req.id = "demo"

        req.script = script
        self.send_cli.call_async(req)

    def send_stop_and_clear(self):
        if not self.send_cli.service_is_ready():
            self.get_logger().warning("/send_script not ready, StopAndClearBuffer skipped")
            return

        req = SendScript.Request()

        if hasattr(req, "id"):
            req.id = "clear"

        req.script = "StopAndClearBuffer()"
        self.send_cli.call_async(req)
        self.get_logger().info("已送出 StopAndClearBuffer()")

    def run_loop(self):
        idx = 0

        while rclpy.ok() and not self._shutdown:

            while True:
                if self._shutdown:
                    break

                with self.lock:
                    if not self.paused:
                        break

                time.sleep(0.01)

            if self._shutdown:
                break

            with self.lock:
                if self.slow_mode:
                    self.send_stop_and_clear()
                    script = self.tcp_points_slow[idx]
                else:
                    script = self.tcp_points_fast[idx]

            self.send_ptp(script)

            t0 = time.time()

            while time.time() - t0 < 3.0 and not self._shutdown:
                with self.lock:
                    if self.paused:
                        break

                time.sleep(0.01)

            idx = (idx + 1) % len(self.tcp_points_fast)

    def detect_loop(self):
        last_log_t = 0.0

        while not self._shutdown:
            if self.zed is None:
                time.sleep(0.05)
                continue

            frame_index = self.camera_frame_index
            self.camera_frame_index += 1

            # 延遲計時起點：開始向 ZED 讀取下一幀
            grab_start_perf = time.perf_counter()

            grab_err = self.zed.grab(self.runtime_params)

            if grab_err != sl.ERROR_CODE.SUCCESS:
                time.sleep(0.005)
                continue

            # 影像幀已成功取得
            frame_acquired_perf = time.perf_counter()

            self.zed.retrieve_bodies(
                self.bodies,
                self.body_runtime_params
            )

            # ZED Body Tracking 結果已取得
            bodies_ready_perf = time.perf_counter()

            humans = []

            for body in self.bodies.body_list:
                human = self._zed_body_to_human(body)

                if human is None:
                    continue

                if not self._apply_xy_square_mask(human.center):
                    continue

                humans.append(human)

            humans = sorted(humans, key=lambda h: h.id)

            # 人體資料轉換與篩選完成
            humans_ready_perf = time.perf_counter()

            for slot_idx, human in enumerate(humans[:MAX_PERSONS]):
                X_full_mm = human.X_full_mm

                valid_mask = np.isfinite(X_full_mm).all(axis=1)

                if not np.any(valid_mask):
                    self.sim.clear_skeleton_slot(slot_idx)
                    continue

                pts3d_m = np.nan_to_num(X_full_mm, nan=0.0) / 1000.0

                draw_color = list(COLORS3D[slot_idx % len(COLORS3D)])

                self.sim.draw_skeleton_slot(
                    slot_idx,
                    pts3d_m,
                    valid_mask,
                    draw_color
                )

            for slot_idx in range(min(len(humans), MAX_PERSONS), MAX_PERSONS):
                self.sim.clear_skeleton_slot(slot_idx)

            # PyBullet 骨架更新完成
            visualization_done_perf = time.perf_counter()

            frame_timing = {
                "frame_index": frame_index,
                "grab_start_perf": grab_start_perf,
                "frame_acquired_perf": frame_acquired_perf,
                "bodies_ready_perf": bodies_ready_perf,
                "humans_ready_perf": humans_ready_perf,
                "visualization_done_perf": visualization_done_perf,
            }

            self._update_safety_state(humans, frame_timing)

            now = time.time()
            if now - last_log_t > 1.0:
                self.get_logger().info(f"[ZED] detected humans: {len(humans)}")
                last_log_t = now

        self.get_logger().info("detect_loop exit")


def main():
    rclpy.init()

    sim = TMSimulator()
    node = ZEDHumanSafetyNode(sim)

    print("=== 程式啟動：ZED Body Tracking + TM5 Safety 主迴圈開始 ===")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.001)
            sim.step()

    except KeyboardInterrupt:
        pass

    finally:
        node._shutdown = True
        time.sleep(0.2)

        # 儲存 TCP 速度 CSV 與 V-T 圖
        try:
            node.save_tcp_speed_results()
        except Exception as e:
            node.get_logger().error(
                f"[TCP SPEED] 儲存失敗：{e}"
            )

        # 儲存相機讀取到 PAUSE 觸發的延遲評估
        try:
            node.save_latency_results()
        except Exception as e:
            node.get_logger().error(
                f"[LATENCY] 儲存失敗：{e}"
            )

        node.close_zed()

        try:
            p.disconnect()
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
