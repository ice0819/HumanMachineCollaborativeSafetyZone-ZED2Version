#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import os
import time
import numpy as np
import pyzed.sl as sl


# ====================================================================
# --- ZED 設定區 ---
# ====================================================================

BASE_DIR = os.path.dirname(os.path.abspath(__file__))


def rel_path(*parts):
    """取得以本程式所在資料夾為基準的路徑。"""
    return os.path.join(BASE_DIR, *parts)


SAVE_PATH_L = rel_path("zed_chessboard", "img_l")
SAVE_PATH_R = rel_path("zed_chessboard", "img_r")

# ZED 解析度
# 可選：
# sl.RESOLUTION.HD2K
# sl.RESOLUTION.HD1080
# sl.RESOLUTION.HD720
# sl.RESOLUTION.VGA
ZED_RESOLUTION = sl.RESOLUTION.HD1080
ZED_FPS = 30

# 棋盤格內角點數量，不是格子數
CHECKERBOARD_SIZE = (4, 3)

# 顯示畫面大小
DISPLAY_WIDTH = 960
DISPLAY_HEIGHT = 540


# ====================================================================
# --- 工具函式 ---
# ====================================================================

def zed_mat_to_bgr(zed_mat):
    """
    將 ZED sl.Mat 轉成 OpenCV BGR 影像。
    ZED get_data() 通常是 4 通道 BGRA/RGBA 格式，這裡統一轉成 BGR。
    """
    img = zed_mat.get_data()

    if img is None:
        return None

    img = img.copy()

    if img.ndim == 3 and img.shape[2] == 4:
        # ZED SDK 在 OpenCV 中通常可視為 BGRA
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    elif img.ndim == 3 and img.shape[2] == 3:
        img = img.copy()
    else:
        return None

    return img


def resize_frame(frame, width, height):
    return cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)


def get_next_image_index(save_path_l, save_path_r):
    """
    讀取左右資料夾中已存在的 left_*.png / right_*.png，
    找出最大編號，並從下一個編號繼續儲存。
    """
    max_idx = -1

    for folder, prefix in [(save_path_l, "left_"), (save_path_r, "right_")]:
        if not os.path.exists(folder):
            continue

        for filename in os.listdir(folder):
            if filename.startswith(prefix) and filename.endswith(".png"):
                try:
                    idx_str = filename.replace(prefix, "").replace(".png", "")
                    idx = int(idx_str)
                    max_idx = max(max_idx, idx)
                except ValueError:
                    pass

    return max_idx + 1


def draw_status(frame, title, img_counter, found, corners=None):
    display = frame.copy()

    instruction_text = "Press SPACE to capture, Q or ESC to quit"

    cv2.putText(
        display,
        title,
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 0),
        2
    )

    cv2.putText(
        display,
        instruction_text,
        (10, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2
    )

    cv2.putText(
        display,
        f"Next index: {img_counter}",
        (10, 90),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2
    )

    if found:
        cv2.drawChessboardCorners(display, CHECKERBOARD_SIZE, corners, found)
        cv2.putText(
            display,
            "Chessboard FOUND",
            (10, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )
    else:
        cv2.putText(
            display,
            "Chessboard NOT found",
            (10, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2
        )

    return display


# ====================================================================
# --- ZED 初始化 ---
# ====================================================================

def setup_zed():
    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.camera_resolution = ZED_RESOLUTION
    init_params.camera_fps = ZED_FPS

    # 拍棋盤格影像不需要深度，關掉可降低負載
    try:
        init_params.depth_mode = sl.DEPTH_MODE.NONE
    except Exception:
        pass

    err = zed.open(init_params)

    if err != sl.ERROR_CODE.SUCCESS:
        print(f"錯誤：無法開啟 ZED，相機初始化失敗：{err}")
        return None

    cam_info = zed.get_camera_information()
    print("ZED 已成功開啟")
    print(f"ZED Serial Number: {cam_info.serial_number}")
    print(f"ZED Camera Model : {cam_info.camera_model}")
    print(f"ZED SDK Version  : {sl.Camera().get_sdk_version()}")

    time.sleep(1.0)

    return zed


# ====================================================================
# --- 主程式 ---
# ====================================================================

def main():
    os.makedirs(SAVE_PATH_L, exist_ok=True)
    os.makedirs(SAVE_PATH_R, exist_ok=True)

    print(f"ZED 左影像將儲存至: {SAVE_PATH_L}")
    print(f"ZED 右影像將儲存至: {SAVE_PATH_R}")

    zed = setup_zed()
    if zed is None:
        return

    img_counter = get_next_image_index(SAVE_PATH_L, SAVE_PATH_R)
    print(f"目前將從編號 {img_counter} 開始接續儲存。")

    runtime_params = sl.RuntimeParameters()

    zed_left = sl.Mat()
    zed_right = sl.Mat()

    flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        + cv2.CALIB_CB_NORMALIZE_IMAGE
    )

    criteria = (
        cv2.TERM_CRITERIA_EPS
        + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001
    )

    try:
        while True:
            err = zed.grab(runtime_params)

            if err != sl.ERROR_CODE.SUCCESS:
                print(f"警告：ZED grab 失敗：{err}")
                time.sleep(0.01)
                continue

            zed.retrieve_image(zed_left, sl.VIEW.LEFT)
            zed.retrieve_image(zed_right, sl.VIEW.RIGHT)

            frame_l = zed_mat_to_bgr(zed_left)
            frame_r = zed_mat_to_bgr(zed_right)

            if frame_l is None or frame_r is None:
                print("錯誤：ZED 影像轉換失敗")
                continue

            gray_l = cv2.cvtColor(frame_l, cv2.COLOR_BGR2GRAY)
            gray_r = cv2.cvtColor(frame_r, cv2.COLOR_BGR2GRAY)

            ret_l_cal, corners_l = cv2.findChessboardCorners(
                gray_l,
                CHECKERBOARD_SIZE,
                flags
            )

            ret_r_cal, corners_r = cv2.findChessboardCorners(
                gray_r,
                CHECKERBOARD_SIZE,
                flags
            )

            if ret_l_cal:
                corners_l = cv2.cornerSubPix(
                    gray_l,
                    corners_l,
                    (11, 11),
                    (-1, -1),
                    criteria
                )

            if ret_r_cal:
                corners_r = cv2.cornerSubPix(
                    gray_r,
                    corners_r,
                    (11, 11),
                    (-1, -1),
                    criteria
                )

            display_l = draw_status(
                frame_l,
                "ZED LEFT",
                img_counter,
                ret_l_cal,
                corners_l if ret_l_cal else None
            )

            display_r = draw_status(
                frame_r,
                "ZED RIGHT",
                img_counter,
                ret_r_cal,
                corners_r if ret_r_cal else None
            )

            display_l = resize_frame(display_l, DISPLAY_WIDTH, DISPLAY_HEIGHT)
            display_r = resize_frame(display_r, DISPLAY_WIDTH, DISPLAY_HEIGHT)

            cv2.imshow("ZED Left Camera", display_l)
            cv2.imshow("ZED Right Camera", display_r)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q") or key == 27:
                print("程式結束。")
                break

            elif key == ord(" "):
                if ret_l_cal and ret_r_cal:
                    img_name_l = f"left_{img_counter}.png"
                    img_name_r = f"right_{img_counter}.png"

                    full_path_l = os.path.join(SAVE_PATH_L, img_name_l)
                    full_path_r = os.path.join(SAVE_PATH_R, img_name_r)

                    cv2.imwrite(full_path_l, frame_l)
                    cv2.imwrite(full_path_r, frame_r)

                    print(
                        f"成功儲存 [{img_counter}]：\n"
                        f"  Left : {full_path_l}\n"
                        f"  Right: {full_path_r}"
                    )

                    img_counter += 1

                else:
                    print("拍照失敗：左右 ZED 影像必須都偵測到棋盤格。")

    finally:
        zed.close()
        cv2.destroyAllWindows()
        print("ZED 已關閉。")


if __name__ == "__main__":
    main()
