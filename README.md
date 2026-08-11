# ZED 2 人體追蹤與 TM5 安全控制系統

## 1. 專案簡介

本專案使用 ZED 2 的三維人體骨架追蹤，將人體位置轉換到 TM5-900 機械手臂的 Base 座標系，並在 PyBullet 中同步顯示人體骨架、機械手臂與安全區域。

系統會依人體所在位置控制機械手臂：

- 人體進入固定減速區時，切換成較低速度的 PTP 指令。
- 人體進入機械手臂動態危險區時，透過 ROS 2 送出 `PAUSE`。
- 人體離開危險區並超過冷卻時間後，送出 `RESUME`。
- 程式結束時，輸出 TCP 速度紀錄與 PAUSE 軟體觸發延遲。

目前唯一的主要入口是：

```bash
python main.py
```

> **安全警告：** 本程式屬研究與測試用途，不是經安全認證的急停系統。第一次執行前必須清空工作區、降低機械手臂速度、確認實體急停可立即操作，並由操作人員全程監看。不可用本程式取代安全 PLC、安全雷射掃描器或其他合規安全設備。

## 2. 執行流程

`main.py` 的主要流程如下：

1. 從 `zed_chessboard/img_l` 與 `zed_chessboard/img_r` 尋找同編號棋盤格照片。
2. 開啟 ZED，從 SDK 取得雙目相機參數。
3. 三角化棋盤格角點，結合 Base→棋盤格矩陣計算 ZED→Robot Base 轉換。
4. 在 PyBullet 載入 TM5-900、地板、固定減速區與動態危險區。
5. 等待 TM Driver 提供 `/send_script` 與 `/set_event` 服務。
6. 啟用 ZED BODY_34 人體追蹤，將三維骨架轉換至 Robot Base 座標。
7. 訂閱 `/feedback_states`，同步機械手臂關節角度並記錄 TCP 速度。
8. 依人體關節點是否進入安全區域，切換快速、慢速、PAUSE 與 RESUME。
9. 按 `Ctrl+C` 結束時，將量測結果寫入 `output/`。

## 3. 目前目錄與檔案功用

```text
專案目錄/
├── main.py
├── capture.py
├── tcp_to_base.py
├── requirements.txt
├── README.md
├── images.png
├── zed_chessboard/
│   ├── img_l/
│   │   └── left_N.png
│   └── img_r/
│       └── right_N.png
└── tm_description/
    ├── package.xml
    ├── CMakeLists.txt
    ├── urdf/
    ├── xacro/
    └── meshes/
```

| 檔案／資料夾 | 功用 |
|---|---|
| `main.py` | 主程式。負責棋盤格校正、ZED BODY_34 追蹤、座標轉換、PyBullet 模擬、TM ROS 2 控制、安全狀態判斷及量測輸出。 |
| `capture.py` | 擷取 ZED 左右棋盤格照片。只有左右畫面都辨識到棋盤格時才會儲存。 |
| `tcp_to_base.py` | 根據 TCP 位姿與 TCP→棋盤格位姿，計算 Base→棋盤格齊次轉換矩陣並繪製座標軸。 |
| `requirements.txt` | 可由 pip 安裝的 Python 相依套件。 |
| `images.png` | PyBullet 地板貼圖；缺少時仍可執行，但不會套用貼圖。 |
| `zed_chessboard/img_l` | 左鏡頭校正照片。 |
| `zed_chessboard/img_r` | 右鏡頭校正照片；數字編號必須與左圖一致。 |
| `tm_description/urdf` | TM 機械手臂 URDF；主程式使用 `tm5-900.urdf`。 |
| `tm_description/xacro` | 各 TM 機型的 Xacro 描述。 |
| `tm_description/meshes` | URDF 顯示及碰撞模型使用的 OBJ、MTL、STL。 |
| `tm_description/package.xml` | ROS 2 `tm_description` 套件資訊。 |
| `tm_description/CMakeLists.txt` | `ament_cmake` 建置及資源安裝規則。 |
| `output` | 執行後自動建立，存放 CSV 與 PNG 圖表。 |

## 4. 路徑規則

所有專案資源都以 Python 程式所在資料夾為基準，不綁定 `/home/...` 等絕對路徑。因此整個專案搬移後，不需要修改校正照片、URDF、貼圖或輸出位置。

主要相對路徑如下：

```python
CALIB_ROOT = rel_path("zed_chessboard")
FLOOR_TEXTURE_PATH = rel_path("images.png")
TM5_URDF_PATH = rel_path("tm_description", "urdf", "tm5-900.urdf")
OUTPUT_DIR = rel_path("output")
```

`capture.py` 也會直接將照片存到：

```text
zed_chessboard/img_l
zed_chessboard/img_r
```

## 5. 環境需求

### 5.1 建議硬體與作業系統

- Ubuntu 22.04 64-bit。
- ROS 2 Humble。
- Python 3.10。
- NVIDIA GPU、相容的 NVIDIA 驅動及 CUDA。
- ZED 2 或相容的 ZED 系列相機，連接 USB 3.x。
- TM5-900 機械手臂，且電腦與機械手臂位於同一網段。
- 可顯示桌面視窗的環境；PyBullet 使用 GUI。

實際版本仍須同時符合 ZED SDK、CUDA、ROS 2 與 TM ROS 2 Driver 的相容性。

### 5.2 不能只靠 pip 安裝的元件

- Stereolabs ZED SDK 與其 Python API `pyzed.sl`。
- ROS 2 與 `rclpy`。
- Techman Robot ROS 2 Driver。
- TM Driver 提供的 `tm_msgs` 訊息及服務。

### 5.3 Python 套件

建議建立可讀取系統套件的虛擬環境，因為 `rclpy` 與 `pyzed.sl` 通常安裝在系統 Python：

```bash
cd <專案目錄>
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

驗證主要模組：

```bash
python -c "import cv2, numpy, matplotlib, pybullet, pyzed.sl, rclpy, tm_msgs; print('環境檢查完成')"
```

若找不到 `rclpy` 或 `tm_msgs`，請先載入 ROS 2 與 TM Driver 工作區：

```bash
source /opt/ros/humble/setup.bash
source <ROS2_WORKSPACE>/install/setup.bash
```

## 6. ROS 2 與 tm_description 建置

將 TM ROS 2 Driver 與本專案的 `tm_description` 放入 ROS 2 工作區的 `src` 後建置：

```bash
cd <ROS2_WORKSPACE>
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

確認介面存在：

```bash
ros2 interface show tm_msgs/msg/FeedbackState
ros2 interface show tm_msgs/srv/SendScript
ros2 interface show tm_msgs/srv/SetEvent
```

## 7. 棋盤格校正準備

### 7.1 目前校正設定

| 參數 | 目前值 | 說明 |
|---|---:|---|
| `CALIB_ZED_RESOLUTION` | HD1080 | 拍攝校正照片時使用的解析度。 |
| `CALIB_ZED_FPS` | 30 | 校正相機幀率。 |
| `CHECKERBOARD_SIZE` | `(4, 3)` | 棋盤格內角點數量，不是格子數。 |
| `SQUARE_SIZE_MM` | 71.0 mm | 棋盤格單格實際邊長。 |
| `CALIB_PAIR_ID` | 0 | 優先使用 `left_0.png` 與 `right_0.png`。 |
| `USE_RECTIFIED_ZED_IMAGES` | `True` | 擷取的左右影像視為 ZED 校正後影像。 |

若棋盤格規格不同，必須先修改 `CHECKERBOARD_SIZE` 與 `SQUARE_SIZE_MM`。

### 7.2 拍攝左右校正照片

確保主程式、擷取程式與 `zed_chessboard` 維持第 3 節的相對位置，然後執行：

```bash
cd <專案目錄>
source .venv/bin/activate
python capture.py
```

操作方式：

- 左右畫面都顯示 `Chessboard FOUND` 時，按空白鍵儲存照片。
- 按 `Q` 或 `Esc` 結束。
- 檔名會自動使用下一個編號，例如 `left_7.png` 與 `right_7.png`。
- 左右圖必須是同一次擷取且使用相同編號。
- 棋盤格需完整、清晰、無反光。
- 相機、棋盤格或 Robot Base 的相對位置改變後，必須重新校正。

若要指定其他照片，修改 `main.py` 的 `CALIB_PAIR_ID`。設為 `None` 時，程式會依 `CALIB_VALID_IMAGE_INDEX` 選取有效配對。

### 7.3 設定 Base→棋盤格矩陣

`main.py` 的 `T_BASE_CHESS_OLD_MM` 表示：

```text
Robot Base ← Chessboard
```

平移單位為 mm。主程式會自動計算：

```python
T_BASE_CHESS_MM = T_BASE_CHESS_OLD_MM @ Ry(180°)
```

因此不要在輸入矩陣中重複套用相同的 180° 修正。

可先修改 `tcp_to_base.py` 內的 `tcp_pose` 與 `tcp_chess_pose`，再執行：

```bash
python tcp_to_base.py
```

程式會印出轉換矩陣、旋轉矩陣行列式，並顯示 Base、TCP、棋盤格座標軸。

## 8. 常用系統與 ROS 2 指令

### 8.1 每個新終端機先載入環境

```bash
source /opt/ros/humble/setup.bash
source <ROS2_WORKSPACE>/install/setup.bash
cd <專案目錄>
source .venv/bin/activate
```

### 8.2 檢查影像裝置

```bash
v4l2-ctl --list-devices
```

若找不到 `v4l2-ctl`：

```bash
sudo apt update
sudo apt install v4l-utils
```

### 8.3 檢查序列與 USB 裝置

```bash
ls -l /dev/serial/by-id/
lsusb
dmesg | tail -n 50
```

`/dev/serial/by-id/` 不存在通常表示目前沒有具固定 ID 的序列裝置。

### 8.4 啟動 TM Driver

本專案曾使用兩個不同網段的 Robot IP，請依現場設定擇一執行：

```bash
# Robot IP 為 192.168.250.30
ros2 run tm_driver tm_driver robot_ip:=192.168.250.30

# Robot IP 為 192.168.10.3
ros2 run tm_driver tm_driver robot_ip:=192.168.10.3
```

不要同時啟動兩個 Driver。啟動前可先確認網路：

```bash
ping -c 4 192.168.250.30
# 或
ping -c 4 192.168.10.3
```

### 8.5 檢查 ROS 2 狀態

```bash
# 列出 node
ros2 node list

# 確認主程式需要的服務
ros2 service list | grep -E '/send_script|/set_event'

# 讀取一筆機械手臂回授
ros2 topic echo /feedback_states --once

# 查看回授頻率
ros2 topic hz /feedback_states

# 查看訊息與服務定義
ros2 interface show tm_msgs/msg/FeedbackState
ros2 interface show tm_msgs/srv/SendScript
ros2 interface show tm_msgs/srv/SetEvent
```

## 9. 執行前安全檢查

1. 確認實體急停功能正常且操作人員可立即觸及。
2. 清空機械手臂運動範圍，第一次測試不可有人站在工作區內。
3. 確認電腦與機械手臂位於同一網段。
4. 確認 TMflow 已進入 TM Driver 所需的 Listen Node／外部控制狀態。
5. 檢查 `tcp_points_fast` 與 `tcp_points_slow` 中的 PTP 關節角度及速度，確保路徑不會碰撞。
6. 在 TMflow Ethernet Slave Data Table 啟用 TCP Speed 與 TCP Speed3D。
7. 確認 `left_0.png`、`right_0.png` 與目前 ZED 是同一套校正資料。
8. 確認 ZED、棋盤格及機械手臂固定位置沒有改變。
9. 先讓機械手臂保持靜止，確認 PyBullet 中的機械手臂、人體骨架與實際座標方向一致。

## 10. 正式啟動

### 終端機 A：啟動 TM Driver

```bash
source /opt/ros/humble/setup.bash
source <ROS2_WORKSPACE>/install/setup.bash
ros2 run tm_driver tm_driver robot_ip:=<TM_ROBOT_IP>
```

啟動後確認：

```bash
ros2 service list | grep -E '/send_script|/set_event'
ros2 topic echo /feedback_states --once
```

### 終端機 B：啟動主程式

```bash
source /opt/ros/humble/setup.bash
source <ROS2_WORKSPACE>/install/setup.bash
cd <專案目錄>
source .venv/bin/activate
python main.py
```

啟動時應依序看到：

1. `T_BASE_CHESS_OLD_MM` 與修正後矩陣。
2. 左右校正照片配對。
3. ZED 雙目內外參及棋盤格三維擬合誤差。
4. `T_ZED_TO_BASE_MM`。
5. PyBullet GUI 中的 TM5-900、安全盒及人體骨架。
6. `[ZED] Camera opened` 與 BODY_34 啟用訊息。
7. 每秒一次的 `[ZED] detected humans: N`。

> 主程式取得服務後會啟動 PTP 運動執行緒，不是只有顯示或模擬。啟動前務必完成第 9 節檢查。

## 11. 安全區與重要參數

| 參數 | 目前值 | 功用 |
|---|---:|---|
| `ZED_RESOLUTION` | HD720 | 人體追蹤解析度。 |
| `ZED_FPS` | 30 | 人體追蹤幀率。 |
| `ZED_DEPTH_MODE` | NEURAL | 深度模式。 |
| `ZED_BODY_FORMAT` | BODY_34 | 34 點人體骨架。 |
| `MAX_PERSONS` | 10 | 最多顯示人數。 |
| `ZED_DETECTION_CONFIDENCE` | 40 | 人體偵測信心門檻。 |
| `ZED_KEYPOINT_CONFIDENCE` | 20 | 關節點信心門檻。 |
| `XY_MASK_HALF_M` | 2.5 m | 只處理中心 `(0, 0)` 周圍 ±2.5 m。 |
| `BIAS_XY_M` | `(0.0, 0.3)` m | 座標轉換後的 XY 平移補償。 |
| `SLOW_BOX_HALF_EXTENT_M` | 2.0 m | 固定減速盒半邊長。 |
| `AABB_MARGIN_M` | 0.05 m | 機械手臂 AABB 額外邊界。 |
| `AABB_SCALE` | 1.5 | 動態危險盒放大倍率。 |
| `EMA_ALPHA` | 0.25 | 動態危險盒平滑係數。 |
| `RESUME_COOLDOWN_SEC` | 0.5 s | 離開區域後恢復前的等待時間。 |

PyBullet 顯示狀態：

- 藍色固定盒：減速區。
- 橘色固定盒：人體已進入減速區。
- 綠色動態盒：機械手臂危險區，目前無人體進入。
- 紅色動態盒：人體進入危險區，已送出 PAUSE。

人體任一有效關節點進入盒內即視為進入該區域。

## 12. 正常停止與輸出

在主程式終端按：

```text
Ctrl+C
```

程式會停止背景執行緒、儲存量測資料、關閉 ZED、斷開 PyBullet 並關閉 ROS 2 node。若程式無反應或機械手臂行為異常，應先按下**實體急停**，不可只依賴 `Ctrl+C`。

有收到相應資料時，`output/` 會包含：

| 輸出檔案 | 內容 |
|---|---|
| `tcp_speed_YYYYMMDD_HHMMSS.csv` | TCP 三軸線速度、三軸角速度、合成速度、PAUSE 與慢速狀態。 |
| `VT_YYYYMMDD_HHMMSS.png` | TCP 合成線速度對時間圖。 |
| `emergency_latency_YYYYMMDD_HHMMSS.csv` | 每次 PAUSE 事件各軟體處理階段的延遲。 |
| `Latency_YYYYMMDD_HHMMSS.png` | 相機讀取起點及取得影像後，到送出 PAUSE request 的延遲圖。 |

延遲資料只量測到送出非同步 PAUSE service request 的時刻，不包含 ROS 2 網路傳輸、TM 控制器處理與機械手臂實際完全停止時間。

## 13. 常見問題

### 找不到 `zed_chessboard` 或校正照片

確認 `zed_chessboard` 與 `main.py` 在同一層，左右資料夾名稱為 `img_l`、`img_r`，且左右照片具有相同編號。預設需要 `left_0.png` 與 `right_0.png`。

### 找不到棋盤格角點

確認程式設定為 4×3 個內角點、單格 71 mm。棋盤格需完整入鏡、清楚、低反光，左右照片也必須是同一次擷取。

### `ZED open failed`

關閉其他 ZED 工具與 `capture.py`，確認 USB 3.x、相機權限、GPU 驅動、CUDA 與 ZED SDK。相同相機不可同時被兩個行程占用。

### 找不到 `pyzed.sl`

`pyzed.sl` 應由已安裝的 ZED SDK 提供，Python 版本必須相容。不要以來源不明的同名 PyPI 套件代替。

### 一直顯示 `Waiting for /send_script` 或 `/set_event`

確認 TM Driver 已啟動、每個終端都已 source ROS 2 與工作區、`ROS_DOMAIN_ID` 一致，且 TMflow 已進入外部控制狀態。

### `/feedback_states` 沒有資料

檢查 Robot IP、網路介面、TMflow 狀態及 Driver 終端錯誤。可用 `ping`、`ros2 node list`、`ros2 topic list` 逐步確認。

### 沒有 TCP 速度輸出

若終端顯示 `msg.tcp_speed 資料不足`，請在 TMflow Ethernet Slave Data Table 啟用 TCP Speed 與 TCP Speed3D。沒有速度樣本時，不會產生速度 CSV。

### PyBullet 無法載入 URDF 或 mesh

確認 `tm_description` 完整存在並已經 colcon 建置及 source。`main.py` 以相對路徑讀取 URDF，而 URDF 內的網格使用 `package://tm_description/...`。

### 人體骨架方向相反或位置偏移

依序檢查：

1. 左右照片是否為同一組。
2. 棋盤格內角點與單格尺寸是否正確。
3. `T_BASE_CHESS_OLD_MM` 是否符合現場 Base 與棋盤格關係。
4. 是否重複套用 180° 座標修正。
5. `BIAS_XY_M` 是否仍適用。

座標不正確時應停止實機測試，不要只靠放大安全盒補償校正錯誤。
