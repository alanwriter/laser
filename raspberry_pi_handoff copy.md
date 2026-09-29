# Raspberry Pi 軟體專案交接紀錄

> 此 Arduino 專案只保留 Nano 硬體控制與接線資料。Raspberry Pi 的 Python、
> systemd 服務與遠端控制程式應在另一個專案維護。

## 目前狀態（2026-09-04）

- 已完成 Raspberry Pi OS Lite 的 SD 卡燒錄並登入本機 console。
- Pi 使用者：`alan`。
- 主機名稱：`raspberrypi`。
- Pi 已連上 Wi-Fi；當時 DHCP 位址為 `192.168.1.199`。此位址可能在重新開機後改變，
  建議日後在路由器設定 DHCP reservation。
- Mac 嘗試 `ssh alan@192.168.1.199` 時得到 `Connection refused`，表示網路正常，
  但 SSH 服務尚未啟動。
- **不要在文件、程式或 Git 中保存 Pi 密碼。**

## 下一步：啟用 SSH

在 Pi 的本機 console 輸入：

```bash
sudo systemctl enable --now ssh
sudo systemctl status ssh --no-pager
```

確認顯示 `active (running)` 後，在 Mac 或 Windows PowerShell 連線：

```bash
ssh alan@192.168.1.199
```

若 IP 已改變，在 Pi 輸入 `hostname -I` 查詢目前位址。

SSH 可用後，應從 Mac 建立 SSH key，取代每次輸入密碼：

```bash
ssh-keygen -t ed25519
ssh-copy-id alan@192.168.1.199
```

## 車體分工與接線原則

```text
Raspberry Pi
  路徑、遠端操作、相機／視覺、資料紀錄、開機自動任務
       │ USB Serial（115200 baud）
Arduino Nano
  encoder 中斷、輪速 PID、MPU6050、里程計、馬達安全停止、L298N
```

- Pi 應以 **USB 線接 Nano 的 USB 埠**，不要以杜邦線直接接 Nano D0/D1。
- Nano 保留所有即時控制與停止保護；Pi 的 Linux 不適合執行高速輪速 PID。
- Pi 用獨立、穩定的 5V 電源；不可由 Nano 或 L298N 的 5V 腳供電。
- Pi、Nano、L298N 與馬達電源必須共地。

## 開機自動執行

未來用 `systemd` 服務讓 Pi 開機後以 `alan` 使用者執行 Python 車控程式；
**不需要**啟用本機 console auto-login。

第一版自動程式的安全順序：

1. 等待 Nano USB serial 出現（使用 `/dev/serial/by-id/`，不要硬編 `/dev/ttyUSB0`）。
2. 先傳送 `S`，確保馬達停止。
3. 查詢 `I`、`D`、`P`，確認 MPU6050、encoder 與控制狀態。
4. 等待明確的使用者／實體解鎖訊號後，才傳送移動命令。

不要讓 Pi 一開機就自動送出 `F`、`B` 或 `G1/G2/G3`。

## Nano 韌體 USB Serial I/O 規範

### 連線

- 實體連線：Pi USB host → Nano USB port。
- Serial：`115200` baud、`8-N-1`。
- 以 ASCII 文字傳送命令；每個命令以 `\n`（建議）或 `\r` 結尾。
- 韌體也接受未附換行的命令，但會等待約 80 ms 才執行；Pi 應一律附 `\n`。
- 每次只允許 **一個**程式開啟 Nano 的 serial port；不要同時開 PlatformIO Monitor、
  Python 程式或其他 serial monitor。
- 開啟 USB serial 有可能使 Nano reset。Pi 必須先等待開機訊息，再送 `S`；Nano 開機時
  馬達輸出預設為停止。
- Linux 請優先使用穩定裝置路徑 `/dev/serial/by-id/...`，不要硬編
  `/dev/ttyUSB0`，因為重新插拔後編號可能改變。

### Pi 可傳送的命令

| 命令 | Nano 行為 | Pi 使用注意事項 |
| --- | --- | --- |
| `S` | 立刻停止兩顆馬達並清除 fault | Pi 連線後第一個命令；任何異常時立即傳送。 |
| `F` | 以 PWM 80 邏輯前進 | 最多 1.2 秒後自動停止。 |
| `B` | 以 PWM 80 邏輯後退 | 最多 1.2 秒後自動停止。 |
| `M<L>,<R>` | 指定左右馬達 PWM，例如 `M80,80` 或 `M-80,80` | 範圍 -165 至 165；同樣有 1.2 秒手動命令 timeout。 |
| `R` | 停止、清除 encoder count、重設相對座標與 heading | 每次新的行程／測試前建議使用。 |
| `C` | 車體靜止時校正 MPU6050 gyro Z bias | 執行約 1.5 秒；期間不可移動車子。完成後才可使用路徑命令。 |
| `G1` | 執行 500 mm 直線 | 需要 MPU 已校正、encoder preflight 已通過。 |
| `G2` | 執行 400 mm × 400 mm 方形路徑 | 同上；第一次實車測試不要直接使用。 |
| `G3` | 執行 350 mm L 型路徑 | 同上。 |
| `P` | 回傳姿態、輪速、PWM、fault、encoder preflight 狀態 | Pi 主要狀態查詢命令。 |
| `D` | 回傳 encoder 腳位與 AB 診斷 | 檢查 A/B phase 是否都有 edge、`invalid` 是否接近 0。 |
| `I` | 回傳 MPU6050 是否存在、校正狀態、roll/pitch、三軸 gyro 與 I2C 錯誤計數 | `present=yes`、`calibrated=yes` 才代表可跑路徑。 |
| `T` | 開啟／關閉 5 Hz 姿態監看 | 輸出 `ATT ...` 非同步行；車體進入移動模式時會自動關閉。 |
| `K` | 回傳 ticks/rev、輪徑、輪距設定 | 讀取幾何校正值。 |
| `?` 或 `H` | 回傳命令說明 | 可用來確認連線與韌體介面。 |

### Nano 回覆格式

回覆是 UTF-8 相容的 ASCII 文字行，並非 JSON。Pi 應以「逐行」讀取並解析，且能接受
不相關的訊息行（例如 fault、開機訊息、`T` 的 telemetry）。不要假設一個命令只會得到一行。

常用回覆範例：

```text
mode=idle,pose_mm=(0.0,0.0),heading_deg=0.0,L[count=...,tps=...,target=...,pwm=...],R[...],fault=none,encoder_preflight=passed
MPU6050 present=yes,address=0x68,calibrated=yes,roll_deg=...,pitch_deg=...,gyro_x_dps_raw=...,gyro_y_dps_raw=...,gyro_z_dps=...
pins LA=...,LB=...,RA=...,RB=...
encoder_preflight=passed
ATT roll=...,pitch=...,yaw_rel=...,accel_g=(...),gyro_dps=(...)
FAULT: ... . Motors stopped.
```

### Pi 端安全流程

每次 Pi 程式啟動或 serial 重新連線時，遵守下列順序：

1. 開啟 `/dev/serial/by-id/...`，等待 Nano 的開機訊息約 2 秒。
2. 傳送 `S\n`，讀取並記錄回覆。
3. 傳送 `I\n`、`D\n`、`P\n`，確認：`present=yes`、`calibrated=yes`、
   `encoder_preflight=passed`、`fault=none`。
4. 若尚未校正，Pi 只能提示使用者將車子靜止；由使用者確認後才傳 `C\n`。
5. 沒有使用者／實體解鎖前，Pi 不可傳送任何 `F`、`B`、`M...`、`G...` 命令。
6. 收到 `FAULT:`、I2C error、serial disconnect，或程式結束時，立即嘗試傳送 `S\n`，
   並將狀態切為未解鎖。

### 目前功能邊界

- Nano 本地執行輪速 PI 控制、encoder 里程計，以及 encoder 與 MPU6050 的航向融合。
- MPU6050 的車體水平轉彎軸已確認為 **Z 軸**；目前 `GYRO_Z_REVERSED=false`。
- `yaw_rel` 是短時間相對角度，無磁力計，因此不可視為長時間的絕對方位。
- `roll`、`pitch` 與 X/Y gyro 目前只用於診斷，不得作為 Pi 的平衡控制輸入。
