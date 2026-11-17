# 三車 Nano 韌體來源盤點（2026-11-12）

此表是 GitHub 可找到的**最新分支來源**，不是已讀回三台 Nano 的燒錄映像。
來源倉庫：[alanwriter/Arduino-ugv](https://github.com/alanwriter/Arduino-ugv)。

| 車 | 最新 Pi USB 分支 | 分支 HEAD／提交日 UTC | 本地備份 |
|---|---|---|---|
| Leader | [`leader1-pi-usb`](https://github.com/alanwriter/Arduino-ugv/tree/leader1-pi-usb) | `c2fd857`／2026-10-13 | `backups/2026-10-13/pi-leader/leader_velocity_7c59b36.hex` |
| F1 | [`follower1-pi-usb`](https://github.com/alanwriter/Arduino-ugv/tree/follower1-pi-usb) | `6448f53`／2026-10-01 | `backups/2026-10-13/pi-follower1/follower1_usb_velocity_30e4dba.hex` |
| F2 | [`follower2-pi-usb`](https://github.com/alanwriter/Arduino-ugv/tree/follower2-pi-usb) | `6728798`／2026-10-13 | 未找到 `.hex` 備份 |

三個分支的 `src/main.cpp` 內容相同（GitHub blob `ffe46d9...`），
`src/vehicle_profile.h` 保存各車不同的 ticks／圈、方向與核准路徑。
Leader 的 `7c59b36 → c2fd857` 只改 VS Code、協定文件與 Pi 範例程式；
F1 的 `30e4dba → 6448f53` 只改協定文件與 Pi 範例程式，因此本地兩個
`.hex` 名稱所指提交與分支 HEAD 的 Nano 核心來源一致。尚未驗證這些映像
是否仍燒錄在實車，也沒有從實車讀回韌體雜湊。

## 通訊與啟動模式

- [USB 協定](https://github.com/alanwriter/Arduino-ugv/blob/leader1-pi-usb/docs/leader_usb_io_protocol.md)：Pi ↔ Nano 經 Nano USB serial，115200 baud，`IO,<seq>,...`；支援 `HELLO`、`STATUS`、`CONFIG`、`IMU`、`ENCODER`、`STOP`、`RESET`、`CALIBRATE`、`MOTOR`、`VELOCITY`、`PATH`、`TELEMETRY`。韌體同時保留人類手動用的單字母診斷指令，兩者仍走同一條 USB 序列連線。
- Pi ↔ Pi 的 Leader 狀態廣播是 `laser` 專案 Python 程式的 UDP JSON，**不是** Nano 上的第二個無線協定；目前 `src/main.cpp` 未找到 nRF24、Wi-Fi、藍牙通訊實作。MPU6050 的 I²C 是車內感測器連線。
- [PlatformIO 設定](https://github.com/alanwriter/Arduino-ugv/blob/leader1-pi-usb/platformio.ini)的 `nanoatmega328` 是互動控制版，開機先停，須收到明確命令才行走。`nanoatmega328_demo_g1`／`nanoatmega328_demo_square` 才是上電自動跑 G1／G2 的獨立編譯目標。早期 `follower1-demo`／`follower2-demo` 分支也是自跑預設路徑的歷史版本。
- `PATH`／`G1/G2` 雖在最新互動版仍可用，需**明確指令**啟動；Pi 的路徑與編隊實驗通常以 `VELOCITY` 送短效輪速命令。

## 航向融合版本

最新三個 Pi USB 分支的共享程式仍是 `IMU_GYRO_BLEND = 0.70`：IMU 校正後
每步航向為 70% gyro-Z 積分、30% 輪距差／`WHEEL_TRACK_MM`。該設定
不是純 MPU。

[F2 舊校正紀錄](https://github.com/alanwriter/Arduino-ugv/blob/follower2/docs/follower2_calibration.md)
記載 2026-09-07 一次**未提交**的 100% gyro-Z 航向方形試驗，目視較方正；
因此可確認做過試驗，但不能由 GitHub 推斷車上目前仍燒的是該版。
當時兩輪編碼器仍用於輪速 PID、前進距離與安全監看。兩種航向融合方式
都可以保留作同地面、同速度、同起點的比較；方形目視結果不代表長路線
的 gyro 漂移已測量。
