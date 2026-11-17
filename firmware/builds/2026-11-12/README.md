# 2026-11-12 encoder 距離＋MPU 航向候選韌體

這三份是 2026-11-12 **已逐台燒錄並由 avrdude 驗證**的互動控制版 Nano `.hex`。共同的 `src/main.cpp` 取自
[`Arduino-ugv` 的 Pi USB 分支](https://github.com/alanwriter/Arduino-ugv)，
套用 [`encoder_distance_mpu_heading.patch`](../../encoder_distance_mpu_heading.patch)：
`IMU_GYRO_BLEND` 由 `0.70` 改為 `1.00`，`IO,CONFIG` 尾端增加第九欄
`heading_gyro_blend=1.00`。輪速 PID、左右 encoder 距離與安全逾時邏輯未改。

| 車 | `vehicle_profile.h` 來源分支 HEAD | 檔案 | SHA-256 |
|---|---|---|---|
| Leader | `leader1-pi-usb` `c2fd857` | `leader_encoder_mpu_heading.hex` | `912f0d1b097406c0dbdadfc6e7356ad8b0b5e2e3d3c553c6f79a21928193d1b1` |
| F1 | `follower1-pi-usb` `6448f53` | `f1_encoder_mpu_heading.hex` | `c84cc27954d525bf943db84c63810b7caea86f50c7230f0c53be66765c37d123` |
| F2 | `follower2-pi-usb` `6728798` | `f2_encoder_mpu_heading.hex` | `d7739adf7959ef7fc59a0711040ee7b7de428af4f3b22c458e051f0e30e598b9` |

本機以 PlatformIO `nanoatmega328` 編譯三次，三份均成功；Leader/F1/F2
Flash 用量分別約 95.6%／95.3%／95.5%。因本機無網路取得未用於
`src/main.cpp` 的 Servo 套件，**只在暫存建置目錄**移除 `platformio.ini` 的
`lib_deps = arduino-libraries/Servo` 後編譯；上游設定與原始檔沒有被改寫。

每台車原 flash／EEPROM 已先保存於 `backups/2026-11-12-pre-mpu-flash/`
及車上同名資料夾。燒錄後三台 `IO,CONFIG` 第九欄均讀回 `1.00`，
`HELLO` 車型分別為 L1／F1／F2、`STATUS` 均 idle。USB 重啟後 IMU 校正與
encoder 預檢尚未重做；真正行走前仍須依 RUNBOOK 預檢。

## Leader 開機 MPU 重試候選

`leader_encoder_mpu_heading_retry.hex` 已於 2026-11-12 **只燒錄 Leader**，
SHA-256 為 `a7e364fa9f70e925f4f92ce52b75930fb636207b3f4a8863e92721863dd15558`。
它在上述 Leader 原始碼及 `encoder_distance_mpu_heading.patch` 之後，
再套用 [`leader_imu_startup_retry.patch`](../../leader_imu_startup_retry.patch)。
只在 Nano 開機且馬達已停止的狀態下，把 MPU6050 偵測改為最多五次、
失敗後相隔 100 ms 再試；若全部失敗，仍回報 `imu_present=0`，
Pi 啟動檢查會拒絕行走。行駛中的 IMU 連續讀取失敗故障處理未改。

本機 PlatformIO 編譯成功，Flash 29404/30720 bytes（95.7%）、
RAM 874/2048 bytes（42.7%）。avrdude 寫入 29404 bytes 並驗證通過。
燒錄前 Leader flash／EEPROM 已保存於
`backups/2026-11-12-pre-leader-retry/pi-leader/`，Pi 上亦有同名備份。
燒錄後連續 12 次 USB 重啟、每次八次唯讀 IMU 查詢均為 `present=1`、
位址 0x68、連續讀取失敗 0；`HELLO` 為 L1，`CONFIG` 航向係數為 1.00。
這個補丁僅處理可能的開機時序，不能證明或修復接線、供電等硬體不穩。
