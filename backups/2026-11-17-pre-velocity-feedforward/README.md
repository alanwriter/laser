# 2026-11-17 Leader 輪速前饋更新前備份

使用者確認 Leader 僅馬達電源斷開，Pi 與 Nano USB 仍供電。此備份在更新 Nano 前，以 `avrdude` 讀取現有 flash／EEPROM，並從 Pi 複製目前四份控制／校正腳本。讀取時未有控制程序占用序列埠；Pi 上也保留一份於 `~/pi_rover/backups/2026-11-17-pre-velocity-feedforward/`。

| 檔案 | SHA-256 |
| --- | --- |
| `pi-leader/leader_flash.hex` | `c73d7b53bfc9f21c804ac977fa3aa096c0d5731c8bf89ba81abde3bf0727282a` |
| `pi-leader/leader_eeprom.hex`（見下方限制） | `1e5a80a60c5561280ea7643c9d7a53bf8290db1780be1df90d51c7915ddb1d68` |
| `pi-leader/leader_formation.py` | `a430ef46e4772d109df50ce904aada70075cc3200c317f2a7e89672f6320151e` |
| `pi-leader/leader_straight_calibration.py` | `018e70a33a854e6ecffa1af6ed8adf043276ed57c276c097664a0bd97e204649` |
| `pi-leader/leader_sine_calibration.py` | `38538ddfab1c90bea1bcbb5d34b944930c5548979efb57e62fcc63edee4ad49e` |
| `pi-leader/motor_curve_point.py` | `01aac5ebb2c85ffa7690dda0803c36f5861341c7ae7ee26cf4a37f9518ab1b44` |

`avrdude` 的完整讀取紀錄在 `pi-leader/read.log`。現有 flash 的 29,404 個應用程式位元組與 `firmware/builds/2026-11-12/leader_encoder_mpu_heading_retry.hex` 逐位元組一致；讀回檔另含整個 32 KiB flash 範圍。四份 Pi 腳本的雜湊也與本機 `pi_rover/` 同名檔一致。此提交 `cd29590` 與標籤 `pre-leader-velocity-feedforward-2026-11-17` 已在更新前推到公開的 `alanwriter/laser`。

## EEPROM 讀回限制（更新後發現）

USB bootloader 的 `avrdude -U eeprom:r:` 回傳了**flash 前 1024 bytes**，而非可信的 EEPROM 內容：更新前 `leader_eeprom.hex` 的全部 1024 bytes 都等於同次 `leader_flash.hex` 開頭；更新後 `leader_eeprom_after.hex` 的 8 個變動位址全都等於新韌體相同位址。這些檔案保留為原始讀取紀錄，**不可拿來還原 EEPROM 或宣稱 EEPROM 校正資料未變**。候選韌體原始碼沒有 EEPROM 讀寫；`CONFIG` 的輪速換算及幾何參數由編譯設定提供，並已在更新後由 Nano 查詢核對。若未來真要備份實際 EEPROM，需用支援 EEPROM 讀取的 ISP 接線／工具另做。

## 更新結果

新韌體的 `avrdude` 寫入與 flash 驗證均成功，紀錄在 `pi-leader/write.log`。隨後 Nano `HELLO` 回報 `L1`；`CONFIG` 為左右每圈 1216.4／1222.4 ticks、輪徑 65.0 mm、輪距 130.0 mm、MPU 比重 1.00；`IMU` 顯示 present=1、位址 104；最後 `STATUS` 為 idle、左右 PWM=0、fault=0，並收到 STOP ACK。重新開機後 `imu_calibrated=0`、`encoder_preflight=0` 屬預期，**尚未接地預檢或行走驗證**。
