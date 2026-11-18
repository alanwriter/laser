# F1／F2 前饋燒錄前的 flash 備份（2026-11-18）

使用者授權燒錄（「燒錄」），依先前的要求 F1/F2 馬達斷電、Nano USB 供電，沒有程序占用序列埠。
每台先以 avrdude（`-c arduino -p m328p -b 115200`；F2 用 `~/pi_rover/.candidate_2026_11_12/avrdude_f2_user_space.sh`）
讀回 flash，再寫入候選並驗證。車上同名資料夾 `~/pi_rover/backups/2026-11-18-pre-follower-feedforward/` 也有一份。

| 車 | 讀回 | 與舊映像比對 | 寫入 | 寫入後唯讀檢查 |
|---|---|---|---|---|
| F1 | `pi-follower1/f1_flash.hex` | 和 `firmware/builds/2026-11-12/f1_encoder_mpu_heading.hex` 的 29,290 個應用位元組完全一致 | `f1_velocity_feedforward_candidate.hex` 29,310 B 寫入並驗證 | HELLO F1；CONFIG 1203.5/1200.0/65/130/MPU 1.00；IMU present=1 位址 104；STATUS idle、fault 0；STOP ACK |
| F2 | `pi-follower2/f2_flash.hex` | 和 `f2_encoder_mpu_heading.hex` 的 29,342 個應用位元組完全一致 | `f2_velocity_feedforward_candidate.hex` 29,362 B 寫入並驗證 | HELLO F2；CONFIG 1216.2/1237.4/65/130/MPU 1.00；IMU present=1 位址 104；STATUS idle、fault 0；STOP ACK |

回退：燒回 `firmware/builds/2026-11-12/f1_encoder_mpu_heading.hex`／`f2_encoder_mpu_heading.hex`。
USB bootloader 讀不到真正的 EEPROM（見 `../2026-11-17-pre-velocity-feedforward/README.md`），所以沒有另讀；
韌體不使用 EEPROM。重開機後 `imu_calibrated=0`、`encoder_preflight=0` 是預期狀態，**尚未接地預檢或行走驗證**。
