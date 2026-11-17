# Leader 左右輪速度→PWM 前饋候選

目的：回應 2026-11-17 的控制方向修正：路徑控制仍只指定左右輪速度，Nano 依每輪目標 TPS 算 PWM，再以 encoder 回授修正。此檔原先是隔離建置候選；**在更新前完整 GitHub 備份核對後，已寫入 Leader 並通過靜態檢查，尚未完成行走驗證。** 更新前映像與結果見 [`backups/2026-11-17-pre-velocity-feedforward/README.md`](../../../backups/2026-11-17-pre-velocity-feedforward/README.md)。

## 來源與變更範圍

- 上游 `alanwriter/Arduino-ugv` 分支 `leader1-pi-usb`，基底提交 `c2fd857`；先套用本專案 [`encoder_distance_mpu_heading.patch`](../../encoder_distance_mpu_heading.patch) 與 [`leader_imu_startup_retry.patch`](../../leader_imu_startup_retry.patch)，保留現場已部署的純 MPU 航向與開機重試候選。
- 再套用 [`leader_velocity_feedforward_candidate.patch`](../../leader_velocity_feedforward_candidate.patch)。只改 `MOTION_VELOCITY`／`MOTION_PATH` 共用輪速控制器的**正轉前饋**；舊 PI 係數與積分限幅、encoder 換算、輪徑、STOP、命令到期及手動 `MOTOR` 模式均不改。倒車仍用原前饋，未作新校正。
- 舊正轉前饋為左右相同的 `55 + 0.14×target TPS`。候選改為左輪 `30 + 0.050×target TPS`、右輪 `35 + target TPS/30`；由 11/16 已起轉後 PWM 45–60 的短暫態輪速點反推線性關係，詳見 [`analysis/2026-11-16-motor-curves/README.md`](../../../analysis/2026-11-16-motor-curves/README.md)。這些點每段約 0.75 s，**不是穩態校正**。

| 目標 TPS | 舊共用前饋 PWM（尚未加 PI） | 候選左／右前饋 PWM（尚未加 PI） | 判讀 |
| ---: | ---: | ---: | --- |
| 150 | 76 | 37.5／40.0 | 低於已量 300 TPS；屬外插，可能不起轉 |
| 260 | 91.4 | 43.0／43.7 | 接近約 5 cm/s 地面速度的候選操作點，仍屬外插 |
| 300 | 97 | 45／45 | 已起轉短點附近 |
| 450 | 118 | 52.5／50 | 兩輪暫態內插 |
| 600 | 139 | 60／55 | 已起轉短點附近 |

表中 PWM 是**未加 Kp／Ki 的前饋值**，不是量到的最終 PWM，也不能把 `k=1.1483` 的直線距離比率直接當每輪地面速度校正。韌體仍用暫定 65 mm 輪徑換算 TPS。現有 18–26 mm/s 命令區約對應 107–156 TPS，正好缺穩態馬達資料；候選下限的起轉、停走與轉彎內輪表現都未知。使用者希望保留約 5 cm/s 的柔和地面速度，正式目標須與地面量測對齊，而不是把目前命令 25 mm/s 當成已校準的真實速度。

## 離線檢查結果

- 在隔離目錄套用上述三個 patch；新 patch 對已套用前兩個 patch 的基底通過 `git apply --check`。
- 用相同工具鏈重建「只套用前兩個 patch」的基底，其 HEX SHA-256 **完全等於** 11/12 已存檔的 Leader MPU 重試版 `a7e364fa9f70e925f4f92ce52b75930fb636207b3f4a8863e92721863dd15558`；故新候選與既有燒錄映像的差異可歸於本次前饋 patch。
- PlatformIO `nanoatmega328` 編譯成功：Flash **29,424／30,720 bytes（95.8%）**、RAM **874／2,048 bytes（42.7%）**；相對先前 Leader MPU 重試版 Flash 29,404 bytes，增加約 20 bytes。建置只在隔離目錄移除未用到的 Servo `lib_deps`，沒有修改上游設定。
- 候選 HEX：[`leader_velocity_feedforward_candidate.hex`](leader_velocity_feedforward_candidate.hex)，SHA-256 `8c1ff86f50d15b7b09d6621533cc3757f1ed684690c020f329172bda0cd86005`。
- 編譯和 `MOTOR 45/45` 測點取消時，此候選尚未燒錄。後續依使用者要求先將完整更新前備份推到 GitHub，才把候選寫入 Leader；flash 驗證與靜態 `HELLO`／`CONFIG`／IMU／STATUS 檢查已通過。尚未接地預檢或行走驗證。

## 實車門檻與回退

先保留 Leader 現有 flash／EEPROM 的新備份並核對 SHA，才考慮燒錄。若使用者授權候選上車，先在馬達斷電狀態完成燒錄及唯讀 `HELLO`／`CONFIG`／IMU 查詢；接地短暫預檢通過後重新擺回 MPU 起點，只做單車固定左右**目標速度相等**的短直線。需同步記錄同窗左右 count 斜率、每輪 target/measured TPS、實際 PWM、地面 MPU 位置與航向；任何 IMU、encoder、USB、超過航向／路程護欄都 STOP。若低速不起轉、停走或偏航，恢復備份韌體，不拿失敗試次擬合前饋。F1/F2 不套用 Leader 係數。

下一版正式前饋仍要用不同電池電量、相同載重與地面下的多次已起轉穩態點，並分開量冷起轉門檻。現有候選未加抗積分飽和或起轉狀態機，這兩項留待實測是否需要，而不在同一趟同時更動多個控制層。
