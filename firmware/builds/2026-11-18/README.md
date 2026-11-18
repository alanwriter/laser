# F1／F2 逐輪速度→PWM 前饋候選（2026-11-18）

**狀態：2026-11-18 已燒錄 F1、F2 並驗證（備份見 `backups/2026-11-18-pre-follower-feedforward/`），尚未行走驗證。** 目的是讓 F1/F2 的「命令→實際輪速」和 Leader（11/17 前饋版）一致。
11/18 大左轉時，F1/F2 用舊韌體在 36–38 mm/s 命令下跑到 92–102 mm/s，跑到 Leader 前面
（見 `analysis/2026-11-18-left-turn-scaled/README.md`）。

## 現況核對（2026-11-18，avrdude 只做驗證）

| 車 | Nano 上的韌體 | 驗證 |
|---|---|---|
| Leader | `firmware/builds/2026-11-17/leader_velocity_feedforward_candidate.hex`（`8c1ff86f…`） | 29,424 bytes 一致 |
| F1 | `firmware/builds/2026-11-12/f1_encoder_mpu_heading.hex`（`c84cc279…`） | 29,290 bytes 一致 |
| F2 | `firmware/builds/2026-11-12/f2_encoder_mpu_heading.hex`（`d7739adf…`） | 29,342 bytes 一致 |

## 來源

- `alanwriter/Arduino-ugv` 的 `follower1-pi-usb` `6448f53`、`follower2-pi-usb` `6728798`；三個 `*-pi-usb`
  分支的 `src/main.cpp` 完全相同（SHA-256 前綴 `e70699f9e9fe`），差別只在 `vehicle_profile.h`
  （ticks／rev：F1 1203.5／1200.0、F2 1216.2／1237.4；輪徑 65、輪距 130 mm 皆暫定）。
  GitHub 的 profile 沒有任何馬達 PWM／速度參數。
- 先套用 `firmware/encoder_distance_mpu_heading.patch`（純 MPU 航向）。只在暫存建置目錄刪掉未使用的
  Servo `lib_deps`。重建結果和 11/12 已燒錄映像**逐位元組相同**（F1 `c84cc279…`、F2 `d7739adf…`），
  所以新舊版的差異只來自下面的前饋 patch。
- 再套用 `firmware/f1_velocity_feedforward_candidate.patch`／`f2_…`：和 Leader 的
  `leader_velocity_feedforward_candidate.patch` 結構相同，只有係數和註解不同。F1/F2 不套用
  Leader 的 IMU 開機重試 patch（只有 Leader 有那個問題）。

## 新舊版差在哪

只改 `MOTION_VELOCITY`／`MOTION_PATH` 輪速控制器的**正轉前饋**；PI（Kp 0.10、Ki 0.08、積分 ±220）、
encoder 換算、輪徑、STOP、命令到期、手動 `MOTOR`、倒車前饋都不變。

| | 舊版（三台共用） | F1 候選 左／右 | F2 候選 左／右 |
|---|---|---|---|
| 前饋公式 | `55 + 0.14 × TPS` | `22.4 + 0.0126 × TPS`／`29.9 + 0.0143 × TPS` | `18.0 + 0.020 × TPS`／`20.7 + 0.0147 × TPS` |
| TPS 150 | 76 | 24.3／32.0 | 21.0／22.9 |
| TPS 300 | 97 | 26.2／34.2 | 24.0／25.1 |
| TPS 400 | 111 | 27.4／35.6 | 26.0／26.6 |

PI 積分最多只能修正約 ±17.6 PWM（220 × 0.08）。舊前饋在低速至少給 76，所以 F1/F2 的 PWM
降不到 30 以下，輪速停在目標的 2.5–3 倍。

## 係數怎麼來的

用 11/18 F1/F2 的直線（1 m、2 m）和大左轉 CSV：只取行走中、啟動 3 s 後、每 1 s 內目標 TPS
變化 < 25% 的區段，每輪以 **count 斜率**（不用有 100 TPS 量化階梯的瞬時 TPS）配同窗平均 PWM；
TPS ≥ 200 的點分 100 TPS 區間取中位數，依點數加權線性擬合。

| 輪 | 區間中位（TPS → PWM，點數） | 採用 |
|---|---|---|
| F1 左 | 256→24.8 (13)、391→28.9 (14)、414→27.6 (22)、525→28.3 (15) | 擬合 |
| F1 右 | 298→34.1 (5)、321→35.0 (18)、407→35.1 (27)、530→37.9 (16) | 擬合 |
| F2 左 | 456→27.4 (29)、571→30.5 (13)、602→37.1 (3) | 擬合為 9.6 + 0.0387 × TPS，被稀疏的 602 點拉陡，TPS 150 只給 15.5，可能起轉不了；**改用保守的 18.0 + 0.020 × TPS**，仍貼合兩個密集區間 |
| F2 右 | 459→27.5 (29)、568→29.1 (14) | 擬合 |

**限制：** 這是閉環平衡點，不是開環掃描；TPS 250 以下沒有資料（舊韌體慢不下來），那一段是外插。
只有一顆電池狀態、一種地面。所以這是**候選**，燒錄後須先單車短直線驗證命令≈實際。

## 建置結果

| 檔案 | Flash／RAM | SHA-256 |
|---|---|---|
| `f1_velocity_feedforward_candidate.hex` | 29,310 B (95.4%)／884 B | `01f415348218996c297882f7fd7c31e3fa96df344a306f78f1c60142a5ca684b` |
| `f2_velocity_feedforward_candidate.hex` | 29,362 B (95.6%)／874 B | `3f1c851dd58493381c5b3e304798067742bfdef6163fa95458cfd7cf7356b94e` |

## 燒錄與回退

1. 馬達斷電、Nano USB 供電、沒有程序占用序列埠。
2. 先用 avrdude 讀回目前的 flash 存進 `backups/2026-11-18-pre-follower-feedforward/`（USB bootloader 讀不到真正的
   EEPROM，見 11/17 紀錄；韌體不使用 EEPROM）。
3. 寫入並驗證，再做唯讀 `HELLO`／`CONFIG`／`IMU`／`STATUS`。
4. 單車接地短直線：比較同窗左右 count 斜率、目標／實測 TPS、PWM、地面位移與航向。
5. 若不起轉、停走或偏航，燒回 `firmware/builds/2026-11-12/f*_encoder_mpu_heading.hex`。
