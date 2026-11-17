# 三車候選路線：直走 1.5 m、左轉、再直走 1 m

這是 **2026-11-12 的離線候選**，沒有部署到 Pi，也沒有啟動任何車。圖見
[route_candidate.png](route_candidate.png)，可用
`.venv/bin/python analysis/2026-11-12-straight-left-turn/plot_candidate.py` 重畫。

## 路線與隊形

| 項目 | 候選值 |
|---|---:|
| Leader 第一段直線 | 1,500 mm |
| 左轉 | 90°，圓弧半徑 1,200 mm |
| Leader 第二段直線 | 1,000 mm |
| Leader 總路程 | 約 4,385 mm |
| Leader 終點 MPU | `(2700, 2200)` mm，航向左 90° |
| F1／F2 起點 MPU | `(-400,+600)`／`(-400,-600)` mm |
| F1／F2 目標 | 沿各自車道落後 Leader 路徑進度 400 mm；左右車道偏移隨路徑切線旋轉 |
| 假設巡航速度 | Leader 30 mm/s，理想行走時間約 146 秒 |

圖上的 F1／F2 以**同一時刻**的 Leader 路徑進度畫出：Leader 進彎時，兩台
仍要再走約 400 mm 才進彎。Leader 到終點時，兩台目標還落後 400 mm，
各在第二段直線行進約 600 mm 的位置。左轉內側是 F1，外側是 F2；
理想圓弧半徑分別為 600／1,800 mm。如果 Leader 在彎中維持 30 mm/s，
兩台理想線速度約為 15／45 mm/s，還需確認實車在低速下能平順行走。

圖中 250 × 150 mm 虛線車身以 MPU6050 點為錨，前緣 +100 mm、後緣 −150 mm、
左右各 75 mm。理想幾何的最小 MPU 點距離為 Leader–F1 約 663 mm、
Leader–F2 約 721 mm、F1–F2 1,200 mm。包含車身與四周各約 300 mm 餘量的
**名義繪圖範圍**約 4.53 × 3.58 m；這不是經現場測量確認的安全區。
前次實跑影片與里程計推算的相對間距曾不一致，不能用此圖證明實際淨空。

## 實車執行前的阻礙與目前進度

- 本地候選程式已新增 Leader `turn`／`left_turn_v1`，F1/F2 的新版
  `PathFormationTracker` 會重建隨切線旋轉的左右車道，並按內外彎半徑調整目標速度。
  S 彎仍保留 `eased_sine_v1`；2026-11-12 已部署到三台 Pi，但尚未實跑。
- 純 MPU 航向的 Nano 修改為 `firmware/encoder_distance_mpu_heading.patch`；
  尚未編譯、燒錄並從三台車讀回 `CONFIG` 核對。程式的
  `--require-mpu-heading` 可在啟動前拒絕 70/30 韌體。
- F1 右輪曾明顯偏慢，尚無修復或接地短行驗證紀錄。三台 Nano 已在
  備份原映像後燒錄純 MPU 航向候選韌體，尚未實跑。F2 Pi 用暫存的使用者層
  `avrdude` 完成燒錄，未安裝系統套件。

因此此候選目前只通過程式與離線驗證；不可直接照圖送現有 `WAVE-3` 指令。
實跑前仍須核對起點、路徑淨空、F1 輪速及三台程式版本，依 RUNBOOK 完成預檢。

## 理想 Leader 下的 follower 控制模擬

[simulated_followers.png](simulated_followers.png) 的**虛線**是幾何目標，
**實線**是控制器逐步算左右輪速度、再以理想差速車運動學積分出的 MPU 軌跡。
模擬檔案為 [simulate_followers.py](simulate_followers.py)、
[simulated_followers.csv](simulated_followers.csv) 與
[simulation_summary.json](simulation_summary.json)。模擬完全離線，未開 USB、
未發 UDP，也未修改或部署實車控制程式。

假設 Leader 沿規劃線以 30 mm/s 理想行走，F1/F2 從 `(-400,±600)` mm
開始；每 0.1 秒以現有 `PathFormationTracker` 的速度、轉向、輪速限制與
停止條件計算命令。模擬器使用 `pi_rover/turn_path.py` 與候選
`PathFormationTracker`，再以 `step_rover` 理想差速積分。Leader 約 146.2 秒到終點，之後
保留 20 秒的 follower 收斂觀察。

| 離線結果 | F1 內側 | F2 外側 |
|---|---:|---:|
| Leader 行走期間目標距離誤差中位數 | 0.3 mm | 0.7 mm |
| Leader 行走期間最大目標距離誤差 | 2.4 mm | 2.5 mm |
| Leader 停下後 20 秒目標距離誤差 | 3.8 mm | 5.4 mm |
| 控制器停止旗標 | 無 | 無 |

這次模擬中，Leader–F1／Leader–F2／F1–F2 的最小 MPU 點距離約為
640／721／1200 mm。這些數值不含馬達死區、輪速 PID 誤差、輪胎打滑、
IMU 與里程計漂移、UDP 延遲、實際地面起點偏差或相機量測；**不是碰撞安全證明**。
F1 Pi 雖已更新候選控制器，這張圖仍不是目前 Pi／Nano 組合的實測。

目前同一套跟隨架構依 `kind` 選擇幾何：`eased_sine_v1` 維持固定起始座標偏移，
`left_turn_v1` 提供隨切線旋轉的法向偏移、投影、曲率與內外側速度比例。
舊實驗資料和舊控制器可照原版本重播；此擴充已部署到 Pi，尚未實跑。
