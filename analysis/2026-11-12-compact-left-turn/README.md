# 第一趟修正候選：Leader 終點前進 1 m、左移 1.2 m

**尚未實車行走。** 這份離線候選將「前進 1 m、往左 1.2 m」解讀為
Leader 的 MPU 終點位移，並保持原本 F1/F2 `(-400,+400)`／
`(-400,-400)` mm 起點及新版 `PathFormationTracker`。若操作者指的是
三台車含車身的共同邊界，這份配置不符合，須重新選車道與路線。

| 路線段 | 數值 |
|---|---:|
| 起始直線 | 200 mm |
| 左轉 | 90°，Leader 半徑 800 mm |
| 出口直線 | 400 mm |
| Leader 路程 | 約 1,856.6 mm |
| Leader MPU 終點 | `(1000,1200)` mm，航向 90° |
| F1／F2 車道 | 左右 ±400 mm，路徑進度落後 400 mm |
| 彎中 follower 半徑 | F1 400 mm／F2 1,200 mm |

由 90° 圓弧幾何可得 `x_end = 直線 + 半徑 = 200 + 800 = 1000`，
`y_end = 半徑 + 出口 = 800 + 400 = 1200`。Leader 巡航假設 30 mm/s，
理想路程約 61.9 秒。Leader 跟隨控制器以理想差速模型積分時約 64.6 秒
完成，終點約 `(999.9,1201.2)` mm、航向 90°。

[`compact_turn.png`](compact_turn.png) 畫出同時刻的 Leader、F1、F2
理想／模擬 MPU 軌跡及 250 × 150 mm 車身。使用實際候選
`TurnPath`、`PathFormationTracker`，假設 Leader 理想走線並以 0.1 秒
步長積分兩台理想差速車，結果在 [`summary.json`](summary.json)；
逐步資料在 [`simulated.csv`](simulated.csv)。兩台 follower 均未觸發
控制器停止。模擬中的最小車身間隙約 Leader–F1 247 mm、Leader–F2
287 mm、F1–F2 650 mm。三車車身在模型中涵蓋約
`x=-550..1479`、`y=-484..1300` mm；四側各加 300 mm 的名義繪圖範圍
約 2.63 × 2.38 m。**F2 車身會超過 Leader 終點的前方 1 m。**

這些值只來自理想模型，不包含馬達死區、輪速控制偏差、IMU 漂移、
encoder 換算誤差、網路延遲或實際起點偏差；不能證明地面淨空。
重跑離線模擬：

```bash
MPLCONFIGDIR=/tmp/mpl-laser .venv/bin/python \
  analysis/2026-11-12-compact-left-turn/simulate.py
```

三台 Pi 已有支援這組幾何的候選程式；此次修正前**只做了版本與 USB
唯讀檢查，沒有馬達預檢或行走**。實跑時仍須依
[`pi_rover/RUNBOOK.md`](../../pi_rover/RUNBOOK.md) 完成逐台預檢，確認
F1 右輪閉環跟隨、起點與淨空，再做唯一一次合併出發確認。

第一趟啟動時 Leader IMU 間歇性未偵測，已在校正／馬達預檢前 STOP；
三台程序均退出。見 [中止紀錄](preflight_abort_leader_imu.md)。

後續 Leader 開機 MPU 重試候選通過靜態重啟測試；第三次授權的實跑
已在操作者要求下中止。該趟原始 CSV、影片觀察及內部控制訊息分析見
[09:43Z 實跑紀錄](real_run_094329Z/README.md)。理想模擬不能代表這次實車結果。
