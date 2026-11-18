# 大左轉（套用 1.140 比例）：離線模擬與 11:01Z 實跑（2026-11-18）

## 路線設計（離線）

目標是讓地面上走出 11/12 候選大左轉的形狀，起點用操作者現有的地面標記：

| 項目 | 地面 | 送進 Pi（地面 ÷ 1.140） |
|---|---:|---:|
| Leader 直線／半徑／出彎 | 1500／1200／1000 mm | `--straight-mm 1315.8 --radius-mm 1052.6 --exit-mm 877.2` |
| F1／F2 落後、左右車道 | 400、±400 mm | `--offset-x-mm -350.9 --offset-y-mm ±350.9`，起點同值 |
| Leader 速度 | — | `--speed-mm-s 60` |

1.140 是同日 [三車直線斜率](../2026-11-18-follower-straight/README.md)。
[simulate_scaled.py](simulate_scaled.py) 用現行 `PathFormationTracker` 加理想差速模型，
得到 [scaled_turn.png](scaled_turn.png) 與 [summary.json](summary.json)：沒有安全停止；
地面最小 MPU 距離 Leader–F1／Leader–F2／F1–F2 為 514／566／799 mm，
車身外框間隙 263／288／638 mm（11/18 修正：原本把已是「度」的 Leader 航向又轉一次，外框畫錯，曾誤寫 222／277；MPU 距離與 Follower 命令不受影響）；F2 外圈會碰到 75 mm/s 輪速上限。
這個模型**沒有馬達動態**，也沒有反映三台韌體的速度特性不同。

**設計錯誤：** 這條路線地面上約需 3.9 m 的前進距離，實驗室直線只有約 2 m。
當時只列出需求、沒有問場地尺寸，Leader 撞牆。

## 實跑 2026-11-18 11:01:37Z（無效：撞牆，Follower 跑到 Leader 前面）

部署程式與 repo 雜湊相同（Leader `leader_formation.py` a430ef46…、`turn_path.py` 2f37b5dd…；
F1/F2 `follower_formation.py` 02d95a3f…、F2 `follower2_formation.py` 4afe8551…），
都加了 `--require-mpu-heading`；三台的預檢都用 PWM 60／600 ms。Leader 的 mDNS 名稱
解析失敗，改用 IP 192.168.1.166 連線。

| 車 | 接地預檢 count 左／右 | 航向 | 結果 |
|---|---|---:|---|
| Leader（11/17 逐輪前饋韌體） | 216／223 | +0.5° | 撞牆後由操作者要求停止；SIGINT → STOP ACK，94 s |
| F1（舊韌體） | 945／653 | −27.8° | 58 s 自己停止：`leader is too close`；STOP ACK |
| F2（舊韌體） | 961／1152 | +15.8° | 26 s 自己停止：`path target error is too large`；STOP ACK |

三台的程序都已退出。原始 CSV（SHA-256）：`leader.csv` 1b8ae22c…、`f1.csv` 668814ce…、
`f2.csv` 942187e1…；終端輸出 `*_console.txt`；圖 [real_run_110137Z/onboard_paths.png](real_run_110137Z/onboard_paths.png)
（車上里程估計，不是地面真值）。

### Follower 跑到 Leader 前面的原因

| 車 | 直線段平均命令 | 車上估計速度 |
|---|---:|---:|
| Leader | 60 mm/s | 58 mm/s（前 20 s） |
| F1 | 35.9 mm/s | 92 mm/s |
| F2 | 38.2 mm/s | 102 mm/s |

Follower 的 Pi 控制器有偵測到自己超前（進度誤差從 −131 mm 增加到 −721 mm），把命令壓到
下限 ≈ Leader 速度 60 − 最大修正 25 = 35 mm/s；但舊韌體的輪速迴路在低速會跑到目標
TPS 的 2.5–3 倍，所以 **Pi 拉不住 Follower**。以前三台都是舊韌體、一起超速，彼此剛好相配；
11/17 只有 Leader 換成逐輪前饋，命令和實際速度變成接近，兩邊的速度特性就不一致了。
提高 Leader 速度並不能解決：Follower 的命令是跟著 Leader 速度走的。

**結論：** F1/F2 的韌體速度特性和 Leader 一致之前，不再跑三車編隊；新路線必須先依實際場地
尺寸設計並畫給操作者確認。
