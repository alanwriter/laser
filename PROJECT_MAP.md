# 專案地圖（2026-11-12 盤點）

這個倉庫同時保存 Mac 雷射雲台、LiDAR 檢視工具與三車編隊的 Pi 程式。Nano 韌體另存於 [Arduino-ugv](https://github.com/alanwriter/Arduino-ugv)。**GitHub 分支、本機工作樹、Pi 上的程式與 Nano 已燒錄映像是四種不同狀態**；任何一種都不能自動代表另外三種。

| 要找的內容 | 入口 | 性質 |
|---|---|
| Mac 雷射雲台 | [README.md](README.md)、`vision_pointer.py` | 本倉庫首頁描述的原始專案 |
| Mac RPLIDAR S3 即時畫面 | `lidar_live_view.py`、`lidar_live_view.html`、`analysis/2026-11-09-lidar-s3/` | 獨立的掃描工具與紀錄；檔案是否已提交以 Git 狀態為準 |
| 三車 Pi 控制程式 | [pi_rover/RUNBOOK.md](pi_rover/RUNBOOK.md)、`pi_rover/*.py` | 操作入口；上車前須核對每台 Pi 的實際版本 |
| 三車候選跟隨器 | [pi_rover/NORMAL_FOLLOW_V0.1.0.md](pi_rover/NORMAL_FOLLOW_V0.1.0.md)、[pi_rover/EXPERIMENT_ENCODER_MPU_TURN.md](pi_rover/EXPERIMENT_ENCODER_MPU_TURN.md) | 三台 Pi 已部署 `0.2.0-dev+left-turn`；三台 Nano 已燒錄 100% gyro 候選版，尚未實跑 |
| 校正與畫圖約定 | [pi_rover/CALIBRATION_PLAN.md](pi_rover/CALIBRATION_PLAN.md)、[pi_rover/PLOT_CONVENTIONS.md](pi_rover/PLOT_CONVENTIONS.md) | 校正方案及 MPU 座標圖例 |
| 歷史實跑與離線分析 | `analysis/2026-10-13-three-rover/`、`analysis/2026-10-29-f1-f2-theory/`、`analysis/normal-follow-v0.1.0-simulation/` | 日期與實驗版本須一併閱讀；模擬不代表實車 |
| 直行左轉候選 | [analysis/2026-11-12-straight-left-turn/README.md](analysis/2026-11-12-straight-left-turn/README.md) | Pi 程式已部署，僅通過離線模擬，未實跑 |
| 歷史快照與映像 | `backups/` | 保存當時檔案，不當作目前 Pi／Nano 狀態 |
| Nano 韌體來源 | [analysis/2026-11-12-firmware-inventory.md](analysis/2026-11-12-firmware-inventory.md)、[Arduino-ugv](https://github.com/alanwriter/Arduino-ugv) | 需區分來源分支與實際燒錄版本 |

## GitHub 分支怎麼讀

- 本倉庫 `main` 是原有主線；`normal-follow/v0.1.0` 是三車路徑跟隨候選版。已有 `formation-telemetry-v1`、`normal-follow-v0.1.0` 標籤。**分支 HEAD 是來源狀態，不是部署紀錄。**
- `Arduino-ugv` 的 `leader1-pi-usb`、`follower1-pi-usb`、`follower2-pi-usb` 分別放三車 Pi USB 互動韌體設定；`leader1`、`follower1`、`follower2`、`*-demo` 仍保留歷史。詳見韌體盤點，不以 `main` 或分支名稱猜測已燒錄版本。
- 最近三車實跑紀錄在 `analysis/2026-10-29-f1-f2-theory/real_run_084014Z/`。那次 F1 舊、F2 新；影片與車上里程計的相對間距不一致，不能據此判定哪個控制器在地面較準。閱讀 `video_csv_findings.md`。

## 後續整理時的資料保留原則

先把每次實驗的原始 CSV、影片來源、程式提交、Pi 部署版本、Nano 韌體雜湊、模擬假設與結論連在同一份紀錄，再考慮調整目錄。修改檔案位置時用可追溯的 Git 提交；保留原始資料及歷史分支／標籤，不覆寫當時快照。本地尚未提交的檔案需先逐項確認用途和大小，不能把它們視為 GitHub 已備份。
