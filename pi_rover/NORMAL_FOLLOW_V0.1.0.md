# 常態跟隨 `v0.1.0`（候選版）

Git 分支 `normal-follow/v0.1.0`，標籤 `normal-follow-v0.1.0`。
這個版本把 F1 的預設控制器改成 F2 已試過的路徑跟隨器；F2 保持右側路徑。
**目前只在本地版本庫，F1 Pi 仍是上一趟 A/B 的舊版，未做雙新控制器實車驗證。**

## 這版控制的定義

Leader 的 `wave` 封包帶 `eased_sine_v1` 路徑幾何和量測進度 `s_L`。
F1/F2 各自用 `s*=s_L−400 mm`，車道分別是起始 Leader 座標系的
`+400/-400 mm` 平移路徑；目標航向取**自身車道在自身投影進度處的切線**。
速度用沿路徑進度差修正，轉向用橫向差、切線航向差和曲率前饋修正。
彎道受最大轉速與輪速限制會減速；遇到舊版／無效路徑封包就保持停止。
無線封包不傳馬達 PWM；Nano 仍執行輪速迴路和命令逾時停止。

程式入口仍是 `follower_formation.py`（F1）和 `follower2_formation.py`（F2）；
兩者的 `--version` 均回報 `normal-follow 0.1.0`。舊 `FormationTracker` 保留
供前次 A/B 資料重播，**不再是 F1 預設的實車控制器**。

## 證據與限制

- 2026-10-29 第二趟實車是**舊 F1／新 F2**：車上估計各自車道橫向誤差
  中位數約 71／13 mm；影片未做地面尺度校正。見
  `analysis/2026-10-29-f1-f2-theory/real_run_074722Z/video_csv_findings.md`。
- 雙新控制器理想差速模擬：既有 1500 mm、±50 mm、三週期、45 mm/s
  路線下，F1/F2 橫向誤差中位數均約 3.7 mm，但因彎道轉速限制，
  沿路徑進度誤差中位數約 135 mm。它**不能維持全程精準 400 mm 間距**。
- 同一理想模型改為 1500 mm、±20 mm、兩週期、25 mm/s，橫向誤差
  中位數約 0.07 mm、進度誤差中位數約 0.18 mm；這僅供選擇首趟
  校正後驗證路線，沒有納入輪胎打滑、Nano 輪速偏差與 IMU 漂移。
- 第二趟 CSV 顯示輪速回報高於目標 TPS，且 Leader 里程計平均 x 進度
  高於命令速度。先按 `CALIBRATION_PLAN.md` 分辨真實速度、編碼器換算及
  輪速迴路，再部署和測量雙新控制器。
- 此版本只接受 `eased_sine_v1` 路徑封包，不代表可直接追任意規劃器輸出的路徑。

離線重跑雙新模擬（不開 USB、不發 UDP）：

```bash
MPLBACKEND=Agg .venv/bin/python pi_rover/simulate_formation.py --mode normal
MPLBACKEND=Agg .venv/bin/python pi_rover/simulate_formation.py --mode normal \
  --amplitude-mm 20 --cycles 2 --speed-mm-s 25 \
  --out-dir analysis/normal-follow-v0.1.0-simulation/gentle_25
```

此版本沒有修改 Nano 韌體，也沒有自動部署到任何 Pi。
