# 三車編隊操作手冊

本手冊含 `normal-follow/v0.1.0` 分支的候選程式。**目前 F1 Pi 仍是上一趟 A/B 的
舊控制器；本地 Git 版本不代表已部署。** 實際上車前先完成下述地面校正、部署
F1 的新檔案並核對版本。每台 Nano 的 USB port
只能由其本車 Pi 上的一個 Python 程式開啟；不要同時執行 `status`、手動測試
或第二個編隊程式。

> 操作者選擇「三台車在地面依序預檢、預檢後放回起點、只做一次最終出發確認」。
> 以下原有手動流程與程式 `MOTOR` 提示仍寫「架空兩輪」；快速流程的車輪實際
> **接觸地面且會短暫移動**，不得假稱架空。該工作約定見根目錄 `AGENTS.md`。

## 候選常態編隊配置（部署後）

- Leader CLI 預設仍為 1.5 m、三週期、±50 mm、45 mm/s；這個急彎配置在
  雙路徑跟隨的理想模擬中有約 135 mm 中位進度落差。完成地面校正後的
  首趟雙新驗證建議明確指定 `--amplitude-mm 20 --cycles 2 --speed-mm-s 25`。
- F1/F2 起點分別為 `(-400,+400)`／`(-400,-400)` mm。兩台追蹤 Leader
  規劃路徑向左／右平移 400 mm 的路線，進度沿路徑落後 400 mm，航向取各自路線切線。
- Leader 對 F1/F2 的 UDP 直接單播：`192.168.1.199:5005` 與
  `192.168.1.175:5005`。
- Leader `wave` 封包附上 `eased_sine_v1` 路徑參數、起始座標及量測進度。
  兩台 follower 沒收到有效路徑都會保持停止。
- Leader 抵達終點後立即廣播 `stopped`；Follower 停止，不做預設的終點追趕。

若換了網路，先在 Leader 上執行 `hostname -I`，把得到的 Leader IPv4 改寫到
兩個 Follower 指令的 `--leader-host`；同時把兩台 Follower 的新 IPv4 改寫到
Leader 的兩個 `--broadcast`。三台 Pi 只需在同一個本地網路，不需要 Internet。

## 啟動前

上一趟 A/B 實驗只更新 Leader 和 F2，F1 Pi 保持舊版。常態跟隨 `0.1.0`
需把 `follower_formation.py`、`wave_path.py` 更新到 F1 Pi；F2 也使用此版本的
`follower_formation.py`、`follower2_formation.py`、`wave_path.py`。Leader 必須
使用發送 `path` 欄位的 `leader_formation.py`、`wave_path.py`。先以
`python3 follower_formation.py --version`／`python3 follower2_formation.py --version`
核對 follower 版本，再執行實車準備。完整校正方案見 `CALIBRATION_PLAN.md`。

1. 接通三台車 Pi、Nano 與馬達電源；先保持馬達可隨時斷電。
2. 建立三個 Mac 終端，分別登入：`ssh pi-leader`、`ssh pi-follower1`、
   `ssh pi-follower2`。
3. 三台都執行 `cd ~/pi_rover`，再用 `ls -l /dev/serial/by-id/` 找到該車 Nano
   的完整 USB 路徑，以下以 `<L_PORT>`、`<F1_PORT>`、`<F2_PORT>` 代稱。
4. 確認沒有殘留控制程式：
   `pgrep -af 'leader_formation|follower_formation|follower2_formation'`。
   有殘留時先在該終端按 `Ctrl-C`，等待程式顯示 STOP 後再繼續。

## 快速地面試跑節奏

當操作者在本專案說「跑」，表示這一趟三台車已靜止、測試區與前方有淨空、
可隨時切斷馬達電源，並接受接地的短暫馬達預檢。目標是在三台正常且操作者
能及時放回起點時，3 分鐘內開始地面路徑運動。

1. 一次完成三台連線、程式版本、Nano USB 及殘留程序檢查；使用下方相同的三台命令。
2. 三台車靜止時依提示送 `CALIBRATE`；**逐台**送 `MOTOR`，不可三台同時轉輪。
   現行程式的「Lift both wheels clear」是原架空流程提示；本快速流程採用操作者
   明確選定的**地面短暫移動**，因此須確保每台車前方與周圍有淨空，且在輸入後
   監看 STOP、`imu_present=1`、`imu_calibrated=1`、`encoder_preflight=1`、
   `fault_code=0`。若任一項失敗，停止三台並排查，不啟動路徑。
3. 三台全數通過後，停在 `FOLLOWER-1`、`FOLLOWER-2`、`WAVE-3` 提示；
   讓操作者把三車放回標記並確認車頭平行、路徑淨空。**這是預檢後唯一一次
   合併的使用者確認**。
4. 收到「現在出發」的肯定答覆後，立刻依序送 `FOLLOWER-1`、`FOLLOWER-2`、
   `WAVE-3`。前兩個字在最終起點 RESET follower，最後一個字 RESET Leader
   並立即行走；送出後不再搬動車。

工具自動審核或任何故障仍可能阻止動作；不能繞過拒絕。此流程不改變三台控制律。

## 原架空輪子手動啟動順序

### 1. 先讓 F1 等待

先將 F1 輪子架空；校正期間車體必須完全靜止。輸入：

```bash
python3 follower_formation.py --port <F1_PORT> \
  --leader-host 192.168.1.166 --listen 0.0.0.0:5005 --unlock
```

程式會要求輸入 `CALIBRATE`，接著要求 `MOTOR`；只有在兩輪皆架空、四周安全時
才輸入，F1 會以 PWM 80 轉 800 ms 做 encoder A/B preflight。通過後，把 F1 放到
Leader 左後方 400 mm、左方 400 mm，朝向與 Leader 平行，再輸入 `FOLLOWER-1`。
F1 將 RESET 自己的里程計並保持停止等待。

### 2. 再讓 F2 等待

同樣先將 F2 輪子架空，輸入：

```bash
python3 follower2_formation.py --port <F2_PORT> \
  --leader-host 192.168.1.166 --listen 0.0.0.0:5005 --unlock
```

依提示在車靜止時輸入 `CALIBRATE`，在輪子架空時輸入 `MOTOR`。F2 使用 PWM 40、
400 ms 的較保守 encoder preflight。通過後，將 F2 放到 Leader 右後方 400 mm、
右方 400 mm，朝向與 Leader 平行，再輸入 `FOLLOWER-2`。F2 也會停止等待。

### 3. 最後啟動 Leader 波浪路徑

Leader 預檢時先架空兩輪；輸入：

```bash
python3 leader_formation.py --port <L_PORT> wave --unlock \
  --length-mm 1500 --amplitude-mm 50 --cycles 3 --speed-mm-s 45 \
  --broadcast 192.168.1.199:5005 \
  --broadcast 192.168.1.175:5005
```

依提示於車體靜止時輸入 `CALIBRATE`，並在兩輪架空時輸入 `MOTOR` 完成預檢。
三台車皆已放到地面、路徑淨空且朝向確認後，才輸入 `WAVE-3`。Leader RESET 後開始
運動；F1/F2 要收到連續三筆有效的 Leader moving frame 才會起步。
兩台 follower 的三筆封包都必須含有效 `path`。此模式僅對 Pi 規劃的 `wave`
生效；Nano 韌體的 `leader 1/2` PATH 不提供路徑參數，兩台都會保持停止。

## 實車前理論模擬

Mac 在 repo 根目錄執行：

```bash
MPLCONFIGDIR=/private/tmp/mpl_f1f2 .venv/bin/python pi_rover/simulate_formation.py
```

結果在 `analysis/2026-10-29-f1-f2-theory/`：`comparison.png` 是軌跡與誤差，
`control_explained.png` 是 F2 進度差、彎道限速與轉速命令，
`summary.json` 是統計，`simulation.csv` 是每 0.1 s 的指令和姿態，
`sample_packets_and_controls.json` 有實際封包格式與兩台 follower 的命令。
這是理想差速運動學，使用目前 Python 控制器和輪速限幅；沒有模擬輪胎打滑、
馬達響應、IMU 漂移或網路延遲。這個模擬仍是**舊 F1／新 F2 的歷史 A/B**；
兩者的距離誤差不可單獨當作控制器優劣比較。常態版兩台都改用路徑追蹤，
請同時看各自車道橫向誤差與進度落後量。

## 運動中與停止

- 任一車有異常，先切斷馬達電源；接著在該 Pi 控制程式終端按 `Ctrl-C`。
- `Ctrl-C`、USB 中斷、UDP 超過 0.35 s、fault、preflight 失敗或程式結束，都會
  嘗試送 Nano `STOP`。
- 正常到達 1.5 m 終點時，Leader 送 `STOP` 並發布 `stopped`；Follower 收到後停止。
  因最後一筆速度指令的到期時間為 180 ms，停止仍可能有不超過約 0.18 s 的命令延遲與
  機械慣性。

## 實驗後資料

每次 Leader run 的 CSV 在 `~/pi_rover/logs/wave_<UTC>.csv`；F1 在
`follower_<UTC>.csv`；F2 在 `follower2_<UTC>.csv`。用完請先複製這三個 CSV 到 Mac
再關機。三台車目前的完整備份與歷史 CSV 在 repo 的
`backups/2026-10-13/`。
