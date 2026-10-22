# 三車編隊操作手冊

本手冊對應 `laser` `main` 分支目前的程式。每台 Nano 的 USB port
只能由其本車 Pi 上的一個 Python 程式開啟；不要同時執行 `status`、手動測試
或第二個編隊程式。

## 目前預設實驗

- Leader：1.5 m、三週期、±50 mm 的 Pi 規劃平滑波浪軌跡，45 mm/s。
- F1 目標：Leader 車體座標 `(-400, +400)` mm，左後方。
- F2 目標：Leader 車體座標 `(-400, -400)` mm，右後方。
- Leader 對 F1/F2 的 UDP 直接單播：`192.168.1.199:5005` 與
  `192.168.1.175:5005`。
- Leader 抵達終點後立即廣播 `stopped`；Follower 停止，不做預設的終點追趕。

若換了網路，先在 Leader 上執行 `hostname -I`，把得到的 Leader IPv4 改寫到
兩個 Follower 指令的 `--leader-host`；同時把兩台 Follower 的新 IPv4 改寫到
Leader 的兩個 `--broadcast`。三台 Pi 只需在同一個本地網路，不需要 Internet。

## 啟動前

1. 接通三台車 Pi、Nano 與馬達電源；先保持馬達可隨時斷電。
2. 建立三個 Mac 終端，分別登入：`ssh pi-leader`、`ssh pi-follower1`、
   `ssh pi-follower2`。
3. 三台都執行 `cd ~/pi_rover`，再用 `ls -l /dev/serial/by-id/` 找到該車 Nano
   的完整 USB 路徑，以下以 `<L_PORT>`、`<F1_PORT>`、`<F2_PORT>` 代稱。
4. 確認沒有殘留控制程式：
   `pgrep -af 'leader_formation|follower_formation|follower2_formation'`。
   有殘留時先在該終端按 `Ctrl-C`，等待程式顯示 STOP 後再繼續。

## 正確啟動順序

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
