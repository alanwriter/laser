# 2026-11-12 候選：encoder 距離＋MPU 航向、雙新版 follower、左轉

## 目前狀態

本地候選程式已支援原本 S 彎 `eased_sine_v1` 和左轉 `left_turn_v1`，
F1/F2 都使用 `PathFormationTracker`。Nano 候選韌體與三份 `.hex` 在
[`firmware/builds/2026-11-12/`](../firmware/builds/2026-11-12/)，
已逐台備份原 flash／EEPROM、燒錄並驗證，`CONFIG` 第九欄均讀回 `1.00`；
2026-11-12 三台 Pi 的新程式**已部署並核對 `--version`**，部署前原檔保存在
`backups/2026-11-12-pre-left-turn/` 與每台 Pi 的
`~/pi_rover/backups/2026-11-12-pre-left-turn/`。不能從本地模擬或 Pi 版本
推論實車已行走驗證。

先前 Nano 的 70/30 韌體每步將左右 encoder 換成距離，航向混合 70% MPU6050
gyro-Z 積分與 30% 輪差。2026-11-12 候選韌體改為 100% MPU 航向；兩輪 encoder 仍負責
距離、輪速 PID 與監看。MPU6050 在這裡提供**相對航向**，不是絕對方位，
長時間偏移仍需用資料檢驗。

## 資料如何保存

Leader、F1、F2 的每筆 CSV 保留 Nano 原始左右 `count`、TPS、目標 TPS、PWM、
車上位置與航向。新欄位 `config_*` 保存每車 `IO,CONFIG` 回報的 ticks/圈、
輪徑、輪距與 `heading_gyro_blend`；`encoder_left_distance_mm`、
`encoder_right_distance_mm`、`encoder_forward_distance_mm` 依該車回報常數換算，
不覆寫原始 count。`--require-mpu-heading` 會在 RESET／出發前拒絕沒有回報
`heading_gyro_blend=1.00` 的 Nano。
每份 CSV 開頭另有 `preflight`（MOTOR 預檢後、最終位置 RESET 前）與
`origin`（RESET 後）事件，便於分開分析短暫預檢和正式行走的 encoder count。

每趟保留三份原始 CSV、起點標記照片與影片、三台 Pi 的 `--version`、Nano
`CONFIG`、韌體 `.hex` SHA-256、路線參數及起止 UTC 時間。多趟資料按趟分目錄
保存，原始 CSV 不修改；今天跑完再分析輪速跟隨、航向偏移及影片間距。

## 目前左轉候選參數：Leader 終點前進 1 m、左移 1.2 m

使用者指定的是 **Leader MPU 終點位移**。先直走 200 mm，接半徑
800 mm、90° 左彎，再走 400 mm；巡航 30 mm/s。Leader MPU 終點為
`(1000,1200)` mm、航向 90°，路程約 1856.6 mm。F1/F2 起點維持
`(-400,+400)`／`(-400,-400)` mm，各自沿路徑進度落後 400 mm，
彎中車道半徑分別為 400／1200 mm。這是離線候選，三台仍未依此實車行走。
模型、車身邊界及侷限見
[`analysis/2026-11-12-compact-left-turn/`](../analysis/2026-11-12-compact-left-turn/README.md)。

## 先前較大範圍左轉候選（不供本次試跑）

Leader：先直走 1,500 mm，以半徑 1,200 mm 左轉 90°，再走 1,000 mm；
巡航 30 mm/s，總路程約 4,385 mm。F1/F2 分別從 Leader 左後／右後
`(-400,+600)`／`(-400,-600)` mm 開始，沿各自車道落後 400 mm。左轉時
F1 是內側 600 mm 半徑、F2 是外側 1,800 mm 半徑；目標速度比例為
Leader 的 0.5／1.5 倍。這些是路徑幾何目標，實際速度仍受每車控制限制。
圖與離線模擬見 [`analysis/2026-11-12-straight-left-turn/`](../analysis/2026-11-12-straight-left-turn/README.md)。

## 部署與出發前的核對

1. 已保存每台 Nano 的原 flash／EEPROM 於
   [`backups/2026-11-12-pre-mpu-flash/`](../backups/2026-11-12-pre-mpu-flash/)，
   並逐台核對 profile、燒錄對應 `.hex`、由 avrdude 驗證寫入；
   `CONFIG` 第九欄三台都為 `1.00`。USB 重啟後 IMU 校正與 encoder 預檢
   尚未重做，這些須在真正試跑時依序進行。
2. 已將同一份本地候選 `leader_formation.py`、`wave_path.py`、`turn_path.py`
   部署到 Leader；F1/F2 已更新 `follower_formation.py`、`leader_formation.py`、
   `wave_path.py`、`turn_path.py`，F2 保留 `follower2_formation.py`。
   `leader_formation.py` 是 follower 共用的 Nano 協定模組。
3. 唯讀核對版本：Leader `python3 leader_formation.py --version` 應為
   `route-controller 0.2.0-dev+left-turn`；F1/F2 的 `--version` 應為
   `normal-follow 0.2.0-dev+left-turn`。再核對 USB、網路、殘留程序與 F1
   右輪閉環 TPS／PWM；預檢旗標通過不等於輪子機械狀態正常。
4. 依 [RUNBOOK](RUNBOOK.md) 逐台接地短暫馬達預檢並監看 STOP；這會讓車移動，
   只能在使用者授權這一趟「跑」且測試區淨空時執行。三台預檢通過後放回
   MPU 起點標記，車頭平行，**只做一次合併的最終出發確認**，然後依序輸入
   `FOLLOWER-1`、`FOLLOWER-2`、`WAVE-3`。RESET 後不再搬動。

候選命令需依現場 USB by-id 與 Pi IP 修改；這裡不代表已啟動：

```bash
# F1: 左車道；在最終位置 RESET
python3 follower_formation.py --port <F1_PORT> --leader-host <LEADER_IP> \
  --offset-y-mm 400 --follower-start-y-mm 400 --require-mpu-heading --unlock

# F2: 右車道；在最終位置 RESET
python3 follower2_formation.py --port <F2_PORT> --leader-host <LEADER_IP> \
  --offset-y-mm -400 --follower-start-y-mm -400 --require-mpu-heading --unlock

# Leader: 兩個 --broadcast 填入 F1/F2 當下 IP
python3 leader_formation.py --port <L_PORT> turn \
  --straight-mm 200 --radius-mm 800 --exit-mm 400 --speed-mm-s 30 \
  --broadcast <F1_IP>:5005 --broadcast <F2_IP>:5005 \
  --require-mpu-heading --unlock
```

2026-11-12 已經完成三台 Pi 程式部署及三台 Nano 燒錄；兩次試跑授權均因
Leader IMU 開機偵測間歇失敗而在馬達預檢前中止。Leader 隨後在馬達斷電下
燒錄開機重試候選，12 次重啟唯讀核對通過；**尚未送出任何行走或馬達預檢命令**。
詳見 [`中止紀錄`](../analysis/2026-11-12-compact-left-turn/preflight_abort_leader_imu.md)。
F2 Pi 沒有系統安裝的 `avrdude`，這次改在暫存資料夾解包
Debian 套件並用 `firmware/avrdude_f2_user_space.sh` 執行。下次接手先重新
核對現況，不拿此紀錄或備份當即時量測。

部署後 Pi 來源 SHA-256：`leader_formation.py`
`a430ef46e4772d109df50ce904aada70075cc3200c317f2a7e89672f6320151e`、
`follower_formation.py`
`8b3db348ad71670468866e4cc106cc314f811f372ffef0269a1f899d3736bd83`、
`turn_path.py` `2f37b5dd695f781a77a82f3aaac9efca4ee5ea6d6c69556cd21b262fc416e1e2`。
