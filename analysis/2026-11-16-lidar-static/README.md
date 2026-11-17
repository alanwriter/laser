# Leader 光達靜止近距掃描：中止紀錄（2026-11-16）

## 目的與現場條件

評估 Leader 上的 RPLIDAR S3M1，能否在左右約 250 mm 的位置看見 F1、F2。操作者確認三車已就位且靜止、輪控馬達電源已切斷；測試期間表示沒有碰觸 Nano USB／電源線或搬動車身。本次沒有發出任何車輪控制命令，也沒有修改 Follower 的安全距離。

Pi 上的 S3 使用 CP2102N USB 串列埠，Nano 使用 CH340。將本機 `lidar_live_view.py` 與 `lidar_live_view.html` 複製到 Leader 的 `/tmp/laser-lidar-static-20261116/`，以 15 秒 `timeout` 限時執行標準掃描並記錄 JSONL.gz；掃描程式退出時送 S3 STOP。

## 實際結果

- [掃描日誌](scan_static_01.log)：S3 `GET_INFO` 成功，回報 S3M1、韌體 1.2；進入掃描後解析器遇到 `Corrupt scan stream`，程式送出 `LiDAR STOP sent` 並退出。後續確認沒有殘留 `lidar_live_view.py` 程序。
- [原始紀錄](scan_static_01.jsonl.gz) 為有效 gzip，但含 **0 行完整掃描**；沒有足夠點雲判斷 F1／F2 是否被看到，也無法量出 250 mm 位置或線材回波。
- [Leader USB 核心事件](leader_usb_events.txt)：同一分鐘的 15:09:37，CH340（Nano，`ttyUSB0`）斷線並重新列舉，15:09:39 才重新附掛。S3 的 CP2102N 為另一個裝置。此時間重合值得調查；單憑日誌不能確定是供電不足、USB 連線、電磁干擾或其他原因。
- Leader 並未整機重啟：測後 uptime 約 1 小時 20 分。先前 2026-11-09 的 [Mac S3 短掃描](../2026-11-09-lidar-s3/README.md) 可得到整圈資料；該結果不能代替目前 Leader Pi、共接 Nano 的電源／USB 驗證。

## 判定與後續門檻

**本次不通過近距可見性測試；原因是沒有有效掃描資料，不是證明 S3 看不到兩車。** Nano USB 在無人碰線的掃描期間重連，應先停在故障診斷，不重複啟動同樣掃描。

下次先在光達不轉動時確認供電、USB 線與接頭、Nano USB 穩定性，必要時分離光達供電或使用合適的供電 USB 集線器；保留兩裝置的核心事件與時間戳。再使用可記錄原始串列位元組／失步位置的短掃描或原廠 SDK，分辨解析器失步與串列資料實際中斷。上述條件穩定後，才比較空場、單獨 F1、單獨 F2、雙車的同一掃描高度點雲；記錄各車的可見率、回波角度／距離及線材是否被掃到。

掃描當時 `collision_stop_distance_mm=150`，是以車上估計的 Leader–Follower MPU 點距離判斷，**不是車身或線材淨空**。使用者在本次異常後選擇不重掃，並要求把 F1/F2 控制器門檻改成 250 mm；本機及兩台 Follower Pi 已更新，舊檔分別備份於 `~/pi_rover/backups/2026-11-16-proximity-250mm/`。新檔 SHA-256 為 `02d95a3fceea9741988a8d9924fbc51e65238a43c7ffecef037e30b1cbd1ac0c`，尚未行走驗證。仍需按 250 × 150 mm 車身外框、MPU 錨點、姿態及線材突出量計算最小間隙，並以外部量測驗證；不能由單一中心距離保證不接觸。

資料來源：本資料夾三個原始檔；[S3 原廠規格](https://www.slamtec.com/en/s3/spec)、[S/C 系列掃描協定](https://bucket-download.slamtec.com/c5971f2703a8d014f3925694d798ea490a370efa/LR001_SLAMTEC_rplidar_S%26C%20series_protocol_v2.8_en.pdf)。
