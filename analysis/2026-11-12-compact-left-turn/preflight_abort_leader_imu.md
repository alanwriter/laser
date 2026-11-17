# 2026-11-12 第一趟中止：Leader MPU6050 間歇性未偵測

操作者確認 `1 m 前進／1.2 m 左移` 指 **Leader MPU 終點位移**。
原第一趟授權沿用於修正後路線：先直走 200 mm、半徑 800 mm 左轉 90°、
再走 400 mm；F1/F2 各在 `(-400,±400)` mm，使用新版路徑跟隨器。

啟動前唯讀核對三台 Pi 均為 `0.2.0-dev+left-turn`、Nano USB 存在、
`CONFIG` 第九欄均為 `1.00`、沒有殘留編隊程序。三個互動程序啟動後，
Leader 在 `commission` 的 `HELLO → STOP → IMU → ENCODER → STATUS` 階段
讀到 `IO,IMU,...,present=0,calibrated=0,address=105`，`STATUS` 同樣
`imu_present=0`。Leader 程式報 `MPU6050 is not detected`，送 STOP ACK 後退出。
F1/F2 此時只停在 `CALIBRATE` 輸入提示，尚未送出校正；立即以 Ctrl-C
讓兩台分別送 STOP 並收到 ACK，程序退出。

**沒有送 `CALIBRATE`、`MOTOR`、`FOLLOWER-1`、`FOLLOWER-2` 或 `WAVE-3`；
三台車沒有依這套程序行走。** 隨後只做三次 Leader IMU 唯讀複查，
`present` 依序 `1／0／1`，成功時 address=104，失敗時 address=105。
這表示偵測狀態不穩定，不能因最後一次讀到 1 就繼續預檢或出發。
最後唯讀查三台均無殘留編隊程序。

下次先由操作者檢查 Leader MPU6050 的電源、GND、SDA/SCL 接線與接頭，
在車體靜止、馬達可斷電時反覆唯讀核對 IMU 穩定，再重新開始**新的一趟**
預檢。F1 右輪閉環 TPS／PWM 問題仍待驗證；此次沒有取得實跑 CSV。

## 接線後第二次授權：仍在唯讀階段中止

操作者表示接線已確認，重新授權一趟。三台 Pi 版本、Nano USB 與
`CONFIG=1.00` 均再次核對；沒有殘留控制程序。Leader 連續三次 USB
重新開啟的唯讀 IMU 結果為 `present=1／1／0`，第三次失敗時回報
`address=105`。**這裡的 105 是韌體依序嘗試 0x68、0x69 後留下的最後
探測位址，不足以證明感測器實際改了位址。** 韌體只在開機時呼叫一次
`initialiseMpu6050()`，若未偵測到會維持 `imu_present=0`，直到下次重啟。
因此此狀況可能是開機時序或實體 I²C／供電連線不穩，尚無證據可定因。

三台 Nano 均再次明確收到 `IO,ACK,1,STOP`；第二次**沒有啟動互動
控制程序、CALIBRATE、MOTOR 或行走**。Leader Pi 的 USB CH340 裝置存在；
`vcgencmd get_throttled` 因 `/dev/vcio` 不可用，未取得供電節流讀值。
本次故障重複出現，不能憑兩次成功的一次性讀值放行出發。

追加的 Leader 唯讀診斷做了五次 USB 開啟／關閉，每次在同一會話內
讀取 IMU 八次：五次開機初始偵測依序 `1／1／0／1／0`；成功的會話
八次讀取均維持 `present=1`，`read_failures=0`、
`consecutive_read_failures=0`；失敗會話則維持 `present=0`。
這支持「開機探測時機」為可能原因，但不能排除接線或供電間歇異常。

操作者確認 Leader 靜止且馬達電源已切斷後，先備份當前 flash／EEPROM，
再只燒錄 [`Leader 開機重試候選`](../../firmware/builds/2026-11-12/README.md)。
avrdude 寫入與驗證通過。燒錄後 12 次 USB 重啟、每次八次 IMU 唯讀查詢
全數 `present=1`、address=104、讀取失敗數 0；`HELLO` 為 L1，
`CONFIG` 航向係數 1.00。診斷腳本為 [`probe_leader_boot_imu.py`](probe_leader_boot_imu.py)。
此操作沒有校正、馬達預檢或行走；需重新授權新一趟才進入三車預檢。
