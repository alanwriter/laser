# 2026-11-12 Nano 燒錄前映像

三台車在馬達電源切斷、Nano USB 供電時，先用 `avrdude` 讀取原 flash 與
EEPROM，再各自複製到本機此資料夾；車上同時保存在
`~/pi_rover/backups/2026-11-12-pre-mpu-flash/`。`read.log` 是讀取紀錄，
`write.log` 包含候選 `.hex` 的寫入與驗證紀錄。這些檔案是**原始備份**，
不可用來推定燒錄前原始碼或目前執行狀態。

| 車 | 原 flash SHA-256 | 原 EEPROM SHA-256 |
|---|---|---|
| Leader | `4fead7c9036aa38be37e48005568c88e1f0b3e27720cdd8185c50daf34f3514e` | `a70edff7ee05387bc4d69419fc6da25c8d3ee828eea6db7b15cad890b5fd10a0` |
| F1 | `24c2358228a61f47ce7badff88a728497b9cb9b6305639c37523a4744028e005` | `c791b0c287ff4a157639ac75d9ff043be71475e1efa6267e2542261e6fce308f` |
| F2 | `f34aa3a7d62ce7f7c7afa84523fcdbc456fd35ff9342b1d2775881cc37e3d72c` | `5c1a8b58d240d708c82c12700e972906741bc142870555140ee1b9b8cf3e1b39` |

之後三台已各自燒錄 [純 MPU 航向候選韌體](../../firmware/builds/2026-11-12/README.md)，
avrdude 驗證完成，`HELLO` 車型各為 L1／F1／F2，`IO,CONFIG` 第九欄
均回報 `1.00`。尚未做行走試驗。若需還原，先確認車輛靜止且馬達電源切斷，
按車名選對原 flash；不要把 F1/F2 映像互換。
