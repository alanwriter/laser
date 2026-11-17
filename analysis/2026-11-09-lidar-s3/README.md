# RPLIDAR S3 短掃描（2026-11-09）

Mac USB 裝置：CP2102N UART bridge；埠 `/dev/cu.usbserial-1230`，1,000,000 baud。
感測器 `GET_INFO` 回覆型號代碼 `0x81`，對應 S3M1；韌體 1.2，硬體版本 18。`GET_HEALTH` 狀態 0、錯誤碼 0。

以標準掃描模式取得約 3 秒資料，共 42,700 筆量測紀錄、29 個整圈起點；結束時已送 STOP。此圖取最後一個完整圈，共 1,610 筆，其中 1,029 筆距離非零。有效距離範圍 81.0–4203.5 mm，中位數 472.8 mm。

`single_revolution.csv` 保留該圈所有角度、距離與品質值；距離 0 表示該方向沒有有效回波。`single_revolution.png` 左圖放大近場，右圖顯示完整量測範圍。原點是 LiDAR，自身角度零點未對齊房間或車體方向。此為固定位置的單圈 2D 截面，不能直接當成房間平面圖或地面真值。

原廠資料：[S3 規格](https://www.slamtec.com/en/s3/spec)、[S/C 系列協定](https://bucket-download.slamtec.com/c5971f2703a8d014f3925694d798ea490a370efa/LR001_SLAMTEC_rplidar_S%26C%20series_protocol_v2.8_en.pdf)。
