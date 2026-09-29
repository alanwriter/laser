# Raspberry Pi 軟體專案交接紀錄

> 此 Arduino 專案只保留 Nano 硬體控制與接線資料。Raspberry Pi 的 Python、
> systemd 服務與遠端控制程式應在另一個專案維護。

## 目前狀態（2026-09-04）

- 已完成 Raspberry Pi OS Lite 的 SD 卡燒錄並登入本機 console。
- Pi 使用者：`alan`。
- 主機名稱：`raspberrypi`。
- Pi 已連上 Wi-Fi；當時 DHCP 位址為 `192.168.1.199`。此位址可能在重新開機後改變，
  建議日後在路由器設定 DHCP reservation。
- Mac 嘗試 `ssh alan@192.168.1.199` 時得到 `Connection refused`，表示網路正常，
  但 SSH 服務尚未啟動。
- **不要在文件、程式或 Git 中保存 Pi 密碼。**

## 下一步：啟用 SSH

在 Pi 的本機 console 輸入：

```bash
sudo systemctl enable --now ssh
sudo systemctl status ssh --no-pager
```

確認顯示 `active (running)` 後，在 Mac 或 Windows PowerShell 連線：

```bash
ssh alan@192.168.1.199
```

若 IP 已改變，在 Pi 輸入 `hostname -I` 查詢目前位址。

SSH 可用後，應從 Mac 建立 SSH key，取代每次輸入密碼：

```bash
ssh-keygen -t ed25519
ssh-copy-id alan@192.168.1.199
```

## 車體分工與接線原則

```text
Raspberry Pi
  路徑、遠端操作、相機／視覺、資料紀錄、開機自動任務
       │ USB Serial（115200 baud）
Arduino Nano
  encoder 中斷、輪速 PID、MPU6050、里程計、馬達安全停止、L298N
```

- Pi 應以 **USB 線接 Nano 的 USB 埠**，不要以杜邦線直接接 Nano D0/D1。
- Nano 保留所有即時控制與停止保護；Pi 的 Linux 不適合執行高速輪速 PID。
- Pi 用獨立、穩定的 5V 電源；不可由 Nano 或 L298N 的 5V 腳供電。
- Pi、Nano、L298N 與馬達電源必須共地。

## 開機自動執行

未來用 `systemd` 服務讓 Pi 開機後以 `alan` 使用者執行 Python 車控程式；
**不需要**啟用本機 console auto-login。

第一版自動程式的安全順序：

1. 等待 Nano USB serial 出現（使用 `/dev/serial/by-id/`，不要硬編 `/dev/ttyUSB0`）。
2. 先傳送 `S`，確保馬達停止。
3. 查詢 `I`、`D`、`P`，確認 MPU6050、encoder 與控制狀態。
4. 等待明確的使用者／實體解鎖訊號後，才傳送移動命令。

不要讓 Pi 一開機就自動送出 `F`、`B` 或 `G1/G2/G3`。
