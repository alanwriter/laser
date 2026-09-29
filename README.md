# 綠色 X 雷射自動指向系統（Mac 端）

本專案用 USB／iPhone 攝影機觀察印有綠色 X 的紙面，控制既有二軸雷射雲台，讓紅色雷射點逐步接近綠色 X 中心。它處理 Pan/Tilt 軸互相耦合、雲台不水平，以及紅點在紙面上呈弧形移動的情況。

> 這個資料夾只有 Mac 端 Python 程式。它**不會上傳、修改或管理 Nano 韌體**，也沒有雷射開關控制。既有 Nano 韌體只需能從 USB serial 接收一行 `P,T\n`，例如 `155,30`。

## 功能與流程

1. 在全畫面偵測中央的大型綠色 X，取其中心為目標。
2. 由 X 外框建立黃色 `PAPER ROI`；紅色雷射只會在此區域內被搜尋，紙外的螢幕、線材與反光不會影響自動控制。
3. 量測多個 P/T 參考姿態中雷射的影像座標，建立 `(P,T) → (x,y)` 的二維映射。
4. 映射先估算中心 P/T；再以紙內紅點的視覺回授分段修正，因此能修正實際機構的弧形路徑。
5. 在相機視窗按 Space 開始移動時，同步錄製含所有標記的 MP4。

四或五個有效點建立完整的 2-D homography，可描述斜向偏移與交叉耦合。若只剩三個有效點，程式會用 3 點 affine（仿射）轉換，仍保留旋轉、縮放、剪切及交叉耦合，但精度通常較低。

## 檔案

| 檔案 | 用途 |
|---|---|
| `vision_pointer.py` | 視覺偵測、校正、尋回、閉迴路瞄準、錄影主程式。 |
| `nano_console.py` | 直接手動傳送一行 P/T 命令。 |
| `requirements.txt` | OpenCV、NumPy、pyserial 依賴。 |
| `anchor_calibration.json` | 執行校正後產生的映射資料。 |
| `recordings/` | Space 後產生的 MP4。 |

## 硬體設定

- Nano serial port：`/dev/cu.usbserial-1320`
- baud rate：`115200`
- 相機：USB 鏡頭或 iPhone Continuity Camera；目前範例使用 `--camera 2`。
- 雷射：長亮，程式不控制開關。

舵機若使用獨立電源，舵機電源的 **GND 必須與 Nano GND 共地**。不要用 Nano USB 的 5 V 直接供應舵機；供電不足可能造成雲台不動、抖動或 Nano 重置。

## 安裝

`cv2` 是 Python 匯入名稱，不是 PyPI 套件名稱；不要執行 `pip install cv2`。

```bash
cd ~/program/laser
source .venv/bin/activate
pip install -r requirements.txt
```

也可直接使用虛擬環境：

```bash
.venv/bin/python vision_pointer.py --help
```

## 相機、視窗與黃框

在 macOS「系統設定 → 隱私權與安全性 → 相機」中允許實際執行指令的 Terminal、iTerm、VS Code 或 Codex 使用相機；授權後重開該 App。

列出鏡頭（不開 Nano serial）：

```bash
python vision_pointer.py --list-cameras
```

預覽 iPhone 相機：

```bash
python vision_pointer.py --mode preview --camera 2
```

預設為 `1280×720 / 30 fps`，並要求單張緩衝。iPhone 延遲大時：

```bash
python vision_pointer.py --mode preview --camera 2 \
  --camera-width 960 --camera-height 540 --camera-fps 30 --camera-buffer-size 1
```

Wi‑Fi Continuity Camera 有延遲時，改用 USB 線連接 iPhone 通常更好。OpenCV 視窗預設放在主螢幕 `(80,70)`，大小 `1200×750`；可用 `--window-x`、`--window-y`、`--window-width`、`--window-height` 修改。

畫面標記：

- 綠色十字 `GREEN X`：偵測到的目標中心。
- 黃框 `PAPER ROI`：唯一允許辨識或接受紅點的區域；預設為 X 外框 `2.6` 倍。
- 紅色空心圈：紙內紅點候選；中心不畫叉，避免遮住實際光點。
- 青色 `REFERENCE`：手動校正時使用者點選的位置。

黃色框太小或太大時：

```bash
python vision_pointer.py --mode preview --camera 2 --paper-roi-scale 3.0
```

請讓它覆蓋雷射可能落點的整張白紙，盡量不要包含紙外紅色物體。預覽時若要顯示紙內紅點候選：

```bash
python vision_pointer.py --mode preview --camera 2 --show-paper-red
```

淡粉／過曝紅點可加 `--detect-pale-red`，但它較容易誤判紙張反光；請先在 preview 確認正確再使用。

## Nano 命令與手動雲台控制

所有命令都是整數 `P,T`：

```bash
python nano_console.py --command 155,30
python nano_console.py --command 35,5
```

互動輸入：

```bash
python nano_console.py
Nano> 155,30
Nano> 35,5
Nano> q
```

`Resource busy` 代表另一個 Serial Monitor、PlatformIO、VS Code serial 擴充或 Python 程式正在使用 Nano；同一時間只能一個程式開啟 serial：

```bash
lsof /dev/cu.usbserial-1320
```

Nano 開啟 serial 後可能重置；主程式會等待約 2 秒，期間相機視窗仍持續更新。若 USB serial 短暫重新枚舉，程式最多等 10 秒。

## 參考姿態

| 示意位置 | P,T |
|---|---:|
| 左上 | `155,30` |
| 正上中間 | `20,45` |
| 右下 | `35,5` |
| 左下 | `145,0` |
| 右上 | `5,35` |

可覆寫為分號分隔的 `標籤:P,T`：

```bash
python vision_pointer.py --mode calibrate --camera 2 --manual-capture \
  --poses 'left_top:155,30;right_bottom:35,5;left_bottom:145,0;right_top:5,35'
```

預設安全範圍為 Pan `0–160`、Tilt `0–45`；超出範圍的參考點或自動修正會被拒絕。

## 操作模式

| 模式 | 功能 |
|---|---|
| `preview` | 只顯示相機、X 與黃框；不開 serial、不動雲台。 |
| `recover` | 依序走參考 P/T，找到穩定紙內紅點後停止；不覆寫校正。 |
| `calibrate` | 蒐集 P/T 對應畫面座標並儲存校正檔。 |
| `aim` | 載入校正檔，估算中心命令，再以紅點回授修正。 |
| `calibrate-and-aim` | 先校正再瞄準，為預設模式。 |

除 `preview` 外，程式會先等相機與綠色 X 可用。請在相機視窗按 **Space** 才開始送 P/T；按 `q` 停止。若要不等待 Space，才加入 `--auto-start`。

### 尋回紙面

紅點跑到紙外時：

```bash
python vision_pointer.py --mode recover --camera 2
```

程式依序巡訪參考姿態，每點最多等待 4 秒，只接受黃框內穩定紅點；找到後停在該位置。需要多巡一次：

```bash
python vision_pointer.py --mode recover --camera 2 --recover-cycles 2
```

## 手動校正（建議）

反光與過曝可能令自動紅點辨識誤判；最可靠做法是手動點選真實紅點：

```bash
python vision_pointer.py --mode calibrate --camera 2 --manual-capture
```

每到一個姿態：

- 紅點位於黃框內：滑鼠左鍵點實際紅點。
- 紅點位於黃框外：點一下相機視窗取得鍵盤焦點，再按 `n` 跳過。
- 按 `q`：停止整次校正。

`--manual-capture` 絕不自動採用紅色候選，避免錯誤資料污染映射。若只想看到候選紅圈作提示，可加 `--show-paper-red`；最後仍以滑鼠點擊座標為準。

理想上取得四或五個分散點。只有三個有效點時會使用 affine fallback；少於三個則拒絕校正。綠色 X 中心必須在參考點形成的範圍內，程式不會對區域外目標強行外插。

校正檔預設為 `anchor_calibration.json`。相機、紙、綠色 X、雲台位置或相機實際解析度變動後，都必須重新校正。

## 自動瞄準與弧形補償

完成校正後，使用紅點回授瞄準：

```bash
python vision_pointer.py --mode aim --camera 2 \
  --gain 0.35 --min-step-deg 5 --max-step-deg 5 --max-iterations 12
```

瞄準演算法會：

1. 由校正矩陣計算 X 中心的初始 P/T。
2. 送出初始命令後，在黃框內量測穩定紅點。
3. 以紅點到 X 中心的像素誤差與局部二維 Jacobian，算出同時修正 Pan、Tilt 的量。
4. 限制單步角度、送出新 P/T、重新觀察實際紅點，直到誤差低於預設 12 px 或到達迭代上限。

因此紅點即使走弧線，也會依每一步的實際紙面位置校正，而不是假設 Pan 只控制水平、Tilt 只控制垂直。終端會印出 X、紅點、誤差、修正量與實際送出的 P/T。

Nano 接受整數 P/T；小數修正若被四捨五入，雲台看起來就不會動。`--min-step-deg` 跨過死區，`--max-step-deg` 限制單步。機構約 5° 才有反應時使用上例的 `5 / 5`；較靈敏時可調小。

只想送出初始估計、不作紅點確認：

```bash
python vision_pointer.py --mode aim --camera 2 --no-red-feedback
```

## 錄影

`recover`、`calibrate`、`aim`、`calibrate-and-aim` 都會在按下 **Space** 的瞬間開始錄製帶有黃框、綠色 X、紅點圈與狀態文字的 MP4。成功、錯誤或按 `q` 結束時都會關閉並印出檔案位置：

```text
recordings/laser-YYYYMMDD-HHMMSS.mp4
```

指定其他輸出位置：

```bash
python vision_pointer.py --mode aim --camera 2 --recording-dir ~/Desktop/laser-videos
```

本次不錄影則加 `--no-record`。`preview` 沒有 Space 動作，不會自動錄影。

## 常見問題

| 現象 | 處理方式 |
|---|---|
| `ModuleNotFoundError: cv2` | 啟用 `.venv` 後執行 `pip install -r requirements.txt`；不要 `pip install cv2`。 |
| 無法開啟相機／無畫面 | 檢查 macOS 相機權限、關閉佔用鏡頭的 App，用 `--list-cameras` 找正確 index。 |
| `USB camera frame read failed` | 重連／重開相機，改用 `960×540`，確認沒有其他 App 正在讀取。 |
| serial `No such file or directory` | Nano 未連上或 port 名稱改變；用 `ls /dev/cu.*` 確認。 |
| serial `Resource busy` | 關閉其他 serial monitor 或 Python 程式。 |
| 紅點在紙上卻找不到 | 先確認黃框覆蓋紙面；用 `--show-paper-red` 預覽，必要時採用手動校正。 |
| 手動點擊被忽略 | 點擊必須在黃框內；提高 `--paper-roi-scale`。 |
| 有紅點座標但雲台不動 | 檢查獨立電源與共地，提高 `--min-step-deg`，查看終端的 `-> send P=..., T=...`。 |
| `Aiming did not reach...` | 到達迭代上限仍未進入 12 px；重點參考點、調整步幅／迭代上限，並檢查紅點候選。 |

## 安全

- 雷射長亮；按 `q`、偵測失敗、程式錯誤或 serial 斷線都**不會關閉雷射**。請使用實體開關並先以低功率／安全環境測試。
- 不要照射人眼、相機感測器或鏡面反射物。
- 自動移動前確認雲台機械極限、紙面位置與黃色 ROI。
