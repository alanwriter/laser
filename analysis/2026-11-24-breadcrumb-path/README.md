# 2026-11-24 軌跡重建路徑（breadcrumb）：路徑 D 模擬與第一次實跑

**狀態：離線模擬＋單元測試；2026-11-24 08:40:14Z 三車實跑一趟完成（見文末），數字皆為車上估計。**

## 改了什麼

Follower 的路徑來源從「Leader 廣播的解析路線」改成可選擇的兩種，**控制律不變**：

| `--path-source` | 路徑從哪裡來 | Leader 進度 `s_L` |
|---|---|---|
| `analytic`（預設，與 `analytic-path-2026-11-24` 標籤相同行為） | 封包內 `path`（`TurnPath`／`WavePath` 解析式） | Leader 計畫上的 `progress_s_mm` |
| `breadcrumb`（新） | Follower 自己記錄收到的 Leader 位姿 | 記錄點累積弧長 |

- `pi_rover/breadcrumb_path.py`：`BreadcrumbPath` 提供與 `TurnPath` 相同的
  `pose_at_s()`、`project()`、`speed_ratio_at_s()`。
  - 每 20 mm 存一個 Leader 點；只保留 Follower 後方 1 m 以內的點。
  - 切線與曲率：對廣播的 MPU 航向 ψ(s) 在 160 mm 視窗內做局部線性迴歸，
    斜率即曲率 κ。沒有用相鄰點差分（20 mm 間距會把 mm 級雜訊放大成度級航向雜訊）。
  - 車道＝中心線沿左法向偏移 `d`；車道曲率 `κ/(1−dκ)`，速度比 `1−dκ`。
  - 第一個點之前：沿起始切線往後直線延伸（Follower 起點在 Leader 後方 400 mm）。
  - 最新點之後：用 Leader 廣播的「下一個目標點」決定一段等曲率弧，最多延伸 400 mm。
  - 投影在上一次結果 ±500 mm 內搜尋；距離超過 250 mm 時退回全域搜尋。
  - Leader 單一封包位置跳動 > 300 mm 時丟出錯誤（Follower 程式會 STOP）。
- `follower_formation.py`：`PathFormationTracker` 抽出 `_resolve_path()`／`_project()`；
  新增 `BreadcrumbFormationTracker`。每個收到的封包都會記點（含等待期間）。
  breadcrumb 模式只要求封包有 `motion`，不要求 `path`，所以之後 Leader 不走解析路線也能用。
  CSV 新增 `control_path_source`、`control_leader_progress_s_mm`、
  `control_path_curvature_per_mm`、`control_breadcrumb_points`。版本字串 `0.3.0-dev+breadcrumb`。
- `leader_formation.py`：`run_wave` 的每個封包多帶 `next_target`（計畫路線前方 300 mm
  的 x、y、航向）。這是新增欄位；舊 Follower 會忽略它，`version` 仍為 1。

## 模擬設定

路徑 D（11/18 第二趟）：直 600 → R800 左轉 90° → 直 400，Leader 38 mm/s，
F1/F2 起點與目標 (−400, ±400)。兩種路徑來源都直接呼叫實車 Follower 程式；
Follower 是理想差速車（與 `simulate_site.py` 相同），10 Hz。

| 情境 | 說明 |
|---|---|
| ideal | Leader 完全照計畫走 |
| noisy | 廣播航向雜訊 0.5°、位置雜訊 2 mm（1σ），10% 封包遺失 |
| detour | Leader 實際走 R900，封包仍宣稱 R800（代表手動遙控／避障，Leader 不照計畫） |

誤差都以 **Leader 實際走過的軌跡偏移後的車道** 為真值。

## 結果（`summary.json`、`breadcrumb_vs_analytic_route_D.png`）

| 情境／來源 | 橫向誤差 中位／最大 F1、F2 (mm) | 沿路徑間距誤差 最大 F1、F2 (mm) | 車身最小間隙 L–F1、L–F2 (mm) | 終點航向 F1、F2 |
|---|---|---|---|---|
| ideal / analytic | 0.4／2.4、0.7／1.8 | 5.2、1.7 | 245、287 | 89.6°、89.7° |
| ideal / breadcrumb | 0.3／2.6、0.5／2.8 | 11.3、5.6 | 246、287 | 89.1°、89.1° |
| noisy / analytic | 0.5／1.3、0.2／0.5 | 2.7、2.6 | 247、288 | 89.7°、89.6° |
| noisy / breadcrumb | 0.7／1.7、0.5／1.2 | 16.2、9.0 | 244、286 | 88.8°、88.4° |
| detour / analytic | 2.4／**98.4**、2.1／**99.1** | 69.1、81.9 | 262、**177** | 90.4°、90.3° |
| detour / breadcrumb | 0.8／1.7、1.1／1.9 | 9.5、5.1 | 253、287 | 88.7°、88.7° |

六種組合都沒有觸發安全停止。ideal／analytic 與 11/18 的 `candidate_D_v38.json`
完全相同（最近 MPU 486／566 mm、車身間隙 245 mm），代表重構沒有改變原控制器。

讀法：

1. **Leader 照計畫走時，兩種來源幾乎一樣**（橫向誤差都 < 3 mm）。breadcrumb 的沿路徑誤差
   在進彎時最大約 11 mm，原因是曲率視窗把 0 → 1/400 的階躍抹成約 160 mm 的斜坡（圖 d）。
2. **雜訊**：F1 內側車道（半徑 400）把曲率雜訊放大，估計值在真值 2.5 /m 附近擺動約 ±0.5 /m，
   但橫向誤差仍 < 2 mm；回授項把前饋誤差吸收了。
3. **Leader 不照計畫走時差異最大**：analytic 的 Follower 依舊走 R800 的車道，偏離 Leader
   實際車道近 100 mm，F2 與 Leader 車身間隙縮到 177 mm；breadcrumb 仍 < 2 mm。
   這是改用軌跡重建的主要理由。
4. breadcrumb 終點航向少約 0.5–1°（曲率平滑的延遲），影響小。
5. 每次控制計算在 Mac 上約 0.06 ms，Pi 的 100 ms 週期有很大餘裕。

## 限制（上實車前要知道）

- 模擬中 Leader 廣播的位姿就是真實位姿。**實車上 Leader 的里程計誤差會直接變成
  Follower 的車道**（11/12 那趟 Leader 的 encoder 與 MPU 分離就是例子）；解析路徑模式
  也用同一個 Leader 位姿算距離，但車道本身不受影響。
- Follower 是理想差速車，沒有輪速響應延遲、死區或打滑；實車誤差會比表中大。
- 沒有模擬 Leader 原地轉、倒車或停車再走。Leader 停住時不會新增點；倒車會讓弧長
  定義失效，目前不支援。
- Leader 的 RESET 定義共同座標系；Follower 起點仍要放在 (−400, ±400)。

## 重現

```bash
cd analysis/2026-11-24-breadcrumb-path && python3 simulate_breadcrumb.py
cd pi_rover && python3 -m unittest -q   # 61 tests
```

## 上實車時的指令差異

只在 F1/F2 加 `--path-source breadcrumb`，其餘照 RUNBOOK；Leader 需部署本分支的
`leader_formation.py` 才會送 `next_target`（沒送也能跑，只是最新點之後的延伸改用最後曲率）。

## 實跑 2026-11-24 08:40:14Z：路徑 D、F1/F2 breadcrumb，完成，無安全停止

部署：三台 Pi 先把舊的 `leader_formation.py`／`follower_formation.py` 備份到
`~/pi_rover/backups/2026-11-24-pre-breadcrumb/`，再複製本分支的 `leader_formation.py`、
`follower_formation.py`、`breadcrumb_path.py`（SHA-256 前綴 `bdb82167`／`02798f57`／`b43f910b`，與 repo 相同），
三台 `--version` 為 `0.3.0-dev+breadcrumb`。韌體未動（11/18 定版）。

指令（三台 Pi 沒有 tmux，用 `tail -f 輸入檔 | python3 -u …` 送提示字）：
- Leader：`leader_formation.py --port … turn --unlock --straight-mm 600 --radius-mm 800 --exit-mm 400 --speed-mm-s 38
  --test-pwm 60 --test-duration-ms 600 --require-mpu-heading --broadcast 192.168.1.199:5005 --broadcast 192.168.1.175:5005`
- F1／F2：`follower_formation.py`／`follower2_formation.py --port … --leader-host 192.168.1.166 --unlock --test-pwm 60
  --test-duration-ms 600 --require-mpu-heading --path-source breadcrumb`（起點預設 (−400, ±400)）

預檢：三台 IMU present=1、位址 104；靜止校正後逐台接地 PWM 60／600 ms：Leader 178／170、33.5 mm、−0.8°；
F1 725／565、123.5 mm、−14.8°（右輪開環仍偏弱，但比 11/18 的 928／382 好）；F2 986／1039、194.3 mm、+3.6°。
全部 `encoder_preflight=1`、fault 0。操作者一次確認後依序送 `FOLLOWER-1`、`FOLLOWER-2`、`WAVE-3`。
啟動 F2 時 Wi-Fi 短暫中斷一次，指令沒有送達，重送後正常；行走期間沒有網路異常。

Leader 64.9 s 正常完成、Nano STOP；F1/F2 隨即停止等待，之後以 SIGINT 結束，三台 STOP ACK、程序退出。

和 11/18 同路線的解析路徑實跑（`../2026-11-18-left-turn-site/real_run_123302Z`）比較，
兩趟都以同一條理想車道（R800 ±400）計算（`real_run_084014Z/metrics.json`、`onboard_comparison.png`）：

| | 11/24 breadcrumb F1 | 11/18 analytic F1 | 11/24 breadcrumb F2 | 11/18 analytic F2 |
|---|---|---|---|---|
| 終點（車上） | (1000, 797) | (1002, 799) | (1787, 803) | (1801, 808) |
| 終點航向 | **91.5°** | 96.2° | **86.4°** | 97.1° |
| 對理想車道橫向 中位／最大 | 5.0／**13.9** mm | 6.4／30.1 mm | 13.6／**32.5** mm | 8.4／22.3 mm |
| 控制器進度誤差範圍 | −14～+24 mm | −15～+34 mm | −9～+13 mm | −19～+11 mm |
| 離 Leader 最近（MPU） | 475 mm | 480 mm | 543 mm | 555 mm |
| 單輪命令 | 3–51 | 6–48 | 22–69 | 12–65 |

Leader 自己兩趟偏離計畫都 ≤ 4.6 mm，所以兩種路徑來源的車道幾何本來就接近；這趟主要驗證
「Follower 只靠記錄的 Leader 軌跡也能完成同一個編隊」。

觀察：
1. **可行。** 只靠 Leader 廣播的位姿重建車道，三車完成左轉、沒有安全停止；終點與設計點差 ≤ 13 mm（車上估計），
   終點航向比 11/18 更接近 90°（11/18 多轉 6–7°）。
2. **F1 變好、F2 變差。** F1 橫向最大 30 → 14 mm；F2 中位 8 → 14 mm、最大 22 → 33 mm，
   且呈約 14 s 週期（約 0.5 m）擺動，在彎道中振幅變大。
3. **曲率估計比模擬雜很多**（右下圖）。在直線上也有 ±0.5 /m 的起伏，彎道中 F1 的車道曲率在 1.2–5.5 /m 之間擺
   （真值 2.5）。Leader 位置雖然只偏離計畫 ≤ 5 mm，航向卻有 ±2° 的來回修正；breadcrumb 把這個擺動原樣當成路徑曲率，
   F1 內側車道（d=+400）再放大約 2 倍。模擬只放了白雜訊，沒有這種低頻擺動，所以沒看到。
   F2 擺動的週期與曲率起伏相近，推測曲率前饋是主因之一；這一點還沒有逐項拆開驗證。
4. 下一步可試：平滑視窗從 160 加大到約 400 mm，或改用位置擬合曲率；先用這趟的 Leader CSV 離線重播比較，再決定是否再跑。

限制：全部是車上里程＋MPU 估計，不是地面真值。影片見下節。

## 誤差分析（2026-11-24 實跑後）

### 影片 `IMG_8242.MOV`

`~/Downloads/IMG_8242.MOV`（208,469,568 B，SHA-256 前綴 `5e8a4a715a4813ff`，不進 Git）：4K 30 fps、64.87 s，
建立時間 08:40:14Z，與 Leader 64.86 s 的行走時間幾乎一致，所以影片時間 ≈ Leader 經過時間。
抽幀：`real_run_084014Z/video_contact_sheet.jpg`（每 5 s）。

- 手持、從終點方向斜拍，中途有平移與變焦；**不是俯視，不能量公分級誤差**。
- 隊形與 CSV 一致：直線段三車平行前進，Leader 沿黃色中線；進彎後依序左轉，
  終點三車朝同一方向排開。全程間距清楚、沒有接觸，看不出明顯蛇行。
- 曾試著用 F2 自己的車寬當比例尺、量直線段 F2 到黃線的距離。手持鏡頭加上逐格目測，
  每點約 ±15 mm，和車上紀錄的 ±15–25 mm 擺動同一量級，**影片無法證實或否定這個擺動**。

### 誤差拆解

車上的 Follower 橫向誤差 = （參考車道本身的誤差）＋（Follower 追自己參考車道的誤差）。

1. **參考車道（breadcrumb）本身：只占約 3–4 mm。** `replay_leader_csv.py` 把這趟 Leader 實際廣播的位姿，
   原封不動餵給模擬 Follower，比較各路徑來源（`replay_summary.json`、`replay_reference_error.png`）：

   | 路徑來源（同一份 11/24 Leader 資料） | 理想 Follower 橫向 最大 F1／F2 | 輪速延遲 0.4 s 時 |
   |---|---|---|
   | analytic | 0.5／0.8 mm | 1.5／1.5 mm |
   | breadcrumb 航向擬合 160 mm（本趟實際） | 3.0／2.8 mm | 3.5／4.0 mm |
   | breadcrumb 航向擬合 400 mm | 4.8／3.2 mm | 4.4／2.6 mm |
   | breadcrumb 位置擬合 300 mm | 2.8／2.5 mm | 2.8／2.8 mm |
   | breadcrumb 位置擬合 400 mm | 3.7／2.9 mm | 3.3／2.9 mm |

   用 11/18 Leader 的資料重播，結果同一量級（breadcrumb 3–5 mm）。曲率估計雖然很雜（F1 車道 1.2–5.5 /m，真值 2.5），
   回授能吸收，所以換平滑視窗或改位置擬合，閉迴路只差 1–2 mm。**本趟實車 F2 的 33 mm 不是 breadcrumb 造成的。**
2. **曲率為什麼雜：Leader 真的在小幅擺動。** Leader 自己的航向誤差有約 12–13 s 週期、±2° 的擺動，但位置只偏 ±2 mm
   （11/18 也一樣：週期 13.1 s、±2.2°）。曲率對這種短波長的小位移非常敏感（1 mm 的 S 形就能讓曲率差 ±40%），
   所以無論用航向或位置擬合都會看到；位置擬合只把最尖的峰削掉（F1 曲率 RMS 誤差 0.96 → 0.62 /m）。
3. **主要誤差來源：每台車自己的慢速擺動（極限環）。** 實車 Follower 的橫向誤差在兩趟、兩種路徑來源下都以
   14–17 s 週期來回（11/24 F1 15.7、F2 14.2 s；11/18 F1 16.4、F2 16.8 s），Leader 也有 12–13 s 的同類擺動。
   兩台 Follower 的輪速整體追蹤都正常（實測／目標 0.97–1.03），但實測轉速與命令轉速的差異 RMS
   （0.06–0.08 rad/s）和命令本身一樣大；Nano 輪速迴路在左右差速上的誤差（約 70–80 TPS 標準差）是擾動來源。
   這屬於車體＋控制增益的特性，和路徑來源無關；模擬裡的理想車看不到。
4. **兩趟之間的差異屬於趟與趟的變化。** F1 30 → 14 mm、F2 22 → 33 mm 方向相反，各只有一趟，
   不能據此說 breadcrumb 讓 F2 變差。

### 結論

- 這套 breadcrumb 的參考車道與「開天眼」的解析路徑相比，多出的誤差只有幾 mm；Follower 在 400 mm 車道上
  ±15–30 mm 的擺動是車子本身的追蹤擺動，解析路徑版本也有。影片看到的隊形整齊、沒有明顯蛇行，和這一點相符，
  但影片精度不足以量出這幾公分。
- 要再降低誤差，應該處理車子本身的擺動（Follower 橫向／航向增益與預視距離、Nano 左右差速迴路），
  而不是換曲率估計方式。`BreadcrumbPath(tangent_source="position")` 已加入並有測試，但**車上預設維持 heading 160**。
- 要量真的地面誤差，需要固定的近俯視相機加上已量好的地面標記。
