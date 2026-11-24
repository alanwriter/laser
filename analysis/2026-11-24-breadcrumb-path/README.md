# 2026-11-24 軌跡重建路徑（breadcrumb）離線驗證：路徑 D

**狀態：只有離線模擬與單元測試，尚未部署到 Pi，也沒有實車行走。**

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
