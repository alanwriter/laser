# 最近三次三車左轉的控制器誤差（2026-11-24 分析）

**前提（依使用者要求）：車上 encoder＋MPU 位姿視為完全準確。** 因此以下全是「控制器誤差」，也就是每台車離它自己
控制器正在追的參考有多遠，不是地面真值誤差。

| 代號 | 時間（UTC） | 路線 | Leader 速度 | Follower 路徑來源 | 資料 |
|---|---|---|---|---|---|
| B | 2026-11-18 12:23:41Z | 直 600 → R800 → 直 200 | 30 mm/s | 解析路徑（事先告知） | `../2026-11-18-left-turn-site/real_run_122341Z/` |
| D | 2026-11-18 12:33:02Z | 直 600 → R800 → 直 400 | 38 mm/s | 解析路徑 | `../2026-11-18-left-turn-site/real_run_123302Z/` |
| D' | 2026-11-24 08:40:14Z | 同 D | 38 mm/s | breadcrumb（視窗 160 mm） | `../2026-11-24-breadcrumb-path/real_run_084014Z/` |

產生：`python3 analyze_turns.py`（→ `metrics.json`、`series.json`），`python3 plot_figures.py`（→ 兩張圖）。

## 誤差定義（`fig1_error_definitions.png`）

在每台車參考車道的路徑座標（Serret–Frenet）裡量：

- **e_y 橫向誤差**：車到車道上最近點的距離（左正）。
- **e_s 前後間距誤差**：目標點（Leader 進度 − 400 mm）與車在車道上位置的差，沿車道量。
- **e_ψ 航向誤差**：車道切線方向 − 車頭方向。

路段依 Follower 在路線上的位置劃分：直線（含起步）、進彎（彎道前 300 mm）、穩定彎道、出彎。
B 的 Follower 停在彎道中，沒有出彎段；D／D' 的出彎段只有幾毫米，不列入。

## 結果（`fig2_controller_errors.png`、`metrics.json`）

| | B 解析 30 | D 解析 38 | D' breadcrumb 38 |
|---|---|---|---|
| Leader e_y RMS／最大 | 1.9／4.2 mm | 2.4／4.6 mm | 2.0／4.6 mm |
| F1／F2 e_y RMS | 4.0／11.9 mm | 11.4／10.3 mm | 7.0／16.3 mm |
| F1／F2 e_y 最大 | 7.4／25.1 mm | 30.1／22.3 mm | 14.3／33.2 mm |
| F1／F2 e_s 最大 | 55.0／17.9 mm | 33.7／18.9 mm | 23.6／12.5 mm |
| F1／F2 e_ψ RMS | 3.4／6.8° | 7.8／4.5° | 5.7／8.2° |
| 擺動週期 L／F1／F2 | 15／35／18 s | 13／16／16 s | 13／16／16 s |
| 轉向偏差 RMS ÷ 轉向命令 RMS（F1／F2） | 0.97／1.06 | 1.04／0.92 | 0.91／1.08 |
| 命令觸及限幅（輪速步階或轉向上限） | ≤ 0.2% | ≤ 0.3% | 0% |
| 控制週期 中位／p95 | 0.112／0.122 s | 同左 | 同左 |
| Leader 封包延遲 中位／最大 | 21／152 ms | 21／156 ms | 23／171 ms |
| 離 Leader 最近（MPU） F1／F2 | 478／557 mm | 480／555 mm | 475／543 mm |

各路段 e_y RMS（F1／F2，mm）：

| | 直線 | 進彎 300 mm | 穩定彎道 |
|---|---|---|---|
| B | 4.0／9.6 | 3.3／14.5 | 4.2／13.1 |
| D | 16.4／7.9 | 6.7／10.1 | 5.5／12.2 |
| D' | 7.5／13.5 | 7.0／23.6 | 6.6／16.4 |

### 讀法

1. **誤差從 Leader 到 Follower 放大 2–8 倍。** Leader 自己只有 e_y RMS 約 2 mm，Follower 是 4–16 mm、最大 7–33 mm。
2. **直線並不比彎道小。** D 的 F1 在直線 RMS 16 mm、穩定彎道只有 5.5 mm；B／D 的 F2 直線與彎道同一量級。
   如果誤差主要來自曲率，直線應該明顯較小。所以主因不是「轉彎追不上」，而是一直存在的擺動。
3. **擺動週期很固定：約 13–16 s，三台車、三趟都一樣**（B 的 F1 振幅很小，頻譜峰值不明顯，算出 35 s）。
   以 30–38 mm/s 換算是 0.4–0.6 m 的波長。這是車體＋控制迴路本身的極限環，與路徑來源無關。
4. **轉向命令幾乎沒有被忠實執行。** 實際轉向速率與命令的差，RMS 和命令本身一樣大（比值 0.91–1.08）。
   但命令幾乎沒碰到限幅（≤ 0.3%），所以不是「力氣不夠」，而是 Nano 左右輪速迴路對差速命令的響應有大誤差／延遲。
   這是目前最主要的擾動來源，也是擺動的可能成因。
5. **前後間距誤差在進彎時最大，而且是內側的 F1。** F1 的 e_s 最大值 24–55 mm 都出現在進彎／彎道中：
   內側車道進彎時速度要突然降到 0.5 倍（1 − 400/800），參考速度是階躍變化。外側 F2 是升到 1.5 倍，e_s 較小。
   B（30 mm/s）的 F1 內輪只要 15 mm/s，落在前饋外插區，e_s 最大（55 mm）。
6. **通訊不是問題。** 封包延遲中位 21–23 ms、最大 0.17 s（逾時門檻 0.35 s）；控制週期實際 0.112 s（設計 0.10 s）。
7. **D vs D'（解析 vs breadcrumb）結果相反**（F1 較好、F2 較差），各只有一趟，屬於趟與趟的變化。
   B 的速度與出彎長度不同，不能和 D／D' 直接比較。

## 頂級期刊審稿會看什麼，以及本資料的狀態

搜尋了期刊論文與預印本的實驗評估方式（來源列在文末）。以 IEEE T-RO／TCST／TIE／RA-L 這類期刊的慣例整理：

| 項目 | 期刊常見做法 | 本研究目前 |
|---|---|---|
| 誤差定義 | 在路徑（Frenet）座標裡分成橫向、沿路徑、航向三項 | ✓ 已照做（圖 1） |
| 統計量 | RMS 與最大值都要報；兩者排名可能不同 | ✓ |
| 暫態 vs 穩態 | 依曲率／路段分開報，標出曲率變化處的峰值 | ✓ 分路段；尚未做曲率掃描 |
| 控制量／平順度 | IAE、ISE、平均控制量、jerk、命令變化 | △ 有命令變化與限幅比例，尚未報 IAE／jerk |
| **獨立地面真值** | 動作捕捉（OptiTrack／Vicon）等外部量測，並說明其誤差 | ✗ 目前假設車上定位正確；影片只能看隊形 |
| **重複試驗** | 每個條件多趟，報平均 ± 標準差或信賴區間 | ✗ 每個條件 1 趟 |
| 誤差傳遞（串穩定性） | 擾動沿隊伍是否被放大，常用頻域增益或峰值比 | △ 只有 Leader→Follower 放大倍數 |
| 通訊延遲／掉包 | 實測延遲分布，並做延遲或掉包的壓力測試 | △ 已量延遲；未做壓力測試 |
| 公平比較 | 同路線、同速度、同條件只換一個因素 | △ 只有 D vs D' 符合；B 速度與出彎不同 |
| 與理論對照 | 模擬或理論預測和實驗放在一起 | △ 有理想模擬與重播，尚未比對擺動頻率 |

要投期刊，最優先補的是：**(1) 外部地面真值**（固定俯視相機＋已量地面標記，或動作捕捉）、
**(2) 每個條件至少 3–5 趟**、**(3) 用轉向響應實驗解釋 13–16 s 的擺動**（例如量 Nano 差速階躍響應，
再用模型預測擺動週期並和實驗對照）。

## 限制

- 「定位完全準確」是使用者指定的假設；實際上車上里程計與地面之間還有未量的誤差。
- 每個條件只有 1 趟，F1／F2 是兩條不同車道，不是重複試驗。
- 擺動週期用 0.1 s 重取樣後的 FFT 峰值，在 3–40 s 之間找；振幅小時峰值不穩（B 的 F1）。
- 轉向實測速率由 MPU 航向差分，每 0.11 s 一筆，含量化雜訊。

## 來源（搜尋結果，多為預印本與其他期刊；未直接取得 T-RO 全文）

- Frenet 座標誤差定義：[Robust MPC for tracking of nonholonomic robots (arXiv 1703.03101)](https://arxiv.org/pdf/1703.03101)；
  [Kanayama et al., A stable tracking control method](https://calhoun.nps.edu/bitstream/handle/10945/40155/Kanamaya_A_stable_tracking_control.pdf?sequence=1)
- RMS 與最大值、曲率相關的峰值：[LPV-based NMPC for trajectory tracking (arXiv 2506.04684)](https://arxiv.org/pdf/2506.04684)；
  [Lateral control for autonomous vehicles: a comparative evaluation (arXiv 2311.07987)](https://arxiv.org/pdf/2311.07987)；
  [Building a winning self-driving car in six months (arXiv 1811.01273)](https://arxiv.org/pdf/1811.01273)；
  [Along Similar Lines (arXiv 2211.02047)](https://arxiv.org/pdf/2211.02047)
- 控制量與平順度：[Finite-time control for wheeled mobile robots with experimental validation (arXiv 2512.20229)](https://arxiv.org/pdf/2512.20229)；
  [Benchmarking local motion planners (arXiv 2211.01812)](https://arxiv.org/pdf/2211.01812)
- 動作捕捉地面真值：[Distributed geometric optimization and control of rigid differential-drive formations (arXiv 2611.07163)](https://arxiv.org/pdf/2611.07163)；
  [Optimization based solutions for control and state estimation in non-holonomic mobile robots (arXiv 1803.06928)](https://arxiv.org/pdf/1803.06928)；
  [Leader–follower multiple mobile robots, experimental verification (MDPI Applied Sciences 2022)](https://www.mdpi.com/2076-3417/12/14/7273)；
  [Collaborative trolley transportation (arXiv 2303.06624)](https://arxiv.org/pdf/2303.06624)
- 串穩定性與誤差傳遞：[Seiler, Pant, Hedrick, IEEE TAC 2004](https://dept.aem.umn.edu/~SeilerControl/Papers/2004/SeilerEtAl_04TAC_DisturbancePropagationInStrings.pdf)；
  [Are commercially implemented ACC systems string stable? (arXiv 1905.02108)](https://arxiv.org/pdf/1905.02108)；
  [Delayed spacing policies in vehicle platooning (arXiv 2508.14587)](https://arxiv.org/html/2508.14587v1)；
  [Distributed protocol with in-vehicle tests (IEEE TCST)](https://iris.unimore.it/retrieve/e31e1250-1a70-987f-e053-3705fe0a095a/TCST_platooning.pdf)
- 重複試驗與通訊：[Decentralized asynchronous perception-action-communication loops (arXiv 2309.10164)](https://arxiv.org/pdf/2309.10164)；
  [Hold-and-hit formation navigation under delay and loss (arXiv 2510.02624)](https://arxiv.org/pdf/2510.02624)；
  [Formation control over packet dropping links (arXiv 1506.08244)](https://arxiv.org/pdf/1506.08244)
