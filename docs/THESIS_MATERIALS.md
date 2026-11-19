# 碩士論文資料夾索引

使用者的論文資料放在本機 **`~/Downloads/碩士論文/`**（約 2.5 GB，含影片、簡報與文獻 PDF），**不進 Git**。
本頁只是索引：記錄裡面有什麼、哪些跟目前工作有關、讀的順序。資料夾內容以本機為準；這裡寫的是
2026-11-19 盤點時的狀態。

- Claude Code：`.claude/settings.local.json` 已把這個資料夾加進 `additionalDirectories`（本機設定，不進 Git）。
- 其他工具（Codex、Copilot）：直接讀本機路徑；若讀不到，請使用者提供檔案。
- 影片和文獻 PDF 有版權或檔案太大，**不要複製進 repo**；需要的話只引用路徑和 SHA-256。

## 先讀哪些

| 順序 | 檔案 | 為什麼要讀 |
|---|---|---|
| 1 | `會議簡報/20260826.pptx` | 8/26 報告：Leader–Follower 架構、Balch & Arkin 行為式跟隨、Liang 2023 的「發現目標 → 蒐集 F1/F2 狀態 → 選代價低者 → 派遣 → 導航／雷射 → 回報 → 回隊」，代價＝路徑距離＋危險程度；雷射雲台數學（homography、H⁻¹ 粗定位、Jacobian 修正、最大步長 2°）；**雷射模組已安裝在跟隨載具上的照片** |
| 2 | `會議簡報/20260907meeting.pptx` | 9/7 報告（實際使用的版本）：方形路徑；**兩週後實機驗收＝開放空間的編隊跟隨**（載具間通訊、路徑設計、結果可量化：落點誤差、校正方法、成功率），先從方形開始；另有「數位開關」頁 |
| 3 | `_E8_AB_96…_E9_99_B3_E7_A7_89_E8_BC_BF.pdf.pdf` | 同實驗室前一位研究者的碩論定稿（2025 年 7 月，同一位指導教授）：《水下載具群編隊系統之設計、驗證與實現》，Leader–Follower＋人工勢場＋結構吸引力＋模糊調參，慣性導航＋航位推算、RF 通訊，99 頁。**這是口試委員熟悉的前作，也可當章節架構範本**（緒論／文獻／系統設計：編隊演算法、定位、系統／實驗：含迴轉編隊／結論） |
| 4 | `參考文獻/全部參考文獻_便利貼.txt` | 27 篇文獻的一句話、問題、價值、限制；優先 #17 Balch & Arkin 1998、#27 Liang et al. 2023。Notion 有同內容的頁面 |
| 5 | `會議記錄/8 21 訂定題目 with 老師 喆恩 ….md` | 8/21 定題會議（Notion 匯出，與 Notion 同頁） |

## 其餘內容

| 類別 | 檔案 | 說明 |
|---|---|---|
| 文獻 PDF | `參考文獻/*.pdf`（約 27 篇，檔名即完整引用） | 編隊控制綜述、虛擬結構、行為式、VIO／MSCKF、UGV／AUV 編隊、車隊 platooning 等 |
| 文獻 PDF（根目錄） | Alonso-Mora 2017（動態環境多機器人編隊與物體搬運）、*Distributed Multi-Robot Formation Splitting and Merging in Dynamic Environments*、*Formation Reconfiguration Control With Collision Avoidance of Nonholonomic Mobile Robots* | 編隊分裂／合併與重構，和「Follower 脫隊執行任務再歸隊」直接相關 |
| 前作研究資料 | `Z. 研究資料 - *.zip`（約 61 MB） | 前一位研究者的 Arduino Leader/Follower RF 與感測程式、Python 繪圖 notebook、0°/30°/45° 編隊路徑資料與圖、論文 docx 與簡報 |
| 雷射雲台程式 | `laser_pointer_demo/` | `vision_pointer.py` 與 repo 根目錄的版本**完全相同**；另有 Nano 伺服韌體 `arduino_nano/arduino_nano.ino`、架構圖 |
| 簡報草稿 | `.codex-finalizer/20260907meeting_下兩週驗收項目_candidate.pptx` | 9/7 的另一版（未採用）：驗收改成「固定車位、固定標靶下 Follower 雷射命中與迭代校正」，並請老師確認標靶尺寸、容許誤差、最大校正次數 |
| 實驗影片 | `IMG_8017.MOV`、`IMG_8103.MOV`（10/29 第三趟）、`IMG_8143.MOV`、`F2demo.MOV`、`L1demo.MOV`、`20261116_2米直線定位測試.MP4`、`20261116_ALAN0527.MP4` | 各趟分析見 `analysis/` 對應日期資料夾 |
| 圖片 | `leader-f1-relative-trajectory.jpg`、`S__23085058.jpg` | — |

## 注意

- `~$異質環境下編隊載具協同架構之探討_研究初稿.docx` 只是 Word 的暫存鎖定檔（162 bytes），**研究初稿本體不在這個資料夾，2026-11-19 全機搜尋也找不到**。暫定論文題目是《異質環境下編隊載具協同架構之探討》。
- `.tmp_laser_ppt/` 是產生簡報時的暫存檔（含 `node_modules`），不是研究資料。
