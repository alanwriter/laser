# 三車軌跡圖的座標點與車體外框

## 車身座標

使用者定義 CSV 的 `(x_mm, y_mm)` 圖上座標點為 **MPU6050 位置**。所有三台車
均使用同一個 250 mm 長、150 mm 寬的矩形外框；MPU 位於車頭後方 100 mm。
目前假設 MPU 在車身寬度的中線；若實際有左右偏移，外框需再依量測修正。
以車頭方向為局部 `+x`、車身左側為局部 `+y`：

- 前緣：MPU 前方 `+100 mm`
- 後緣：MPU 後方 `−150 mm`
- 左／右側：`±75 mm`
- 車體幾何中心：MPU 後方 `25 mm`

對任一角點局部座標 `(u,v)`，CSV 航向 `θ` 下的世界座標為
`(x + u cosθ − v sinθ, y + u sinθ + v cosθ)`。**CSV 點不平移**；外框圍繞
該 MPU 點旋轉，反映物理車長的不對稱位置。此圖層不改實車定位或控制律。

## 圖例與線條

- 實線：車上里程計；虛線：理想路徑或控制器當下目標。Leader 深灰、F1 藍、
  F2 橘，三車各自顏色一致。
- 空心圓加小實心圓：MPU 座標點。半透明填色＋細虛線矩形：250 × 150 mm
  車體範圍；實線前緣與短箭頭：車頭方向。
- 整段軌跡只在起點、中段與終點放稀疏外框，終點較醒目；影片對時圖則每個
  時刻各放一個外框，避免把每一筆資料都畫成車身造成遮擋。
- 座標軸單位為 mm，`x`／`y` 使用等比例。外框只是依 CSV 姿態推算的示意，
  **不是**從相機影像測得的實際車身位置或碰撞證據。

做法參考 Berkeley 的 IROS 2023 論文 Figure 6：將歷史路徑、目標與機器人
矩形分層標示；Zhen Zhang 等人在 IEEE/ASME Transactions on Mechatronics 2021 的論文
Figure 5 則說明矩形外框可呈現車身航向，圓形／單點會丟失這項資訊。
半透明填色、稀疏快照與 MPU 圓點是本專案為了降低遮擋所作的視覺設計。

- Berkeley: https://hybrid-robotics.berkeley.edu/publications/IROS2023_NMPC_DCBF_Duality_NarrowSpace_Walking.pdf
- Zhejiang University: https://april.zju.edu.cn/wp-content/papercite-data/pdf/zhang2021emp.pdf
