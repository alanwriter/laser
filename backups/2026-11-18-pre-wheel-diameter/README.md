# 每車 2 m 輪徑韌體燒錄前的 flash 備份（2026-11-18）

使用者確認三台馬達斷電（「好了」）後，依序 Leader → F1 → F2：avrdude 讀回 flash → 寫入
`firmware/builds/2026-11-18/{leader,f1,f2}_ff_d2m.hex` 並驗證 → 唯讀 HELLO／CONFIG／IMU／STATUS。

| 車 | 讀回與燒錄前映像比對 | 寫入 | CONFIG（ticks 左/右、輪徑、輪距、MPU） | 其他 |
|---|---|---|---|---|
| Leader | 與 `2026-11-17/leader_velocity_feedforward_candidate.hex` 29,424 位元組一致 | 29,424 B 驗證通過 | 1216.4/1222.4、**74.6**、130、1.00 | IMU present 位址 104；idle、fault 0；STOP ACK |
| F1 | 與 `2026-11-18/f1_velocity_feedforward_candidate.hex` 29,310 位元組一致 | 29,310 B 驗證通過 | 1203.5/1200.0、**74.3**、130、1.00 | 同上 |
| F2 | 與 `2026-11-18/f2_velocity_feedforward_candidate.hex` 29,362 位元組一致 | 29,362 B 驗證通過 | 1216.2/1237.4、**75.1**、130、1.00 | 同上 |

（CONFIG 輪徑只印一位小數；實際值 74.64／74.30／75.05。）回退：燒回上表的燒錄前映像。尚未行走驗證。
