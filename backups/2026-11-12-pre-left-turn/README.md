# 2026-11-12 Pi 程式部署前備份

從三台 Pi 的 `~/pi_rover/` 唯讀複製原有程式，保留原檔；三台 Pi 上亦在
`~/pi_rover/backups/2026-11-12-pre-left-turn/` 保留相同的原檔。
這是**Pi 程式部署前快照**，不代表此後的實車版本。Nano 舊 flash／EEPROM
另在 `backups/2026-11-12-pre-mpu-flash/` 完整備份後，才燒錄候選 `.hex`。

| Pi | 原有主要程式 SHA-256 |
|---|---|
| Leader `leader_formation.py` | `e99a79ae06ebd7b373c768ef391f2f6e57489b583671dc93b7487c3acb2bbdde` |
| F1 `follower_formation.py` | `f26c7aafa94f66076d13619c78bebc6dde396e122b3d9dbd91cbec295e68b524` |
| F2 `follower_formation.py` | `2f33f109619fa25017240b8c15a2cc88a7ca5b47e2e296c5a09622997615f4a7` |
| F2 `follower2_formation.py` | `4afe85512689dc74ed0d97611a56850bbb6ff6fbd4ee463f5fa282834aeaacf4` |

三台 Pi 更新後的候選版：Leader `route-controller 0.2.0-dev+left-turn`；
F1/F2 `normal-follow 0.2.0-dev+left-turn`。部署與回退前皆須確認沒有控制程序
運作，並在車體靜止、可斷馬達電源時進行。
