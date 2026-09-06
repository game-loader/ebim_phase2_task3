# TMR：连续门框对齐与定距前进

字母投放扩展（当前仅保存在本地开发树）：

- `scripts/14_letter_guided_search.py`：白卡字母识别、near/far 分类、最多
  2.40 m 向右搜索和自适应图像居中；
- `scripts/15_return_from_letter.py`：按搜索实测距离等距向左、逆时针
  180°，随后复用门中线/0.50 m/1.20 m 流程；
- `scripts/14_prepare_letter_vision.sh` 只在部署时建立轻量 OpenCV 环境，
  实时任务使用 `14_run_letter_guided_search.sh`，不在比赛期间安装软件。

## 新的连续“起点到抓取位”流程

`scripts/07_start_to_pickup.py` 把当前确认的底盘路线做成一个常驻进程：

1. 里程计闭环前进 `0.85 m`；
2. 里程计闭环顺时针转 `90°`；
3. 从**本次运行的 `/map` OccupancyGrid** 找门框候选，再由前后双雷达多帧确认；
4. 先横移到门洞中线，并保持在门框中垂线上；
5. 底盘中心先停在门平面前方 `0.50 m`；
6. 沿同一中垂线前进 `1.20 m`，最终停在门后约 `0.70 m`。当前主流程不再边走边搜索桌腿。

示例 PNG 只用于说明“墙体中间的缺口”是什么样，不读取其像素，也不保存某次 SLAM 的绝对地图坐标。门口一旦确认，就把本轮门坐标冻结在里程计局部系中，避免 SLAM 回环时目标跳动。

新流程只发布到独立的 `/tmr_cycle/mission_cmd_vel`。`cmd_vel_adapter.py` 收到后会锁存任务租约，忽略旧 `/tmr_cycle/cmd_vel`、Nav2 和遥控输入；脚本还要求控制器输入端只能有适配器这一个发布者，避免旧 04/05/06 脚本直接抢占。每个控制周期都会检查双雷达/里程计新鲜度、速度所有权、进展和带制动距离的整车扫掠包络；同时利用实时 `map -> base_link` 位姿，把完整矩形车体的短扫掠路径投到 `/map` 上，只有已知占用栅格会追加触发停车。未知栅格既不会单独触发停车，也不能被当成地图已确认安全，实时雷达保护始终保留。进入硬停止区后锁存 `ABORT`，适配器即使在任务进程退出后仍保持零速，不会自动恢复旧目标。

本地只检查配置和流程（不会连接 ROS，也不会产生运动）：

```bash
python3 ~/tmr_cycle/scripts/07_start_to_pickup.py \
  --config ~/tmr_cycle/config/start_to_pickup.yaml
```

比赛时把整个目录放在底盘主机并让 SLAM、双雷达、控制器和新版 `cmd_vel_adapter.py` 常驻。确认手柄已释放、Nav2 没有活动目标，并停止会**直接**发布 `/swerve_drive_controller/cmd_vel` 的 pedal `base_bridge`/旧 04、05、06 进程；控制器输入必须只剩适配器一个发布者。然后一条命令运行完整流程：

```bash
python3 ~/tmr_cycle/scripts/07_start_to_pickup.py \
  --config ~/tmr_cycle/config/start_to_pickup.yaml --execute --disable-collision-guard
```

当前最终比赛入口按现场要求关闭全部底盘碰撞判定，包括平移雷达/地图扫掠、旋转净空、地图冲突和门侧净空判断。`03_start_navigation.sh --run-mission` 与综合任务协调器都会显式传入该参数。里程计目标闭环、传感器时间戳、控制所有权、超时与无进展停止仍然有效。直接运行 `07_start_to_pickup.py --execute` 而不带该参数时，保留原碰撞判定，仅供需要时诊断。

正式运行应在底盘主机本地启动，不把 20 Hz 控制环放在远程 Codex/SSH 链路上；远端只负责发起、停止和查看结构化状态。机械臂若没有收纳在当前 `0.58 m` 宽、前后各 `0.40 m` 的包络内，必须先把 `config/start_to_pickup.yaml` 的 `footprint` 改成整机实测外廓。

任务结束后默认继续锁住底盘零速。确实要交还手柄/Nav2 时，显式释放一次：

```bash
ros2 topic pub --once /tmr_cycle/mission_active std_msgs/msg/Bool "{data: false}"
```

如果脚本报告 `controller input has ... publishers`，说明仍有进程绕过适配器直连控制器；应停止那个发布者，不能扩大允许数量来绕过检查。

## 坐标不再绑定某一张地图

脚本启动时读取当前 `map -> base_link`，把机器人当时的位置和朝向定义为任务坐标系 `mission_start`：

- `+x`：机器人初始正前方；
- `+y`：机器人初始左侧；
- 地图原点、图片像素和每次 SLAM 的原点可以变化；
- 房间级路线只保存相对 `mission_start` 的距离和朝向。

执行时会冻结本轮门框中点和法向，并用同一个局部门框参考系计算门前 `0.50 m` 与随后前进 `1.20 m` 的目标，避免 SLAM 更新造成目标跳动。

因此每次执行前，机器人必须准确回到地面标记的起点和初始朝向。不得在路线中途重启执行器。

## 实际完成方法

1. 每次换设备/场地，启动在线 SLAM 和 Nav2，用手柄重新覆盖需要通行的区域。
2. 建图后回到固定起点，整个任务期间不要停止 SLAM。
3. 执行器自动冻结本次 `mission_start`，先按相对路线到桌边观察点。
4. 机器人静止采集 4 秒双雷达数据，在预期区域内栅格聚合小障碍簇。
5. 通过桌腿直径、两腿间距、成对方向、预期区域和多帧稳定性识别指定的近侧两腿。
6. 用两腿中点和桌边法向实时计算靠近位姿，保持约 `0.55 m` 距离后交给双臂。
7. 出房间，在保持指定朝向时横移并触发桌面感知，最后返回起点循环。

候选不唯一、桌腿偏移超过阈值、横移方向出现障碍或双臂未确认完成时，底盘停车并报错；默认不会用旧坐标盲目靠近。

## 部署

把整个目录复制到 `/home/tmr-user/tmr_cycle`，然后：

```bash
chmod +x ~/tmr_cycle/scripts/*
```

## 每次新场地/新开机

终端 A：

```bash
~/tmr_cycle/scripts/03_start_navigation.sh
```

上面只启动并保持零速，不执行路线。比赛的一条命令完整流程为：

```bash
~/tmr_cycle/scripts/03_start_navigation.sh --run-mission
```

脚本默认使用仅限底盘本机的 `ROS_DOMAIN_ID=97`、`ROS_LOCALHOST_ONLY=1`，避免 `.50` 的 Humble 与 `.100/.101` 的 Jazzy 在 domain 0 互相污染。底盘控制器、双雷达、SLAM、速度适配器和 `07` 任务都由同一进程树启动并继承该环境。跨主机机械臂联动后续应通过明确的桥接接口实现，不能把底盘控制图重新混回 domain 0。

该脚本负责本机底盘/雷达/SLAM 与速度适配器（连续任务不启动 Nav2）；它不会替你停止 `.101` 上单独启动的 `tmr_pedal_teleop base_bridge`。进入新的自主流程前必须停止那个直发进程。不同版本的 `franka_bringup` 若还创建了额外 controller 发布者，`07_start_to_pickup.py` 会保持零速并明确报出发布者数量；以这个运行时图检查为准。

`controller_manager` 只有真实 `list_controllers` RPC 成功返回才算就绪，不再相信 DDS 图中的服务名称。正常 FCI 握手历史上不到一秒；若 `172.16.16.10` 接受 TCP 后 15 秒仍未返回协议握手，脚本会终止底盘进程并明确报告需要检查 TMR 状态，不会用长期 spawner 等待掩盖故障，也不会自动执行 `fault-reset` 或 `switch-on`。内置 spawner 获得完整窗口后才允许一次带同一 `controllers.yaml` 的重试，避免两个 spawner 并发。

只有 `swerve_drive_controller=active`、收到新鲜 odom、前后雷达均有数据后，才会继续启动 SLAM 和零速锁存适配器。退出时按有界的 `INT → TERM → KILL` 顺序清理，避免停止流程永久卡住。环境和每个进程的日志保存在同次 `logs/live_slam_*` 目录。

碰撞判断使用实时双雷达作为硬保护，并以当前机器人在实时 OccupancyGrid 中的位姿做第二层制动扫掠。两层都只判断本次速度将要**新进入**的区域；机器人初始占用体积内的底盘、线缆、机械臂回波和地图残影不再被误报为前方碰撞。速度方向之外不会固定放宽：前进时正前方、横移时对应侧面、后退时后方的障碍仍会立即阻挡。

新版适配器启动时默认锁住零速。手动建图或运行旧 `00_bootstrap_mapping.py` 前，先显式释放一次；自主 `07` 启动后会自行重新取得独占租约：

```bash
ros2 topic pub --once /tmr_cycle/mission_active std_msgs/msg/Bool "{data: false}"
```

建议把新的固定起点向当前方案的右侧移动约 `0.20–0.30 m` 并贴地标。底盘位于新起点、朝向正确且 Xbox 已松开时，先做检查：

```bash
python3 ~/tmr_cycle/scripts/00_bootstrap_mapping.py \
  --config ~/tmr_cycle/config/route.yaml --dry-run
```

确认日志中 TF、雷达和速度适配器均就绪后，人员手持急停，执行初步建图动作：

```bash
python3 ~/tmr_cycle/scripts/00_bootstrap_mapping.py \
  --config ~/tmr_cycle/config/route.yaml
```

默认动作是：相对机器人初始朝向横向左移 `0.25 m`，以约 `10.3°/s` 原地转一圈，恢复原朝向，再横向右移回到同一个起点。左移途中前方净空不足 `0.55 m`，或旋转扫掠半径内净空不足 `0.60 m`，都会停车。可用 `--left-distance 0.15` 临时覆盖左移距离；正式任务不要使用 `--no-return`，否则后续任务原点会改变。

这一圈主要补齐机器人周围以及上方、左侧桌腿；如果房间、门口和横移通道仍有未观测区域，继续用 Xbox 手柄慢速覆盖。最终回到新起点标记并恢复固定初始朝向。保持终端 A 和 SLAM 运行，松开手柄。若要留档，可在另一终端执行：

```bash
~/tmr_cycle/scripts/02_save_map.sh site_check
```

保存的地图只用于查看和复盘，任务坐标不依赖它。

终端 B，先只验证桌腿识别（会到观察点，但不会靠近或调用双臂）：

```bash
python3 ~/tmr_cycle/scripts/execute_cycle.py \
  --config ~/tmr_cycle/config/route.yaml --detect-only --skip-arm
```

然后做一次底盘空跑：

```bash
python3 ~/tmr_cycle/scripts/execute_cycle.py \
  --config ~/tmr_cycle/config/route.yaml --cycles 1 --skip-arm
```

双臂联调完成后循环：

```bash
python3 ~/tmr_cycle/scripts/execute_cycle.py \
  --config ~/tmr_cycle/config/route.yaml --cycles 0
```

调试时只有显式加 `--allow-pickup-fallback` 才会在识别失败后使用估计点；正式运行不要使用。

## 一次性现场标定

当前相对路线和 `0.66 m` 桌腿间距来自 `final_clean_20260828_144206` 与红色标注的估算，必须先低速确认。若路线误差明显，可在同一 SLAM 会话里手动放置机器人并依次记录：

```bash
python3 ~/tmr_cycle/scripts/capture_pose.py ~/tmr_cycle/config/route.yaml p1_start
python3 ~/tmr_cycle/scripts/capture_pose.py ~/tmr_cycle/config/route.yaml table_observe
python3 ~/tmr_cycle/scripts/capture_pose.py ~/tmr_cycle/config/route.yaml room_exit
python3 ~/tmr_cycle/scripts/capture_pose.py ~/tmr_cycle/config/route.yaml inspect_end
```

桌腿相关参数在 `config/route.yaml` 的 `table_detection`：

- `roi`：相对起点的桌腿搜索范围；
- `expected_pair_spacing`：目标两腿中心距；
- `expected_pair_midpoint`：目标腿对的大致中心；
- `approach_standoff`：底盘中心到腿对中点的停止距离。

先用卷尺确认腿中心距，再在 `--detect-only` 模式观察日志。只扩大与实测差异对应的阈值，不要一次把搜索区域和全部容差放得很大。

## 接口

- 雷达：`/navigation/scan` (`sensor_msgs/LaserScan`)
- 动态腿簇：`/tmr_cycle/detected_legs` (`geometry_msgs/PoseArray`)
- 动态靠近位姿：`/tmr_cycle/detected_pickup_pose` (`geometry_msgs/PoseStamped`)
- 语义 TF：`mission_start`、`detected_table_edge`
- 双臂命令：`/tmr_cycle/arm_command` (`std_msgs/String`)
- 双臂完成：`/tmr_cycle/arm_done` (`std_msgs/Bool`)
- 桌面视觉：`/tmr_cycle/perception_command` (`std_msgs/String`)

实机测试必须低速进行，人员手持急停。当前底盘曾出现 `communication_constraints_violation`，在通信/控制器稳定前不要进行自主移动。

## ZED 字母搜索通信隔离

`14_run_letter_guided_search.sh` 默认把 ZED 放在独立的视觉域 1，由
`zed_frame_export.py` 原子更新 `/tmp/tmr_zed_latest.jpg`；字母搜索器在底盘
控制域中读取该文件，因此不会在控制 DDS 域订阅高带宽压缩图像。ZED 冷启动
最多等待 65 秒，避免旧版 10 秒窗口造成误判离线。

正常比赛仍通过 `cmd_vel_adapter` 独占发布。`--direct-controller` 仅用于适配器
故障恢复；使用前必须停止适配器，确保 `swerve_drive_controller/cmd_vel` 只有
一个发布者。底盘 bring-up 若自动启动 `teleop_twist_joy_node`，自主门框流程会
拒绝运行，需先停止该直发节点。
