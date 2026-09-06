# 新增：15 结束后的放物往返路线

这是 2026-09-06 新增的独立 `20` 阶段，不是原有路线。只接入底盘移动，
不调用机械臂策略。已完成代码和离线测试；尚未实机验证。

桌腿 ROI 的参考坐标系是 **原预览图的绿色 START**：起始前方 +x、左方 +y。
它不会跟随 15 的终点重新置零。ROI 固定为 x=[1.65,3.65]、y=[-0.10,0.60] m。

```text
15 返回取物侧并停车 A
  → 左移 0.85 m 到 B
  → 双雷达多帧检测 ROI 内两条桌腿，锁定 x 最大的那条
  → 沿绿色 START +x 前进，底座最前沿与远端桌腿 x 对齐，到 C
  → 顺时针转 90°
  → WAITING_FOR_PLACEMENT：停车并保存状态，等待外部放物

人工完成放物、单独发起 return
  → 逆时针恢复转向前的朝向
  → 沿途经位置 C → B → A 返回
  → 校验回到 15 的停车位
  → 运行原有 16（也可以 --stop-at-pickup 只停在 A）
```

20 不使用碰撞检测，继承 13 的里程计闭环。返程按保存的**实际途经位置**
计算目标，不盲目重复标称距离。运动期间保留里程计新鲜度、位姿跳变、
独占速度适配器、超时、无进展检查。失败不会自动重放运动阶段。
到位后发布零速并确认静止；退出后由现有 cmd_vel_adapter 保持任务零速锁定。

## 出发前：记录绿色 START

先启动本轮底盘驱动、里程计、双雷达和既有速度适配器。在机器人底座中心位于
预览图绿色 START、朝向与预览图起始 +x 一致时，记录原点。此命令**只订阅
里程计，不创建速度发布器、不移动机器人、不自动启动驱动**。

```bash
cd ~/tmr_cycle
route_run="state/run_$(date +%Y%m%d_%H%M%S)"
bash scripts/20_run_after_return_placement.sh capture-origin \
  --capture-at-green-start --state-file "$route_run/green_start.json"
```

`--capture-at-green-start` 表示操作者确认实际位置和朝向对应图中绿色 START；
软件只能验证底盘静止，无法从预览图片自行确认物理起点。命令保存本轮原始
里程计位姿、坐标系和底盘驱动进程指纹。必须在出发之前记录并沿用同一里程计
会话；驱动重启、里程计重置后，需回到绿色 START 新建本轮记录。
文件已存在时拒绝覆盖。若已经完成 15 但没有记录，不能在 15 终点补采冒充 START。

## 15 已经完成：单独执行 20

在 `.50` 的 `~/tmr_cycle` 下操作，沿用之前终端中的 `route_run`（本轮状态目录）。
如果换了终端，先把它设为本轮实际目录。目录里的 `return.json` 必须是本轮 15
的成功结果，底盘驱动/里程计和 cmd_vel_adapter 应保持运行。不要在保存状态后
重启里程计或遥操移动底盘；起点/返程位姿校验不通过时会拒绝移动。

预览，不连接 ROS 或移动底盘：

```bash
cd ~/tmr_cycle
python3 scripts/20_after_return_placement.py outbound
```

去放物位，只停车等策略：

```bash
bash scripts/20_run_after_return_placement.sh outbound \
  --after15-state "${route_run:?请设置本轮状态目录}/return.json" \
  --roi-origin-file "$route_run/green_start.json" \
  --state-file "$route_run/placement.json" --execute
```

完成状态为 `WAITING_FOR_PLACEMENT`，`placement_completed=false`。
记录包括 `origin`、`after_left`、`before_turn`、`placement_pose`，单位 m/rad。
这里 `origin` 是返程使用的 15 停车位；`roi_origin` 单独保存绿色 START。
`leg_approach` 保存选中的近/远腿、实际前进距离和转向前的前沿对齐误差。
不会执行放物策略，也不会自动返程。

你执行完放物策略后，单独返程并接原有 16：

```bash
bash scripts/20_run_after_return_placement.sh return \
  --state-file "${route_run:?请设置本轮状态目录}/placement.json" \
  --placement-complete --execute
```

若本次只想返回 15 的终点，在以上命令追加 `--stop-at-pickup`。
之后仍可用同一个 return 命令、不加此参数，单独衔接 16，不会重复走放物往返段。
`--placement-complete` 表示你已完成外部放物操作；软件不会检测杯子是否放下。

## 15 尚未执行：一次衔接到新增放物停车位

在原来的 15 命令中追加：

```text
--with-placement-detour --placement-state-file "$route_run/placement.json" \
--placement-roi-origin-file "$route_run/green_start.json"
```

例如（`return_left_m` 仍取自本轮字母搜索的 `actual_right_m`）：

```bash
bash scripts/15_run_return_from_letter.sh \
  --execute --left-m "${return_left_m:?请读取本轮字母搜索结果}" \
  --disable-collision-guard \
  --state-file "${route_run:?请设置本轮状态目录}/return.json" \
  --with-placement-detour --placement-state-file "$route_run/placement.json" \
  --placement-roi-origin-file "$route_run/green_start.json"
```

不带新参数时，15 的原有行为保持不变。新参数只在 15 确认 COMPLETE、
释放原任务锁后启动 20；20 再核对当前位姿与 15 的最终里程计记录。
如果不指定 `--placement-state-file`，默认使用 15 状态文件同目录的
`<原文件名去掉.json>_placement.json`。已有记录时拒绝重复执行，不覆盖旧任务。

## 原有 16 的限制

16 仍然执行其既有固定动作：后退 1.20 m、逆时针转 90°、后退 0.85 m。
它没有保存或逆转 07 去程中动态计算的全部位移。因此新增 20 的原路返回可以
核对回到 15 的停车位，但不能据此承诺 16 精确回到最初起点。
20 将此结果标记为 `LEGACY_RETURN_COMPLETE`，并记录
`original_start_pose_verified=false`，不将固定动作完成冒充原始起点定位确认。

## 实时桌腿检测与停车判据

20 订阅 `/lidar_front/scan`、`/lidar_rear/scan`，复用旧脚本的完整雷达外参投影。
优先查 TF；只有已测量的 `lidar_front`/`lidar_rear` 帧允许使用已有固定外参。
按每帧扫描时间插值 `/swerve_drive_controller/odom`，再变换到保存的绿色 START；
没有覆盖扫描时刻的里程计就丢弃该帧，不能用最新位姿代替。没有逐束运动补偿。

复用 `table_leg_detection.cluster_points`：2.5 cm 栅格、6 cm 聚类连接距离、
每格至少 3 点、每簇至少 10 点、腿簇最大直径 20 cm。采用 1 秒滚动窗口，
至少 6 个独立扫描帧、每雷达至少 2 帧，每条腿至少 3 帧支持；两路雷达和每条腿
都需在 0.30 秒内有新证据。要求恰好两条稳定腿，x 至少相差 0.20 m，连续
3 次结果稳定后锁定 x 最大的腿。不沿用旧横向腿对的 0.66 m 间距/90° 方向筛选。

底座模型确认前沿距 `base_link` 中心 0.40 m、半宽 0.29 m；朝向完全对齐时，
停车判据是 `远腿.x - (底座.x + 0.40)` 进入 ±0.02 m。微小偏航时计算前沿
最靠前角点在 START +x 上的投影。这里的腿位置是激光聚类中心，2 cm 是软件
停止阈值，不是实测精度保证。没有旧的 0.55 m 停靠间距，也没有固定 1.20 m 回退。

前进速度最高 0.05 m/s，最多前进 3 m、超时 90 秒，沿左移结束的 y 位置前进并
保持朝向。初始朝向偏离绿色 START +x 超过 10° 拒绝执行，原点/双雷达校验先于
左移。前进期间缺腿、多腿、证据过期、目标跳变均发零速重新确认；初始 8 秒
未找到目标或途中持续 4 秒无法重新确认则失败停车，不再转向。锁定后不切换
到另一条腿。确认前沿对齐并静止后才顺时针转 90°，保存 WAITING_FOR_PLACEMENT。

参数位于 `config/post15_leg_approach.json`。只复用旧聚类算法，不使用静态地图点
代替实时检测，也不做四腿矩形匹配。需在实际桌腿可见性、里程计漂移条件下验证。
