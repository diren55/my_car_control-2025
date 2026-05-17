# 智能车 20 届室外 ROS 无人车赛项目交接报告

> **本文件目的**：让接手的 AI 或队员在一个文档里就能完整理解项目状态、决策依据、已做和待做工作，不必再翻几十轮对话。
>
> **当前进度**：stage1 工程化基建已落地（15+ commits）。准备进入 Stage 0（上车数据收集）。
>
> **最近更新**：2026-05-16（已核查 control_only/full_system/modular_system 三个 launch 的 YAML 传参状况）

---

## 0. 一句话现状

接手 25 届学长（国一获奖）的 ROS2 代码 `my_car_control`，作为 2026 年第 21 届智能车赛室外 ROS 无人车组的底座。已经完成桌面阶段的工程化基建（调试工具链 + 配置体系 + git 版本管理），**所有改动到目前为止都是"零运行时影响"的离线准备工作**。接下来需要先上车做事实核查（Stage 0），再决定具体策略路线。

---

## 1. 项目概况

| 项目 | 信息 |
|------|------|
| 比赛 | 全国大学生智能汽车竞赛 第 21 届（2026 年）室外 ROS 无人车赛 高教组 |
| 主队代码底座 | 25 届学长的 `my_car_control`（2025 年第 20 届国一） |
| 车辆 | 官方比赛车型，Ubuntu 22.04 + ROS2 Humble + 昇腾 310 NPU |
| 雷达 | LS01X，360°，10Hz 扫描，25m 量程 |
| IMU | IMU-03A |
| 舵机 | 1520us / 330Hz |
| 底盘 | XT-RC R3 1/8 |
| Git 远程 | github.com/diren55/my_car_control-2025 |
| 用户身份 | 接手的本科生队员（暂未拿到车，桌面准备阶段） |

**比赛节奏假设**：上车时间紧张，每次车上调试时间宝贵。这是所有"先离线做完再上车"策略的核心理由。

---

## 2. 代码资产（三套国一代码）

### 2.1 主底座：`my_car_control`（25 届学长，第 20 届国一）

- **角色**：当前改造底座
- **特点**：原生 ROS2、依赖完整、所有 launch 能起、代码闭合、自测能完赛
- **缺点**：工程化粗糙（魔法数字 / 死参数 / 圈数注释 / 半实现机制并存）
- **目录**：git 仓库根/my_car_control/（与 setup.py 同级的那一层是 ROS 包根）

### 2.2 经验金矿：`5Gsmartcar`（22 届学长，第 19 届国一）

- **角色**：参考与经验来源，**不直接迁移代码**
- **特点**：ROS1 原生（迁 ROS2 半成品），算法成熟有完整技术手册
- **关键陷阱**：22 届服务 **19 届** 规则（单一固定赛道、无蓝挡板），跟 20 届不一样。看起来好的地方未必能直接用
- **价值点（可吸收）**：
  - 行驶记录 + 离线 matplotlib 重画的调试方法论 ✅ 已迁移
  - A/B 牌投票 + combo 锁定（与底层算法无关的工程模式）✅ 已迁移
  - PWM 中心值标定（验证一致）✅ 已确认
- **价值点（不能直接用）**：
  - 斜边圆 cut_length 公式（19 届特化、依赖固定赛道几何）❌
  - 圈数切换状态机（25 年学长主动废弃，更简单方案在 20 届更鲁棒）❌
  - 轮廓数判别 A/B（25 年学长试过后主动换成模板匹配）❌
  - Pure Pursuit + PD 控制（不同流派，不要换）❌

### 2.3 战略参考：山大 2026 国一方案（"惯导记忆回放"）

- **核心**：第一圈低速跑用 IMU+编码器记录轨迹，第二圈纯惯导死算位姿 + 开环回放第一圈记录的舵角/速度
- **理论优势**：开环回放不受感知-控制环路延迟约束，第二圈可逼近车物理极限
- **没有代码**，只有方法论
- **可行性**：跟 20 届规则完全兼容（必须实时跑第一圈，必须建图）
- **推荐为主路线**，但**前提是 Stage 0 验证 /odom_combined 漂移可控**

---

## 3. 20 届规则要点（决定一切策略）

### 3.1 时间计算公式（最核心！）

```
有效时间 T = 第一圈耗时 × 0.30 + 第二圈耗时 × 0.70 + 罚时

得分 = ((T_max - T) / (T_max - T_min))² × 80    [平方曲线]
```

**含义**：
- **第二圈权重 70%**：所有提速优化优先做第二圈
- **平方曲线**：越接近 T_min 边际收益越高，每秒价值不线性
- **罚时是硬刀子**：宁慢勿罚

### 3.2 关键罚时表（按危险性排序）

| 罚项 | 罚时 | 备注 |
|------|------|------|
| 没建图 / 建图明显造假 | **+50s** | 一刀几乎判死，必须 Cartographer 全程跑 |
| 用 ROS1 | +100s | 强制 ROS2 |
| 人为干预 | +100s/次 | 必须完全自主 |
| 第二圈停车失败 | +20s | A/B 识别 + 停车定位要稳 |
| 红灯没停 3 秒 | +20s | 红灯识别 + 停车计时要稳 |
| 黄线压线 | +5s/次，连续 3s 判失败 |  |
| 撞每个锥桶 | +5s/个 |  |
| 偏离每错绕一个锥桶 | +5s/个，错绕超 3 个判失败 |  |
| 第一圈漏蓝色挡板 | 每丢一块 +5s（5 块满计） | 20 届新增反作弊 |
| 全程停车超 10s | 判失败 | 紧急刹停别太久 |

**直接结论**：
- 建图 50s 罚是头号生死线
- 撞桶 5s 比"激进过弯"更值得规避（宁慢勿撞）
- A/B + 红灯识别可靠性比速度更重要

### 3.3 20 届与 19 届的关键差异

| 维度 | 19 届（22 年代码服务）| 20 届（25 年代码服务，今年）|
|------|---------------------|---------------------------|
| 赛道版本 | **1 个固定赛道** | **A/B/C/D 四种，比赛当天定** |
| 蓝色挡板 | 无 | **5 块（50×80cm）随机摆** |
| 未建图罚 | +15s | +50s |
| 红灯罚 | +10s | +20s |
| 停车失败 | +10s | +20s |

**核心洞察**：20 届规则在反作弊和反过拟合方向上比 19 届严格得多。22 年代码里很多"针对单一赛道硬编码"的优化，在 20 届会因为赛道未知而失效。25 年学长在很多地方做的"看似工程粗糙"的简化，其实是 20 届规则下的主动选择。

---

## 4. 战略级判断（最重要的章节）

### 4.1 25 年代码里"看着差但是工程妥协"的特性（**绝对不要改！**）

这些是 25 年学长在 20 届规则下的主动选择，**不是疏忽、不是 bug**。改回 22 年风格会让车在 20 届变差。

| 25 年现状 | 表面问题 | 实际原因 |
|----------|---------|---------|
| `self.circle` 永远=1，红灯每次都停 3.5s | 圈数永远不切换 | 4 赛道下"现在是第几圈"鲁棒判断难，直接不要这个状态简单一万倍 |
| 内外道分离用 4 个 if/else 极简规则 | 看起来粗糙 | "前方远点优先归外道"是对付 5 蓝挡板的针对性方案，挡板会被自动归外道不污染内道点 |
| A/B 用模板匹配（注释掉了 22 年的轮廓识别）| 不是"经典算法"| 模板匹配在 NPU 摄像头光照下更稳定，是试错后的主动选择 |
| 直角弯检测：视觉 + 几何双信号 + 挡板过滤 | 复杂 | 这是 20 届独有的挡板规则迫使的针对性工程，22 年完全没这套 |
| 两个节点各自做 odom 归零 | 看起来冗余 | 4 赛道下绝对坐标无意义，这是 20 届泛化的基础设施 |

### 4.2 22 年里**真正可以安全迁移**的（只有 3 项）

1. ✅ **行驶记录 + 离线 matplotlib 重画**（调试方法论）→ 已迁移为 TrackMemoryRecorder + Viewer
2. ✅ **A/B 投票 + combo 锁定**（抗闪烁工程模式，独立于底层算法）→ 已实现为 ab_vote_node（默认关闭）
3. 🟡 **navigation_manager 补 save/load 触发**（25 年学长写了没接通）→ 未做，等 Stage 0 决定要不要做

### 4.3 推荐主路线：山大惯导记忆回放（待 Stage 0 验证）

**核心思想**：
```
第一圈：
  - 主控：保留 25 年现有反应式（PID + 当帧 /target）
  - 速度降低 60-80%，保证轨迹稳定
  - Cartographer 全程跑（避 50s 罚 + 顺便记 5 蓝挡板）
  - TrackMemoryRecorder 50Hz 记 (s, x, y, yaw, target_speed, target_steer, mode, ...)
  - 红灯停 3.5s 后触发 save_csv()

第二圈：
  - 主控：MemoryReplay（当前死算位姿 → 查最近轨迹点 → 直接 publish 记录的 steer/speed）
  - 安全层：当帧雷达只做"前方 0.5m 内有点 → 强减速"，不参与转向
  - 视觉层：红灯/A/B/挡板继续检测（决定停车）
  - 兜底：位姿偏离 >0.5m 或 IMU 超时 → 自动降级回反应式
```

**为什么这条路线**：
- 70% 权重在第二圈，开环回放能榨干物理极限
- 山大今年用这个拿了国一，方法论被验证
- 跟 20 届规则完全兼容（必须实时跑第一圈、必须建图、不能预录）
- 兜底机制保证最坏情况退回 25 年原版（仍能完赛）

**关键技术风险**（必须 Stage 0 解决）：
1. `/odom_combined` 漂移多大？谁发的？有没有融合 IMU？
2. 第二圈一圈累积误差 < 0.5m 吗？
3. Cartographer pose（map→base_link）能不能作为更稳的位姿源？

**这条路线现在不能立刻动代码**：在车都没碰过之前写大量"假设性"代码，70% 概率上车一测就要推倒。所有山大路线的代码工作要等 Stage 0 数据回来才有依据。

### 4.4 明确**不做**的（保护性清单）

- ❌ 把 22 年斜边圆 cut_length 公式接入代码（19 届固定赛道特化，20 届挡板会破坏前提）
- ❌ 回退到 22 年圈数切换状态机（25 年"红灯不分圈"在 4 赛道更鲁棒）
- ❌ 换回 22 年轮廓判别 A/B（学长已经试过放弃了）
- ❌ Pure Pursuit / ADRC 替换现有 PID（不同流派，没必要换，风险大）
- ❌ Nav2 / MoveBase 主控（学长尝试过没接通，不要再尝试）
- ❌ 自定义 .msg 包（Float32MultiArray 加字段足够，少一层构建风险）
- ❌ chassis_driver_node 抽象层（直接发 /teleop_cmd_vel 链路最短）
- ❌ 删除 25 年代码里看似冗余的设计（特别是 4.1 表里那些）
- ❌ 在车上**没**录 baseline bag 之前做任何"修复"工作

---

## 5. 当前代码已知问题清单（按严重性）

### 5.1 🔴 严重（影响关键功能）

| # | 问题 | 影响 | 修复策略 |
|---|------|------|---------|
| 1 | `self.circle` 永远=1（圈数切换全注释）| `check_xy_parking` 里 circle=2 的停车点永远不触发 | 上车确认实际停车机制后再决定。可能现状已是最稳方案 |
| 2 | 4 个 launch 入口，3 个走旧 KMeans，只有 `perception_control*` 走进化版 `simple_cluster` | 选错 launch 就用不上 simple_cluster 的直角弯双信号 / 挡板过滤 | 决定金线 launch，建议 perception_control* 系列 |
| 3 | `navigation_manager` 写了 `start_recording()` 但没有 `save_csv()` / `load_csv()` 触发 | 第一圈记录的路径点只在内存，CSV 永远不生成 | 待 Stage 0 决定要不要接通 |
| 4 | `enable_xy_parking=False` 且 `park_points=[]` 默认 | check_xy_parking 实际不工作。当前停车机制不确定 | 必须问学长或上车实测 |

### 5.2 🟡 中（陷阱性，已加 WARNING）

| # | 问题 | WARNING 位置 |
|---|------|------------|
| 5 | `cut_length_y/x` 在 YAML / declare 但代码不用（19 届残留）| perception_params.yaml 已加 WARNING |
| 6 | `yellow_curve_angle/speed` 在 YAML 但代码硬编码不读 YAML | control_params.yaml 已加 WARNING |
| 7 | `TRAFFIC_LIGHT` / `PARK_ZONE` 协议中有但 perception 不发 | 协议注释已写明 |

### 5.3 🟢 低（信息价值）

- 两个节点各自 odom 归零（理论冗余实际无 bug，不动）
- `cut_length_y` 在 YAML 值是 0.8，原 22 年值是 0.7，差异未澄清
- `image_detector` HSV 阈值硬编码（未来可参数化）

### 5.4 ✅ 已核查无问题（更新自第一版 HANDOFF.md）

- 之前担心 `control_only/full_system/modular_system` 三个 launch 可能 `default_value=...` 只是 launch arg 不真传给 Node。**经 grep 核查：三个文件的 control_node Node 段都正确写了 `parameters=[control_config]`**，YAML 真的传到位了。只有原版 `perception_control.launch.py` 是真坑（已修复 commit aacd907）。
- 这是个**积极信号**：25 届学长的 launch 文件大部分是工程正确的，坑没有想象中多。

### 5.5 ⚪ 未确认的关键事实（需要问学长 / 上车）

- 比赛实际跑的是哪个 launch？（推测 `full_system.launch.py` 但不确定）
- 圈数切换逻辑是什么时候、为什么注释掉的？
- `~/navigation_points.csv` 比赛当天是否真的生成？
- `~/maps/first_circle_map.pbstream` 是否存在？
- 比赛实际靠什么机制最后停车？（红灯 cooldown 之后？xy_parking？）

---

## 6. 已完成的工作（commits 时间线）

```
xxxxxxx  stage1: 加 .gitignore（屏蔽备份包/colcon 构建产物/运行时数据/编辑器临时/误重定向残留）
08347d5  stage1: 加 HANDOFF.md 完整交接报告（给下一个接手的 AI/队员用）
aacd907  stage1: perception_control.launch.py 加载 control_params.yaml
ce9748b  stage1: control_params.yaml 补 10 个缺失参数 + yellow_curve WARNING
9cc89c1  stage1: cut_length_y/x 加 WARNING 注释（标历史档案）
f84eb3a  stage1: 加 csv_diff.py 离线 TrackMemory CSV A/B 对比工具
ef671a5  fix: 删除误重定向产生的 h 文件
5d30188  stage1: 加 ab_vote_node 节点（22 年 A/B 投票+combo 锁定，零侵入）
b683985  stage1: package.xml 补 cartographer_ros_msgs/std_srvs/rosbag2_py 依赖
a9e896c  stage1: 加 .gitattributes 强制 LF 行尾
ba5a143  stage1: 复活 perception_params.yaml（27 个参数）+ launch 改为加载 YAML
41bef07  stage1: 加 bag_to_csv 离线 rosbag2 → CSV 转换器
2c76877  stage1: 给 perception/control/recorder 加魔法数字协议注释
28bc6fe  fix: 把 track_memory_recorder/viewer 从 launch 移到 my_car_control 模块目录
78e65b9  stage1: 加 TrackMemoryRecorder 等被动观察工具
13142e4  (tag: v0-baseline) baseline: 学长 2025 原版代码，未做任何修改
```

### 关键工程特性

- ✅ **完整 git 历史**：每个改动都是独立 commit，可任意回退到 `v0-baseline`
- ✅ **零运行时影响**：到目前为止所有改动都是"新增 / 注释 / 配置 / YAML 默认值不变"，不影响车的实际行为
- ✅ **github 远程同步**：所有 commit 已 push 到远程
- ✅ **统一行尾**：.gitattributes 强制 LF，Windows/Linux 协作不会再有 CRLF 警告
- ✅ **垃圾自动屏蔽**：.gitignore 屏蔽备份包/构建产物/运行时数据/编辑器临时文件

### 已搭建的工具链（这些工具是后续所有改动的"验证基础设施"）

| 工具 | 用途 | 入口 |
|------|------|------|
| `track_memory_recorder` | 50Hz 被动记录每帧状态到 CSV | `ros2 run my_car_control track_memory_recorder` |
| `track_memory_viewer` | 离线 matplotlib 重画 CSV，按模式着色 | `python3 track_memory_viewer.py xxx.csv` |
| `bag_to_csv` | 把 ros2 bag 直接转 CSV（不用 play）| `ros2 run my_car_control bag_to_csv my_run` |
| `csv_diff` | 两份 CSV A/B 对比，按 s 弧长对齐 | `python3 csv_diff.py old.csv new.csv` |
| `ab_vote_node` | A/B 牌投票后处理（默认关闭）| 用 `perception_control_with_ab_vote.launch.py` 启用 |
| YAML 参数体系 | 感知和控制都能从 YAML 调参 | `config/perception_params.yaml` + `config/control_params.yaml` |

---

## 7. 剩余的离线小风险改动（按优先级）

接手 AI 在没车的情况下可以继续做的工作：

| # | 任务 | 风险 | 工作量 | 价值 | 状态 |
|---|------|------|------|------|------|
| ~~A~~ | ~~核查三个 launch 是否真传 control_config~~ | - | - | - | ✅ 已完成，见 5.4 |
| B | 整理 13 个 launch 文件关系图 + 决定金线 launch | 零 | 30 分钟 | ⭐⭐⭐⭐ |  |
| C | 写 LAUNCH_GUIDE.md（13 个 launch 各自用途 + 推荐使用流程）| 零 | 30 分钟 | ⭐⭐⭐⭐ | 建议与 B 合并 |
| D | 离线 HSV 调试工具（读单张图，三滑块调 H/S/V 阈值实时显示 mask）| 零 | 1 小时 | ⭐⭐⭐ |  |
| E | 整理 `image_detector.py` 的 HSV 硬编码值列表，准备未来 YAML 化 | 零 | 30 分钟 | ⭐⭐ |  |

### 推荐先做 B+C 合并：LAUNCH_GUIDE.md

13 个 launch 文件谁是入口、谁加载什么 YAML、推荐用哪个，目前散落在代码里。整理成一份文档对**上车决策**和**下一个 AI**都极有价值。

---

## 8. 必须上车做的事（Stage 0）

**所有后续战略决策都建立在这一步的实测数据上**。在拿到下面这些数据之前，**不要做任何战略性的代码改动**。

### 8.1 必做（不解决就别动战略性改动）

1. **问学长 5 个问题**（最高优先级）
   - 比赛实际用的是哪个 launch？
   - 圈数切换逻辑是何时、为什么注释掉的？
   - `~/navigation_points.csv` 比赛当天是否真的生成过？
   - 有没有手机视频或比赛日志？
   - 知道蓝挡板新规则吗？比赛当天怎么处理的？

2. **在车上完整复现一次原版**
   - 用 `full_system.launch.py` 或学长 bash_history 里出现最多的命令
   - 录 baseline bag：`ros2 bag record /scan /image_raw /image_detection /odom_combined /target /teleop_cmd_vel /tf /tf_static -o baseline_run`
   - 现场观察：实际跑成什么样？哪里停？哪里失误？

3. **量化 `/odom_combined` 漂移**
   - 静止 60s 漂移：应 < 5cm
   - 推车直线 2m 误差：应 < 10cm
   - 推车原地转 360° yaw 误差：应 < 5°
   - 这是山大路线可行性的**命门**

4. **蓝挡板进 Cartographer 地图实测**
   - 在赛道边放一块 50×80cm 蓝色硬纸板
   - 跑 `mapping_with_tf.launch.py`
   - 在 pbstream 里能看见这块板吗？
   - 看不见就调 Cartographer lua（min_z / missing_data_ray_length / max_range）

5. **`git init` 备份 + 桌面端已有的 git 推送到车上**
   - 让车上代码与桌面 git 同步
   - 推荐：在车上 `git clone` 远程仓库到 `~/ros2_ws/src/`

### 8.2 建议（不做的话第三阶段以后会卡）

6. 摸清外部依赖：`ros2 pkg list | grep -E "cartographer|usb_cam|nav2"`
7. 看 `~/maps/` 和 `~/navigation_points.csv` 是否存在（不要覆盖！先备份）
8. 跑完 bag 后用 `bag_to_csv` + `csv_diff` 验证工具链能在车上跑

---

## 9. 工程化决策原则（重要！）

接手 AI 看到任何"看起来能优化"的地方之前，先过一遍这个原则：

1. **比赛优先 > 代码优雅**：能拿分的丑代码 > 拿不到分的优雅代码
2. **已验证能跑 > 理论更优**：25 年学长已经拿了国一，每个看起来怪的地方都可能是踩过坑的工程妥协
3. **工程笨办法 > 学术算法**：硬阈值、硬窗口、硬状态机更容易现场调参；学术算法依赖过多参数和标定，比赛日抓瞎
4. **防御性 > 激进**：宁慢勿罚。撞桶 -5s 比"稳过 +2s" 价值高
5. **罚时压线第一**：50s 建图罚 > 20s 红灯罚 > 5s 撞桶罚
6. **离线 A/B 实证 > 主观判断**：任何"我觉得这样更好"都用 csv_diff 验证，没数据不上线
7. **可回退 > 不可回退**：所有改动 git 分 commit，能一键回到 v0-baseline
8. **每次改动至多动一个文件 / 一个功能**：避免"改了一堆，跑不通了不知道哪里坏的"

### 看到什么时停一下，先想想是不是工程妥协：

- `self.circle 永远=1` → 故意的
- 内外道分离 4 个 if/else → 故意的
- A/B 用模板匹配不用轮廓 → 试错后选的
- 两节点各自 odom 归零 → 不影响实际行为
- check_xy_parking 看起来主路径但默认关闭 → 学长留的可选机制

---

## 10. 与用户交互的注意事项

### 10.1 用户偏好

- **比赛工程视角**：不要写学术论文式分析。直接说"做什么、风险多大、能拿几分"
- **每改一件事就 commit + push**：用户喜欢小步快走，git 历史是实证基础
- **风险评估优先**：先说零风险的，再说有风险的
- **明确不做的事**：用户会被"看起来好"的方案诱惑，要清晰列出"为什么不要做"

### 10.2 命令行环境

- 用户在 Windows + VSCode + PowerShell
- 仓库路径：`C:\Users\23998\Documents\xwechat_files\wxid_c50e3xrrui522_164e\msg\file\2026-05\my_car_control`
- **PowerShell 陷阱**（已踩过的）：
  - 不支持 `&&`，用分号 `;` 或分多行
  - 命令复制粘贴后必须按 Enter 让光标到新行再粘下一条，否则会粘连出错
  - 不要给注释和命令混合的代码块（`# xxx` + `command`），用户经常把注释一起粘上去
  - PowerShell 显示 UTF-8 中文会乱码（`type` 命令），让用户用 VSCode 看文件
  - PowerShell 5 默认 `>` 是 UTF-16，文本文件别用 `> file.txt`（会乱码），用 `Out-File -Encoding utf8`
  - `git config --global core.pager ""` 关闭分页（已设过但偶尔会忘）
  - **Windows 复制点开头文件会丢点**：`.gitignore` 复制后变 `gitignore`，需要 `Rename-Item gitignore .gitignore` 修
  - **PowerShell 误重定向**：命令拼写错或粘连时容易触发 `> 文件名`，产生 `h` / `tatus` 这种残留文件。.gitignore 已屏蔽常见单字母残留

### 10.3 文件位置易错点

**重要**：仓库根 vs ROS 包根 vs Python 模块层是三层不同的目录！

```
my_car_control/              ← Git 仓库根（PowerShell 当前位置）
├── .git/
├── .gitattributes           ← 仓库级文件放这
├── .gitignore               ← 仓库级文件放这
├── HANDOFF.md               ← 本文件
└── my_car_control/          ← ROS 包根
    ├── config/              ← YAML 参数文件放这
    ├── launch/              ← launch 文件放这
    ├── my_car_control/      ← Python 模块层
    │   ├── perception_node.py
    │   ├── control_node.py
    │   ├── *.py             ← 所有 Python 节点放这层
    │   └── ...
    ├── package.xml          ← ROS 包级文件
    ├── setup.py             ← ROS 包级文件
    └── setup.cfg
```

每次给用户新文件，**必须**明确写出"放在哪一层"，否则 90% 会放错位置。出过两次错位事故已经修复。

### 10.4 推荐工作流

每次改动：
1. AI 给文件 + 标注精确路径
2. 用户复制 / VSCode 编辑
3. 用户跑 `git status` 确认是 modified 而不是 untracked（除非真新增）
4. 用户跑 `git diff` 看具体变化
5. 用户 `git add -A` + `git commit -m "..."` + `git push`
6. 用户贴 `git log --oneline` 给 AI 确认
7. AI 开下一件

---

## 11. 下一步该做什么（接手 AI 的优先级判断）

### 11.1 如果接手时用户还没有车

继续做剩下的离线小风险改动（第 7 节 B-E）。**不要碰山大路线代码**——没有 Stage 0 数据，写出来的 memory_replay 都是空中楼阁。

### 11.2 如果用户拿到车

立刻进入 Stage 0（第 8 节），把测量数据收集完。**Stage 0 完成前不要做任何战略性改动**，包括启用 A/B 投票。

### 11.3 Stage 0 数据回来后

根据 `/odom_combined` 漂移和 Cartographer pose 质量决定：

| Stage 0 测量结果 | 推荐路线 |
|----------------|---------|
| odom 漂移 < 10cm/分钟 且 Cartographer pose 稳定 | 启动山大惯导回放路线（第 4.3 节） |
| odom 漂移 10cm-50cm/分钟 | 用 Cartographer pose 作为位姿源，仍可走山大路线 |
| odom 漂移 > 50cm/分钟 | 山大路线放弃，回到 25 年原版反应式，做细节优化 |

无论走哪条路，**先启用 A/B 投票**（已经做好的）应该都能净拉分（避 A/B 错检 -20s）。这是 Stage 0 之后第一个该开的开关。

---

## 12. 关键问题留给下一个交接

接手 AI 在做完一轮工作后，**必须更新本文件**，让再下一个接手的人无缝衔接。包括：

1. 本轮做了什么（git commits 列表）
2. 本轮发现了什么新事实（更新第 5 节问题清单，把已核查的移到 5.4）
3. 本轮放弃了什么尝试（让下一个人不重复）
4. 当前 Stage 0 完成度
5. 用户的下一次车上时间窗口

---

## 13. 关键链接和文件指引

| 类别 | 路径 |
|------|------|
| 仓库远程 | github.com/diren55/my_car_control-2025 |
| baseline tag | `git checkout v0-baseline` |
| 主底座代码 | `my_car_control/my_car_control/` |
| YAML 参数 | `my_car_control/config/` |
| Launch 文件 | `my_car_control/launch/` |
| 推荐金线 launch（待 Stage 0 确认）| `perception_control.launch.py` 或 `perception_control_with_ab_vote.launch.py` |
| 22 年源代码 | 用户本地有 5Gsmartcar 解压目录，仓库里没有 |
| 20 届规则文档 | 比赛官方下载 |

---

## 附录 A：14 项考古发现（详细版）

> 这是 stage1 期间对 25 年代码 + 22 年代码做的系统对比，所有发现都已 grep / view 验证。

主要发现摘要：

1. 22 年 `cut_length_y/x` 真实使用在 main.py L107（25 年死参数的历史来源）
2. 22 年圈数切换完整闭环（25 年注释掉了，是主动选择）
3. 22 年 A/B 投票 + combo 锁定（已迁移）
4. 22 年用 Pure Pursuit + PD，25 年用 PID
5. 22 年 MoveBase + navigation.csv 真实接通（25 年同思路但未接通）
6. 2025 PWM 中心值与 22 年一致（1500/75）
7. 25 年双停车机制并存
8. 25 年 self.circle 永远=1 的原因（红灯不分圈机制）
9. 25 年 navigation_manager 几乎是摆设
10. 25 年 A/B 用模板匹配是主动选择
11. 25 年两节点各自 odom 归零
12. 25 年 perception 不发 -99999/-88888（注释了）
13. simple_cluster 比 KMeans 进化但默认 launch 走 KMeans
14. simple_cluster 直角弯双信号源 + 挡板过滤是 20 届独有特性

---

## 附录 B：本次交接产出的状态校验清单

接手 AI 上手后，先快速跑一遍这个清单确认理解正确：

- [ ] `git log --oneline` 显示 16+ 个 commit 一条直线，HEAD 在 master，tag `v0-baseline` 在最初
- [ ] `git status` 显示 `working tree clean`，远程已同步
- [ ] `my_car_control/my_car_control/` 下应有 `track_memory_recorder.py` / `track_memory_viewer.py` / `bag_to_csv.py` / `csv_diff.py` / `ab_vote_node.py` 五个新文件
- [ ] `my_car_control/launch/` 下应有 `perception_control_with_ab_vote.launch.py` / `track_memory.launch.py` 两个新 launch
- [ ] `my_car_control/config/perception_params.yaml` 顶部和 `cut_length` 处有 WARNING 注释
- [ ] `my_car_control/config/control_params.yaml` 在 `yellow_curve` 处有 WARNING + 末尾有 10 个补充参数
- [ ] `my_car_control/setup.py` `entry_points` 里有 5 个新 entry
- [ ] `.gitattributes` 和 `.gitignore` 在仓库根存在
- [ ] `HANDOFF.md` 在仓库根存在，本文件
- [ ] `package.xml` 有 cartographer_ros_msgs / std_srvs / rosbag2_py / rosidl_runtime_py

如所有项 ✅，说明状态正确，可以开始下一阶段工作。
