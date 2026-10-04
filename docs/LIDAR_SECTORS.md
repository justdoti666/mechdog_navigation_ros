# 雷达扇区消费 (S2) — 离线首版设计

> 2026-10-04 · 状态：**离线首版完成**（纯计算 + 单测；**未接入节点**）· 接入/启用须师兄会签
> 关联：《可行性_深度退化时超声与雷达兜底_2026-09-25.md》§4-S2；`docs/ULTRASONIC_WIRING.md`

## 背景

- `/scan`（RPLIDAR A1M8）已在 `safety_node` 订阅但**零消费**——只存 `scan_ranges_` 缓存 + 日志打 `scan=N` 计数（可行性文档 §1.1 取证）。
- S2 口径：用 scan 算**前向 L/C/R 扇区最小距离**（+ 安装角外参 ⇒ base_link），作第三路"取近"；**建议仅退化期启用**。
- 本版 = "帧 → 扇区最小距离"的**纯函数 + 单测**；不改任何节点行为（接入 = 行为变化，须会签）。

## 接口（`src/lidar_sectors.hpp`）

- 输入：`ranges[] / angle_min / angle_increment / range_min / range_max`（LaserScan 口径，单位 m）+ `LidarSectorConfig`
- 输出：`LidarSectors{ left, center, right {min_m, n_valid}, n_input }`
- 默认扇区（base_link 系，度）：**L=[-45,-15) · C=[-15,+15) · R=[+15,+45]**
  - 对齐超声三颗布局（中心 −30°/0°/+30°，各 ±15° 宽）；**现场标定/会签后可调**
- 外参：v1 仅偏航 `yaw_offset_deg`（雷达系 → base_link；默认 0，**待现场标定**）
- 单位：输入 m，输出 m（与融合层一致；超声链路 cm 在上游各自转换）

## 语义（单测逐条锁定，勿改语义而不改测试）

1. **左闭右开 + 最右端闭合**；每个点只归一个扇区（缝点用例 `SeamPointsFollowHalfOpenRule`）
2. **无效点一律跳过**：NaN / ±inf / 0.0（部分驱动的"无回波"标记）/ `< range_min` / `> range_max`
3. **fail-closed**：扇区无有效点 ⇒ `min_m = NaN`；配置非法（分界倒挂 / hi<rc）⇒ 整体全 NaN
   —— 绝不给 0 / "畅通"读数（与超声、深度守门同一口径）
4. 角度绕 ±180° 归一化；`angle_increment` 为负（反向扫描）同样成立

> 浮点说明：恰在分界角上的点归属取决于 deg→rad→deg 往返的 1e-12 级舍入（如名义 −15° 实际落 −15.000000000000002 ⇒ 归 L），物理上无意义；单测用 ±0.001° 偏移锁定语义。核对脚本：`证据/float_mirror.py`。

## 未含项（后续，须会签）

- **接入 safety_node**：参数映射（扇区边界 / 外参）+ 状态日志（如 `scan_sectors=[L,C,R]`）+ 仅退化期"取近"参与融合（与 S1 降级链联动口径一并送签）
- **现场标定**：雷达安装角（yaw）至少；安装高度/横移对扇区判定的影响评审
- **误标风险评审**：外参错 ⇒ 假障碍 / 漏障碍（可行性文档 §4 已列）

## 验证

- 单测：`test/test_lidar_sectors.cpp`（10 用例；**先红 7/10 → 绿 10/10**）
- 全量回归：`colcon test`（本笔含全量结果）
- 证据：`C:\Users\Public\s2_evidence_20261004\`（red/ + green/ + float_mirror.py 期望值核对）
- 复跑：`colcon build --packages-select mechdog_navigation_ros --cmake-args -DBUILD_TESTING=ON` 后跑构建目录下 `test_lidar_sectors`；或 `colcon test --ctest-args -R lidar`
