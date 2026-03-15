## 重定位与原点行为说明

### 问题概述

当启用 `locate_in_prior_map` 后，系统如何确定坐标原点？是使用先验地图的原点，还是以当前启动/Teaser 给出的初始位姿为原点？

### 当前实现（关键点、代码位置）

- FAST‑LIVO 仅在 `locate_in_prior_map == true` 时订阅初始位姿 `/icp_result`：
  - [src/FAST-LIVO/src/LIVMapper.cpp](src/FAST-LIVO/src/LIVMapper.cpp#L292)
- 若启用则加载 `prior_map_path` 指定的 PCD 并下采样：
  - [src/FAST-LIVO/src/LIVMapper.cpp](src/FAST-LIVO/src/LIVMapper.cpp#L312-L328)
- 节点在主循环中若 `locate_in_prior_map && !initial_pose_received` 会等待/返回，直到收到初始位姿：
  - [src/FAST-LIVO/src/LIVMapper.cpp](src/FAST-LIVO/src/LIVMapper.cpp#L761-L768)
- 接收初始位姿回调将 `initial_pose` 保存并将 `initial_pose_received = true`：
  - [src/FAST-LIVO/src/LIVMapper.cpp](src/FAST-LIVO/src/LIVMapper.cpp#L1040-L1048)
- 核心：用收到的初始位姿构建变换 `T_init`，取其逆并用该逆变换把先验地图变换到 FAST‑LIVO 的坐标系，随后交给 `voxelmap_manager->BuildVoxelMap()` 初始化：
  - [src/FAST-LIVO/src/LIVMapper.cpp](src/FAST-LIVO/src/LIVMapper.cpp#L524-L540)
  - 变换后把点云发布为 `odom_frame`：见同段代码。
- Teaser/GICP 发布初始位姿（frame_id 为 map）：
  - [src/relocalization/src/teaser_gicp_node.cpp](src/relocalization/src/teaser_gicp_node.cpp#L408-L416)

### 行为结论（一句话）

当前实现把“Teaser/GICP 发布的初始位姿”作为对齐参考；FAST‑LIVO 用该初始位姿的逆变换把先验地图插入到它的内部 `odom/world` 地图中。换言之，系统以“当前启动/初始位姿”为参考把先验地图对齐并插入，而不是直接把先验地图原点强制设为系统原点。

### 可能的选项与建议

1. 保持当前行为（推荐用于常见重定位流程）
   - 在 `avia_relocation.yaml` 中把 `locate_in_prior_map: true` 并确保 `prior_map_path` 指向可读 PCD。保证 `teaser_gicp_node` 能发布稳定的 `/icp_result`。
2. 若你需要“以先验地图原点作为系统原点”
   - 需要修改 FAST‑LIVO 的初始化逻辑：不要对 prior map 做 `T_init.inverse()` 变换并插入；或者在插入后把 FAST‑LIVO 的内部状态初始化到与 prior map 原点对齐（修改 `_state` 或 odom→map 的 TF 发布逻辑）。我可以帮你实现并提交补丁。

### 下一步（你可以选）

- 我帮你把 `locate_in_prior_map` 设为 `true` 并验证 `prior_map_path` 可读。
- 或者我直接改代码，使系统以先验地图原点为全局原点（并提供补丁与说明）。

---

> 文件由自动化分析生成，基于 `LIVMapper.cpp` 与 `teaser_gicp_node.cpp` 的实现。

### 变更记录（你在 `LIVMapper.cpp` 做的修改——摘要）

下面条目基于当前 `LIVMapper.cpp` 的实现，说明为解决 issues 中的原点/重定位问题所做的具体修改点，已同步到本文件以便追踪：

- 在参数声明与读取中显式保留并读取 `locate_in_prior_map` 与 `prior_map_path`（默认由 YAML 控制）：

  - 见 `LIVMapper::readParameters`（[src/FAST-LIVO/src/LIVMapper.cpp#L120-L140](src/FAST-LIVO/src/LIVMapper.cpp#L120-L140)）
- 在 `initializeSubscribersAndPublishers` 中，加入当 `locate_in_prior_map==true` 时订阅初始位姿话题 `/icp_result`：

  - 见 `LIVMapper::initializeSubscribersAndPublishers`（[src/FAST-LIVO/src/LIVMapper.cpp#L284-L292](src/FAST-LIVO/src/LIVMapper.cpp#L284-L292)）
- 在同一函数中，当 `locate_in_prior_map==true` 时加载先验 PCD（`prior_map_path`）并对其下采样，保存到 `feats_down_prior_map`：

  - 见加载与下采样代码（[src/FAST-LIVO/src/LIVMapper.cpp#L319-L330](src/FAST-LIVO/src/LIVMapper.cpp#L319-L330)）
- 在首次构建体素地图逻辑处（`handleLIO` 的首帧初始化分支），加入：

  1) 若 `locate_in_prior_map` 为真，则先确保已收到 `initial_pose`（由 `/icp_result` 填充）；若未收到则提前返回等待（避免错误初始化）。
  2) 用收到的 `initial_pose` 构建变换 `T_init`，计算 `T_init.inverse()`（`T_init_inv_f`），把 `feats_down_prior_map` 用该逆变换转换成 `tmp_prior`：
     - 见构建与变换（[src/FAST-LIVO/src/LIVMapper.cpp#L520-L540](src/FAST-LIVO/src/LIVMapper.cpp#L520-L540)）。
  3) 将 `tmp_prior` 发布（frame 设置为 `odom_frame`）并交给 `voxelmap_manager->feats_down_world_` / `feats_down_body_`，然后调用 `voxelmap_manager->BuildVoxelMap()` 完成初始化。
     - 见后续交接与 `BuildVoxelMap()`（[src/FAST-LIVO/src/LIVMapper.cpp#L540-L552](src/FAST-LIVO/src/LIVMapper.cpp#L540-L552)）。
- 为防止在定位模式误更新地图，修改运行时逻辑使得在 `locate_in_prior_map` 为真时跳过 `voxelmap_manager->UpdateVoxelMap(...)`（定位模式只读 prior map，用来配准、估计位姿）：

  - 见 `handleLIO` 中的分支（[src/FAST-LIVO/src/LIVMapper.cpp#L720-L728](src/FAST-LIVO/src/LIVMapper.cpp#L720-L728)）
- 为确保回调不会在未初始化时处理数据，多个回调（`standard_pcl_cbk`、`livox_pcl_cbk`、`imu_cbk`、`img_cbk` 等）里加入了 `if (locate_in_prior_map && !initial_pose_received) return;` 的守卫，避免在等待初始位姿期间处理传感器数据：

  - 见对应回调（例如 `standard_pcl_cbk` 在 [src/FAST-LIVO/src/LIVMapper.cpp#L284-L302](src/FAST-LIVO/src/LIVMapper.cpp#L284-L302) 与 `livox_pcl_cbk` 在 [src/FAST-LIVO/src/LIVMapper.cpp#L328-L346](src/FAST-LIVO/src/LIVMapper.cpp#L328-L346)）。

### 变更带来的效果 (总结)

- 解决了之前启用 `locate_in_prior_map` 后 FAST‑LIVO 在未收到初始位姿时进入错误/阻塞状态的问题：现在会明确地加载先验地图、等待 Teaser 的 `icp_result`，并用 Teaser 提供的初始位姿把先验地图对齐到 FAST‑LIVO 的 `odom/world` 中完成初始化。
- 以当前实现为准，系统仍是“以收到的初始位姿为参考把先验地图插入到FAST‑LIVO地图中”（即 prior map 被变换并插入），而非直接把先验地图原点设为 FAST‑LIVO 的全局原点；如果需要后者，参见上面 "可能的选项与建议" 中第 2 项，我可以帮你实现代码改动。

如果你确认这些变更描述准确，我会把它们保存在 `issues.md`（已完成）并可以生成一个简要的补丁说明用于提交到仓库历史。
