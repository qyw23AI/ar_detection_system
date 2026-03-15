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
