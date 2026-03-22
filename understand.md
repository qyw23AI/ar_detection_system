# AR 定位系统技术详细说明文档

## Layer1 雷达重定位层

本层目标：利用先验地图完成重定位，并持续输出两套里程计坐标表达：

- `aft_mapped_to_init`：以当前启动时 `camera_init` 为原点的里程计（局部里程计坐标系）。
- `aft_mapped_in_map`：以先验地图原点（`map`）为原点的里程计（全局定位坐标系）。

---

### 1. 雷达层整体架构与功能边界

雷达层由以下核心功能包构成：

1) **`livox_ros_driver2`（传感器驱动层）**

- 负责雷达硬件通信与点云/IMU原始数据发布。
- MID360 常用启动文件：`launch_ROS2/msg_MID360_launch.py`。
- 典型输出：Livox `CustomMsg`（供 FAST-LIVO	 与 relocalization 消费）。

2) **`relocalization`（全局重定位层）**

- 负责“当前实时点云 ↔ 先验地图”的配准。
- 支持 TEASER++ 粗配准 + GICP/VGICP 精配准。
- 关键输出：`icp_result`（`map` 系位姿），并通过 `transform_publisher` 转成 `map -> camera_init` TF。

3) **`fast_livo`（局部里程计/地图匹配层）**

- 负责 LiDAR-IMU 状态估计（必要时视觉可参与）。
- 在重定位模式下使用先验地图建立体素地图并持续跟踪。
- 发布：
  - `/aft_mapped_to_init`（局部）
  - `/aft_mapped_in_map`（全局）
  - 以及点云、路径、IMU传播里程计等辅助话题。

4) **`small_gicp` / `teaserpp`（算法库依赖）**

- 分别提供高效局部配准与鲁棒全局配准能力。
- 被 `relocalization` 的可执行节点调用，不直接承担 ROS 业务编排。

---

### 2. 典型启动链路（你当前的重定位流程）

常见编排见 `example_teaser_gicp.launch.py`：

1. 启动 `transform_publisher`（订阅 `icp_result`，发布 `map -> camera_init`）。
2. 启动 `teaser_gicp_node`（输出 `icp_result`）。
3. 启动 `fastlivo_mapping`（加载 `avia_relocation.yaml` + `camera_pinhole.yaml`）。
4. RViz 显示结果。

这条链路中，`relocalization` 解决“全局对齐”，`fast_livo` 解决“连续跟踪与状态输出”。

---

### 3. 关键坐标系与变换关系

#### 3.1 坐标系定义

- **`map`**：先验地图坐标系（全局参考）。
- **`camera_init`**：FAST-LIVO 启动时局部世界系（局部参考）。
- **`aft_mapped`**：FAST-LIVO 当前估计机体位姿的 child frame。

#### 3.2 关键 TF 与里程计关系

先看两条“基础变换”：

1. `relocalization/transform_publisher` 根据 `icp_result` 发布
   - **`map -> camera_init`**（全局对齐关系）
2. `fast_livo` 在每帧 LIO 结果中计算
   - **`camera_init -> aft_mapped`**（局部连续跟踪关系）

然后，两个里程计就是同一物理位姿在不同参考系下的表达：

- **局部里程计 `/aft_mapped_to_init`**

  - 直接使用 `camera_init -> aft_mapped`。
  - 也就是把当前位姿写在 `header.frame_id = camera_init` 下。
- **全局里程计 `/aft_mapped_in_map`**

  - 先从 TF 树查询 `map -> camera_init`。
  - 再与当前 `camera_init -> aft_mapped` 做链式复合：

$$
T_{map\rightarrow aft\_mapped}=T_{map\rightarrow camera\_init}\cdot T_{camera\_init\rightarrow aft\_mapped}
$$

- 把结果写成 `header.frame_id = map` 后发布。

#### 3.3 为什么“两个都需要”

- `/aft_mapped_to_init`：局部连续性强，适合控制与短时闭环，受全局跳变影响小。
- `/aft_mapped_in_map`：全局一致性强，适合跨会话定位、任务规划、AR全局锚定。

#### 3.4 一帧数据的 TF 推导过程（按时间顺序）

1. `teaser_gicp_node` 输出一次或多次 `icp_result`（`map` 参考系）。
2. `transform_publisher` 把该位姿转成 TF：`map -> camera_init`。
3. `fast_livo` 根据 LiDAR+IMU 算出当前局部位姿：`camera_init -> aft_mapped`。
4. 发布 `/aft_mapped_to_init`（局部表达）。
5. 若 TF 可查到 `map -> camera_init`，则复合后发布 `/aft_mapped_in_map`（全局表达）。
6. 若暂时查不到 TF，则保留局部里程计，并跳过全局里程计（不会发布错误值）。

#### 3.5 常见理解误区澄清

- 误区1：`/aft_mapped_in_map` 是独立估计器输出。不是。它是“局部里程计 + 全局TF”复合后的结果。
- 误区2：`camera_init` 与 `map` 是同一个原点。不是。二者通过 `map -> camera_init` 联系，通常存在平移和旋转偏置。
- 误区3：有了 `map` 系里程计就不需要 `camera_init` 系。
  实际工程里两者常并存：一个保连续性，一个保全局一致性。

---

### 4. 话题发布与作用（重点）

#### 4.1 `livox_ros_driver2` 侧

- 输入：雷达 UDP 数据流。
- 输出（典型）：
  - `/livox/lidar`（`livox_ros_driver2/msg/CustomMsg`）
  - `/livox/imu`（`sensor_msgs/msg/Imu`）

`/livox/lidar`（CustomMsg）关键字段：

- `header`：ROS 时间戳与 frame_id。
- `timebase`：该帧点云基准时刻。
- `point_num`：点数。
- `points[]`：每个点含 `offset_time/x/y/z/reflectivity/tag/line`。

这类“每点带偏移时间”的结构能支持 FAST-LIVO 做去畸变与精确时序对齐。

`/livox/imu`（Imu）关键字段：

- `angular_velocity`：角速度。
- `linear_acceleration`：线加速度。
- `header.stamp`：与点云同步的核心时间依据。

作用：为重定位与里程计提供同源、可同步的原始观测。

#### 4.2 `relocalization` 侧（`teaser_gicp_node`）

- 订阅：
  - `/livox/lidar` 或 `/pointcloud2`（实时源点云）
  - `initialpose`（人工给初值，强制切到 GICP）
- 发布：
  - `icp_result`（`geometry_msgs/msg/PoseWithCovarianceStamped`，`frame_id = map`）
  - `prior_map`（地图可视化）
  - `transformed_cloud`（当前对齐调试点云）

`icp_result` 关键字段：

- `header.frame_id = map`：结果位姿是地图坐标系下定义。
- `pose.pose.position`：平移（x,y,z）。
- `pose.pose.orientation`：旋转（四元数）。
- `pose.covariance`：位姿协方差（不确定度）。

作用：提供 `map` 参考下的全局位姿解。

#### 4.3 `relocalization` 侧（`transform_publisher`）

- 订阅：`icp_result`
- 发布 TF：`map -> camera_init`

数据映射关系：

- `icp_result.pose.pose.position` → `transform.translation`
- `icp_result.pose.pose.orientation` → `transform.rotation`

也就是说，`icp_result` 的位姿数值被直接桥接到 TF 树中。

作用：把“重定位位姿消息”桥接为 TF 树，使 FAST-LIVO 可通过 TF 查询融合。

#### 4.4 `fast_livo` 侧（`LIVMapper`）

输入：

- LiDAR：`common.lid_topic`（通常 `/livox/lidar`）
- IMU：`common.imu_topic`（通常 `/livox/imu`）
- 图像：`common.img_topic`（启用视觉时）
- 初始位姿：`/icp_result`（仅 `locate_in_prior_map=true`）

发布（核心）：

- `/aft_mapped_to_init`（`nav_msgs/msg/Odometry`）：局部里程计（`camera_init`）
- `/aft_mapped_in_map`（`nav_msgs/msg/Odometry`）：全局里程计（`map`）
- `/cloud_registered`（`sensor_msgs/msg/PointCloud2`）：配准后点云
- `/path`（`nav_msgs/msg/Path`）：轨迹
- `/LIVO2/imu_propagate`（`nav_msgs/msg/Odometry`）：高频 IMU 传播里程计
- `/Laser_map`、`/cloud_effected`、`/planes` 等调试/可视化话题

`/aft_mapped_to_init` 关键字段：

- `header.frame_id = camera_init`
- `child_frame_id = aft_mapped`
- `pose.pose`：当前位姿（局部系）

`/aft_mapped_in_map` 关键字段：

- `header.frame_id = map_frame_id`（通常是 `map`）
- `pose.pose`：经 TF 复合后的全局位姿

`/LIVO2/imu_propagate` 关键字段：

- 姿态与速度由 IMU 连续积分得到，更新频率通常高于 LiDAR 主帧频。

作用：持续输出可控制、可融合、可视化的状态估计结果。

#### 4.5 端到端发布/订阅协作链（数据视角）

1. `livox_ros_driver2` 发布 `/livox/lidar` + `/livox/imu`。
2. `teaser_gicp_node` 订阅点云，输出 `icp_result(map系)`。
3. `transform_publisher` 订阅 `icp_result`，发布 TF `map -> camera_init`。
4. `fast_livo` 订阅 LiDAR/IMU（及可选 `icp_result`），输出局部里程计 `/aft_mapped_to_init`。
5. `fast_livo` 查询 TF 后复合，输出全局里程计 `/aft_mapped_in_map`。

因此，系统协作不是“单话题驱动”，而是“消息 + TF 树”共同驱动。

---

### 5. 关键代码文件与职责

#### 5.1 `fast_livo` 包

- `src/FAST-LIVO2/src/main.cpp`

  - 进程入口，创建 `LIVMapper` 并运行主循环。
- `src/FAST-LIVO2/src/LIVMapper.cpp`

  - 雷达层核心业务：参数读取、订阅发布、同步、状态估计、地图构建、里程计发布。
  - 你本次重点功能在这里：`/aft_mapped_in_map` 发布逻辑。
- `src/FAST-LIVO2/include/LIVMapper.h`

  - 数据成员与接口总定义：缓冲区、状态量、发布器、TF缓存等。
- `src/FAST-LIVO2/config/avia_relocation.yaml`

  - 重定位模式关键参数：
    - `locate_in_prior_map`
    - `prior_map_path`
    - `publish.map_frame_id`
    - 传感器话题与滤波参数。
- `src/FAST-LIVO2/launch/mapping_avia_relocation.launch.py`

  - FAST-LIVO 单独重定位版启动（不含 relocalization 算法节点）。

#### 5.2 `relocalization` 包

- `src/relocalization/src/teaser_gicp_node.cpp`

  - TEASER 阶段：全局粗定位（鲁棒特征匹配）。
  - GICP 阶段：局部精定位（连续收敛判据）。
  - 发布 `icp_result`。
- `src/relocalization/src/transform_publisher.cpp`

  - 监听 `icp_result`，发布 `map -> camera_init`。
- `src/relocalization/CMakeLists.txt`

  - 各可执行节点构建入口：`teaser_gicp_node`、`transform_publisher` 等。

#### 5.3 `livox_ros_driver2` 包

- `src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py`
  - 驱动参数入口：点云格式、发布频率、frame_id、设备配置 json。

---

### 6. `/aft_mapped_to_init` 与 `/aft_mapped_in_map` 的实现原理

#### 6.1 局部里程计 `/aft_mapped_to_init`

来源：`publish_odometry()` 直接输出 FAST-LIVO 当前估计位姿，`header.frame_id = camera_init`。

意义：

- 低耦合、连续性好
- 适合控制与短时运动估计
- 会随局部累计误差产生漂移（相对全局）。

#### 6.2 全局里程计 `/aft_mapped_in_map`

来源：`publish_odometry()` 中先查 TF：`lookupTransform(map_frame_id_, "camera_init", ...)`，再与当前 `camera_init->aft_mapped` 复合，发布到 `map`。

意义：

- 输出与先验地图同一坐标原点的姿态
- 适合上层定位、任务规划与跨会话复用。

注意：

- 若 `map -> camera_init` 不可用，则会跳过该话题并节流告警。
- 全局精度依赖重定位结果与 TF 稳定性。

---

### 7. IMU 数据链路（重点）

IMU 在本系统中有两条用途：

1) **主滤波链路（LIO融合）**

- IMU 与 LiDAR 数据经缓冲同步（`sync_packages`）后进入状态估计。
- 用于姿态、速度、偏置等状态更新。

2) **高频传播链路（`/LIVO2/imu_propagate`）**

- 在两帧 LiDAR 之间用最新 IMU 做前向传播（`imu_prop_callback`）。
- 输出更高频里程计，利于控制或外部融合。

关键技术点：

- 偏置补偿（`bias_a/bias_g`）
- 重力补偿
- 指数映射积分（旋转更新）
- 时间戳一致性检查（防回环/跳变）

---

### 8. 文件之间的调用关系（工程视角）

1. `example_teaser_gicp.launch.py` 编排 `relocalization + fast_livo + rviz`。
2. `teaser_gicp_node.cpp` 产生 `icp_result`。
3. `transform_publisher.cpp` 将 `icp_result` 变为 `map -> camera_init` TF。
4. `LIVMapper.cpp`
   - 读取 `avia_relocation.yaml` 参数
   - 接收 LiDAR/IMU/初始位姿
   - 输出 `/aft_mapped_to_init` 与 `/aft_mapped_in_map`。
5. 上层（AR投影层）可直接消费 `map` 系里程计，实现跨会话一致定位。

---

### 9. 重要技术名词解释（简明）

- **重定位（Relocalization）**：在已有先验地图中恢复当前位姿。
- **先验地图（Prior Map）**：离线构建的全局点云地图。
- **TEASER++**：对外点鲁棒的全局配准算法。
- **GICP / VGICP**：基于局部几何统计的精细点云配准。
- **LIO**：LiDAR-Inertial Odometry，激光-惯导融合里程计。
- **`camera_init`**：FAST-LIVO 启动时定义的局部世界参考。
- **`map`**：先验地图全局参考系。
- **TF**：ROS 中坐标系变换树。
- **`icp_result`**：重定位求得的位姿消息（PoseWithCovarianceStamped）。
- **体素降采样（VoxelGrid）**：通过体素网格减少点数，平衡速度与精度。
- **点云配准 Fitness Score**：配准残差质量指标，越小通常越好。

---

### 10. 你当前改进的价值总结

你已完成的关键改进是：

1. 新增并发布 `aft_mapped_in_map`（地图原点坐标表达）。
2. 保留 `aft_mapped_to_init`（启动原点坐标表达）。
3. 形成“局部连续性 + 全局一致性”并存的输出体系。

这使系统同时满足：

- 本体稳定控制（优先局部里程计）
- 全局语义定位/AR落点一致（优先地图里程计）

即：同一状态估计结果可以在两种参考系下被上层模块无缝使用。

## Layer2 AR计算投影层

本层目标：基于雷达层输出的全局/局部里程计与相机图像，把世界坐标系中的九宫格（任意 rows×cols）实时投影到图像，并发布结构化可见格子结果供后续识别与任务逻辑使用。

### 0. 代码速查跳转（变换与数据流）

> 这一节只放“可点击定位”的关键实现，便于你按链路快速核对。

#### 0.1 AR 变换主链路（`ar_grid_node.py`）

- 参数/订阅发布初始化：
  - [读取外参 `T_c_s`](ar_calculate/src/ar_grid_detector/ar_grid_detector_py/ar_grid_node.py#L56)
  - [订阅 `Odometry` 与 `Image`](ar_calculate/src/ar_grid_detector/ar_grid_detector_py/ar_grid_node.py#L65-L66)
  - [发布 `GridCellArray`](ar_calculate/src/ar_grid_detector/ar_grid_detector_py/ar_grid_node.py#L68)
- 里程计 -> 位姿矩阵 `T_w_s`：
  - [`odom_callback` 定义](ar_calculate/src/ar_grid_detector/ar_grid_detector_py/ar_grid_node.py#L424-L442)
  - [四元数转旋转矩阵 `quat_to_rot_matrix`](ar_calculate/src/ar_grid_detector/ar_grid_detector_py/utils.py#L14)
  - [构造齐次矩阵 `make_transform`](ar_calculate/src/ar_grid_detector/ar_grid_detector_py/utils.py#L93)
- 图像帧时序对齐：
  - [按时间戳选最近里程计 `_select_pose_for_image`](ar_calculate/src/ar_grid_detector/ar_grid_detector_py/ar_grid_node.py#L377-L391)
  - [在 `image_callback` 中调用](ar_calculate/src/ar_grid_detector/ar_grid_detector_py/ar_grid_node.py#L449)
- 世界点到相机点：
  - [`T_s_w = inverse_transform(T_w_s)`](ar_calculate/src/ar_grid_detector/ar_grid_detector_py/ar_grid_node.py#L466)
  - [`T_c_w = T_c_s @ T_s_w`](ar_calculate/src/ar_grid_detector/ar_grid_detector_py/ar_grid_node.py#L467)
  - [中心点 world-&gt;camera](ar_calculate/src/ar_grid_detector/ar_grid_detector_py/ar_grid_node.py#L493-L501)
  - [角点 world-&gt;camera](ar_calculate/src/ar_grid_detector/ar_grid_detector_py/ar_grid_node.py#L507-L515)
- 相机投影与可见性：
  - [角点投影 `project_point`](ar_calculate/src/ar_grid_detector/ar_grid_detector_py/ar_grid_node.py#L525)
  - [中心投影 `project_point`](ar_calculate/src/ar_grid_detector/ar_grid_detector_py/ar_grid_node.py#L534)
  - [可见性规则计算](ar_calculate/src/ar_grid_detector/ar_grid_detector_py/ar_grid_node.py#L539-L548)

#### 0.2 雷达层数据如何进入 AR（`FAST-LIVO -> AR`）

- AR 参数配置使用全局里程计：
  - [`odom_topic: /aft_mapped_in_map`](ar_calculate/src/ar_grid_detector/config/ar_grid.params.yaml#L3)
- FAST-LIVO 发布全局里程计：
  - [创建 `/aft_mapped_in_map` Publisher](FAST_LIVO2_relocation_revise/src/FAST-LIVO2/src/LIVMapper.cpp#L309)
  - [`publish_odometry` 中查 `map->camera_init` TF](FAST_LIVO2_relocation_revise/src/FAST-LIVO2/src/LIVMapper.cpp#L1606-L1610)
  - [复合得到 `map->aft_mapped` 并写入 Odometry](FAST_LIVO2_relocation_revise/src/FAST-LIVO2/src/LIVMapper.cpp#L1622-L1636)
  - [发布 `/aft_mapped_in_map`](FAST_LIVO2_relocation_revise/src/FAST-LIVO2/src/LIVMapper.cpp#L1638)
- AR launch 如何加载参数文件：
  - [ar_grid.launch.py 指定参数文件](ar_calculate/src/ar_grid_detector/launch/ar_grid.launch.py#L12)

#### 0.3 AR 输出数据结构（下游使用）

- 发布点：
  - [`visible_cells_pub.publish(cells_msg)`](ar_calculate/src/ar_grid_detector/ar_grid_detector_py/ar_grid_node.py#L657)
  - [`image_pub.publish` 当前注释](ar_calculate/src/ar_grid_detector/ar_grid_detector_py/ar_grid_node.py#L700)
- 消息定义：
  - [`GridCell.msg`（`cell_id/is_visible/position_camera_frame/corners_pixel`）](ar_calculate/src/ar_grid_detector/msg/GridCell.msg#L8-L39)
  - [`GridCellArray.msg`（`total_cells/visible_cells/cells/visible_cell_ids`）](ar_calculate/src/ar_grid_detector/msg/GridCellArray.msg#L13-L25)

---

### 1. Layer2 目录与功能包分工

在 `ar_calculate` 中，AR 核心功能主要由以下文件/模块实现：

1) `src/ar_grid_detector/ar_grid_detector_py/ar_grid_node.py`

- AR 主节点。
- 完成里程计订阅、图像订阅、坐标变换、投影计算、可见性判断、消息发布与可视化。

2) `src/ar_grid_detector/ar_grid_detector_py/camera_models.py`

- 相机模型库。
- 支持 pinhole 与多种鱼眼模型（equidistant/equisolid/orthographic/stereographic/Kannala-Brandt）。

3) `src/ar_grid_detector/ar_grid_detector_py/grid_generator.py`

- 九宫格/通用网格几何生成。
- 根据三角点（或中心点语义）生成每个格子的世界坐标中心与四角点。

4) `src/ar_grid_detector/ar_grid_detector_py/utils.py`

- 变换数学工具：四元数转旋转矩阵、SE(3) 构造与求逆、RPY 变换等。

5) `src/ar_grid_detector/msg/GridCell.msg` 与 `GridCellArray.msg`

- AR 结构化输出消息定义。

6) `src/ar_grid_detector/config/*.yaml`

- 运行参数：话题名、相机内参/模型、网格定义、外参、绘图开关。

7) `start_all.sh` 与 `record_lidar_poses.py`

- 系统编排与标定辅助工具。

---

### 2. AR 主节点输入输出（严格按代码）

主节点构造函数中明确了订阅/发布：

- 订阅：
  - `Odometry`（参数 `odom_topic`）
  - `Image`（参数 `image_topic`）
- 发布：
  - `GridCellArray`（参数 `visible_cells_topic`）
  - `Image`（参数 `output_image_topic`）

但当前实现中，图像发布语句在代码里被注释：

- `# self.image_pub.publish(out_msg)`

因此，当前默认行为是：

- 结构化结果会发布到 `/ar_grid/visible_cells`；
- 图像主要通过 OpenCV 窗口 `cv2.imshow` 显示，而不是 ROS 话题持续发布。

---

### 3. AR 关键坐标变换与投影计算（核心）

#### 3.1 输入位姿语义

`odom_callback` 中把里程计消息解释为：

- `T_w_s`：传感器（s）在世界（w）中的位姿。

实现步骤：

1. 里程计四元数 -> 旋转矩阵 `R_w_s`。
2. 里程计位置 -> 平移 `t_w_s`。
3. 组合成齐次矩阵 `T_w_s`。

#### 3.2 外参语义

外参加载函数明确语义：

- `T_c_s`：传感器系 -> 相机系。

可由两种参数方式获得：

- 4x4 矩阵 `extrinsic_matrix_4x4`；
- RPY+XYZ `extrinsic_rpy_xyz`（内部转成 4x4）。

#### 3.3 世界点到相机点主链路

`image_callback` 的主链路（代码注释已写明）：

1. `T_s_w = inverse(T_w_s)`
2. `T_c_w = T_c_s * T_s_w`
3. 对世界点 `P_w`：

$$
P_c = T_{c\leftarrow w} P_w = T_{c\leftarrow s} T_{s\leftarrow w} P_w
$$

这一步是 AR 投影的数学核心。

#### 3.4 单格可见性判定（逐格）

每个格子执行：

1. 中心点 world->camera 变换。
2. 四角点 world->camera 变换。
3. 深度检查：任一点 `z<=0` 则判为不可用（在相机后方）。
4. 相机模型投影：`project_point` 得到像素坐标。
5. 可见规则：
   - 四角都可投影；
   - 且“中心在图像内”或“至少一个角在图像内”。

满足则 `is_visible=true`，并写入像素角点、中心、深度。

---

### 4. 话题与消息字段（数据级说明）

#### 4.1 输入话题：里程计

类型：`nav_msgs/Odometry`（在 AR 主节点中是这个类型）

使用字段：

- `pose.pose.position` -> `t_w_s`
- `pose.pose.orientation` -> `R_w_s`
- `header.stamp` -> 与图像时间对齐（最近时间戳匹配）

用途：提供每帧图像对应的传感器位姿。

#### 4.2 输入话题：图像

类型：`sensor_msgs/Image`

使用字段：

- `header.stamp`：和里程计历史做最近匹配
- 图像数据：作为叠加绘制底图

用途：承载 AR 投影的目标平面（像素空间）。

#### 4.3 输出话题：`/ar_grid/visible_cells`

类型：`ar_grid_detector/GridCellArray`

顶层字段：

- `rows` / `cols` / `total_cells`
- `visible_cells`
- `visible_cell_ids`
- `cells[]`（全体格子，含不可见格）

`cells[]` 中每个 `GridCell` 关键字段：

- `cell_id,row,col`
- `is_visible`
- `position_world_frame`（格子中心世界坐标）
- `position_camera_frame`（格子中心相机坐标，可见时有效）
- `corners_pixel[4]`（左上/右上/右下/左下）
- `center_pixel`
- `depth`

不可见格的约定（代码中明确）：

- 相机系坐标与像素字段置零
- `depth=0`

这使下游可以统一遍历 `cells[]`，通过 `is_visible` 决策，不需要做空对象分支。

#### 4.4 输出话题：`/ar_grid/image`

类型：`sensor_msgs/Image`

说明：Publisher 已创建，但当前发布调用被注释，默认不会持续发出该话题图像。

---

### 5. 网格几何生成与标定语义

`grid_generator.py` 支持两类输入语义：

1) 外框角点语义（`input_is_cell_centers=false`）

- 输入点是网格外边框角点：TL/TR/BL。
- 按 `rows/cols` 等分，生成各格中心与四角。

2) 中心点语义（`input_is_cell_centers=true`）

- 输入点是左上格中心/右上格中心/左下格中心。
- 先反推外框角点，再生成全网格。

这与现场采点方式强相关，配置错会导致整体系统性偏移。

此外还支持：

- 从 legacy 九点 `nine_grid_points.yaml`（1..9）转换。

---

### 6. 相机模型与鱼眼投影实现

在 `camera_models.py` 中：

- `PinholeCamera`：针孔模型 + 径向/切向畸变。
- `FisheyeCamera` 基类 + 4种投影子类：
  - equidistant
  - equisolid
  - orthographic
  - stereographic
- `KannalaBrandtFisheye`：OpenCV 鱼眼常用模型（k1~k4 多项式）。

主节点通过参数 `camera_model` + 内参参数创建具体模型，并统一调用 `project_point`。

因此 AR 层对外表现为“模型可替换、投影接口统一”。

---

### 7. 雷达层数据如何被 AR 层使用（对接关系）

你当前改为使用全局里程计后，AR 层的关键接入点是 `odom_topic`。

- 在 `ar_grid.params.yaml` 中已配置：
  - `odom_topic: "/aft_mapped_in_map"`

这意味着 AR 投影中的世界坐标参考，与先验地图 `map` 原点一致。

直接效果：

- 九宫格世界坐标（`nine_grid_points.yaml` 或参数定义）与里程计参考系一致时，投影更稳定；
- 若切回 `/aft_mapped_to_init`，则世界参考变为启动局部原点，跨会话一致性会下降。

---

### 8. 脚本链路与文件关系

#### 8.1 `start_all.sh`（系统编排）

- 可按环境变量启停模块：相机驱动、重定位、AR节点。
- AR 节点默认命令：
  - `ros2 launch ar_grid_detector ar_grid.launch.py`
- 并在启动后检查关键话题可用性。

#### 8.2 `record_lidar_poses.py`（采点工具）

- 用于交互记录九宫格关键点并生成 JSON。
- 当前脚本订阅类型是 `PoseStamped`，话题是 `/aft_mapped_to_init`。

注意：

- AR 主节点使用的是 `Odometry`，雷达层也通常发布 `Odometry`。
- 因此该脚本是否可直接接收，取决于现场是否存在把 `Odometry` 转 `PoseStamped` 的桥接节点。

---

### 9. 当前实现中的关键注意事项（基于代码现状）

1. AR 图像发布被注释

- 结构化消息正常发布；图像主要走 OpenCV 本地窗口。

2. 时间同步策略是“最近时间戳匹配”

- 通过 odom 历史队列选择与图像最接近的位姿。
- 若时间差过大，会输出 warning（提示投影可能抖动）。

3. 可见性判定是几何可投影 + 图像范围判定

- 未做遮挡推理。

4. 外参方向严格采用 `T_c_s`

- 即 sensor->camera，链路写死在 `T_c_w = T_c_s * T_s_w`。
- 外参方向填反会导致整体镜像或偏移。

---

### 10. Layer2 小结

AR 层本质是“世界网格几何 + 雷达层位姿 + 相机模型”的实时融合计算：

1. 雷达层给位姿（建议全局 `/aft_mapped_in_map`）。
2. AR 层构建 `T_c_w` 并将每个格子 world 点投影到像素。
3. 输出结构化可见格子消息（机器可用）和叠加图像（人可视化，当前默认窗口显示）。

这套设计使上层任务可以直接基于格子 ID 与相机系位置进行目标筛选、ROI提取与语义决策。

## Layer3 感知空间变换层（Sensor Spatial Transform）

本层目标：在不改动 Layer1（雷达重定位）和 Layer2（AR 投影主逻辑）的前提下，把“雷达位姿 + 云台动态角度 + 机械静态外参”融合为相机全局位姿，供 AR 层直接消费。

### 0. 代码速查跳转（Layer3）

- 新增功能包：`ar_calculate/src/sensor_spatial_transform`
- 动态 TF 节点：`sensor_spatial_transform/gimbal_tf_handler.py`
- 相机位姿适配节点：`sensor_spatial_transform/ar_pose_adapter.py`
- 启动文件：`launch/sensor_spatial_transform.launch.py`
- 参数文件：`config/sensor_spatial_transform.params.yaml`
- AR 侧 Layer3 参数模板：`ar_calculate/src/ar_grid_detector/config/ar_grid.layer3.params.yaml`

---

### 1. Layer3 架构边界与不影响原则

1. **不改 Layer1**

- 继续使用 `/aft_mapped_in_map` 作为雷达全局里程计来源。
- 不修改 FAST-LIVO、relocalization 的计算与发布逻辑。

2. **不改 Layer2 算法主链**

- AR 节点依旧接收 `Odometry` + `Image`，并按既有投影链路计算。
- 只把 `odom_topic` 从雷达位姿切换为“相机全局位姿”时使用 Layer3 输出。

3. **Layer3 只做适配与合成**

- 输入：电机控制角度（pitch/yaw）与 TF 树。
- 输出：`/camera_pose_in_map`（`nav_msgs/Odometry`）。

---

### 2. Layer3 TF 链路设计

本层按机械连接关系构造如下链路：

- `map -> lidar_link`：由 Layer1 提供（全局定位）
- `lidar_link -> motor_2006_base`：静态外参1
- `motor_2006_base -> motor_2006_shaft`：动态 Pitch（绕 Y）
- `motor_2006_shaft -> motor_scs0009_base`：静态外参2
- `motor_scs0009_base -> camera_link`：动态 Yaw（绕 Z）
- `camera_link -> camera_color_optical_frame`：静态外参3

最终由 `ar_pose_adapter` 查询：

$$
T_{map\rightarrow camera}=T_{map\rightarrow lidar}\cdot T_{lidar\rightarrow motor2006base}\cdot T_{pitch}\cdot T_{shaft\rightarrow scs}\cdot T_{yaw}\cdot T_{camera\rightarrow optical}
$$

---

### 3. 节点职责与话题协议

#### 3.1 `gimbal_tf_handler`

- 订阅：`/gimbal/target_cmd`（`geometry_msgs/Vector3`）
  - `x` = pitch(rad)
  - `y` = yaw(rad)
- 可选订阅：`/gimbal/ack`（`std_msgs/Bool`）
  - `require_ack=true` 时，收到 `true` 才应用目标角
- 持续发布 TF（默认 50Hz）：
  - 静态三段 + 动态两段

参数化要点：

- `*_frame`：全部坐标系名称可配置
- `*_xyz` / `*_rpy`：静态外参可配置
- `pitch_min/max`、`yaw_min/max`：动态角限幅

#### 3.2 `ar_pose_adapter`

- 查询 TF：`map -> camera_color_optical_frame`
- 发布：`/camera_pose_in_map`（`nav_msgs/Odometry`）
- 发布频率：默认 50Hz
- 异常处理：TF 查找失败时节流 warning，不发布错误位姿

---

### 4. 参数与运行方式

#### 4.1 Layer3 参数文件

文件：`ar_calculate/src/sensor_spatial_transform/config/sensor_spatial_transform.params.yaml`

关键参数：

- 话题：`cmd_topic`、`ack_topic`、`odom_topic`
- 坐标系：`map_frame`、`lidar_frame`、`camera_frame`
- 静态外参：
  - `lidar_to_motor_2006_base_*`
  - `motor_2006_shaft_to_scs0009_base_*`
  - `camera_link_to_optical_*`

#### 4.2 启动 Layer3

```bash
cd /home/r1/9_grid_ar_detection/ar_calculate
source install/setup.bash
ros2 launch sensor_spatial_transform sensor_spatial_transform.launch.py
```

#### 4.3 AR 切换到 Layer3 输出（可选）

使用参数模板：

- `ar_calculate/src/ar_grid_detector/config/ar_grid.layer3.params.yaml`

其核心改动：

- `odom_topic: /camera_pose_in_map`
- `extrinsic_matrix_4x4 = I`（单位阵）

---

### 5. 一键脚本接入（默认不启用）

`ar_calculate/start_all.sh` 已新增可选开关：

- `ENABLE_SENSOR_SPATIAL_TRANSFORM=1` 时启动 Layer3
- 默认 `0`，即不影响你当前既有链路

示例：

```bash
cd /home/r1/9_grid_ar_detection/ar_calculate
ENABLE_SENSOR_SPATIAL_TRANSFORM=1 ENABLE_AR_DETECTOR=1 ./start_all.sh
```

---

### 6. Layer3 小结

第三层本质是“姿态适配层”而非“定位层”：

1. Layer1 继续负责全局定位。
2. Layer2 继续负责几何投影。
3. Layer3 仅负责把机械运动补偿进相机全局位姿。

因此可以在不破坏原有功能的情况下，引入云台/电机动态补偿能力。
