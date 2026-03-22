# AR 定位系统使用指南（开发者见understand.md)

## 1. 系统概述

本系统通过 **激光雷达 + 相机 (绝对绝对的标准内参)+ IMU** 的多传感器融合方案，实现精确的 AR 定位目标区域功能。系统能够在真实世界中定义一个区域，并实时叠加区域格子显示在相机画面上。

### 1.1 系统架构

```
┌────────────────────────────────────────────────────────────────────────┐
│                         AR 定位系统                               │
├────────────────────────────────────────────────────────────────────────┤
│                                                                        │
│  ┌──────────────┐    ┌──────────────────┐    ┌─────────────────────┐  │
│  │   Livox      │    │   FAST-LIVO2     │    │   Relocalization    │  │
│  │   MID360     │───>│   激光-视觉-惯性  │───>│   重定位模块        │  │
│  │   雷达驱动   │    │   里程计建图      │    │   TEASER++ + GICP   │  │
│  └──────────────┘    └──────────────────┘    └─────────────────────┘  │
│         │                    │                         │              │
│         │                    ▼                         │              │
│         │           ┌──────────────────┐               │              │
│         │           │ /aft_mapped_in_map│               │              │
│         │        		<──────────┘              │
│         │           └──────────────────┘                              │
│         │                    │                                        │
│         │                    ▼                                        │
│  ┌──────────────┐    ┌──────────────────┐    ┌─────────────────────┐  │
│  │   RealSense  │    │   AR Overlay     │    │   九宫格世界坐标    │  │
│  │   D435       │───>│   叠加显示节点   │<───│   配置文件          │  │
│  │   相机驱动   │    │                  │    │                     │  │
│  └──────────────┘    └──────────────────┘    └─────────────────────┘  │
│                              │                                        │
│                              ▼                                        │
│                     ┌──────────────────┐                              │
│                     │ /ar_overlay/image│                              │
│                     │ AR 叠加输出图像  │                              │
│                     └──────────────────┘                              │
│                                                                        │
└────────────────────────────────────────────────────────────────────────┘
```

### 1.2 核心原理

#### 步骤一：建立世界坐标系（建图）

1. **FAST-LIVO2** 融合激光雷达点云、相机图像和 IMU 数据
2. 通过 SLAM 算法实时估计传感器在世界坐标系中的 6DoF 位姿
3. 同时构建环境的 3D 点云地图

#### 步骤二：重定位（通过先验地图匹配，无视启动位置的干扰）

1. 如果系统重启或移动到新位置，**Relocalization** 模块启动
2. **TEASER++** 进行全局特征匹配，估计粗略位姿（无需初始值）
3. **small_gicp** 进行精细配准，获得高精度位姿

#### 步骤三：通过雷达获取先验地图下的要投影的目标区域的世界坐标

#### 步骤四：第三层感知空间变换（可选，推荐用于云台/电机场景）

1. `gimbal_tf_handler` 接收 `/gimbal/target_cmd`（pitch/yaw），发布云台动态 TF。
2. 在机械静态外参的基础上，把雷达位姿与云台角度合成为完整 `map -> camera_color_optical_frame`。
3. `ar_pose_adapter` 将该 TF 输出为 `/camera_pose_in_map`（`Odometry`）。

#### 步骤五：AR 叠加显示

1. **ar_overlay_node** 实时读取里程计位姿 $T_{W}^{S}$（世界→传感器）
2. 结合相机-雷达外参 $T_{C}^{S}$，计算相机在世界坐标系的位姿
3. 将目标世界坐标通过针孔相机模型投影到图像像素坐标
4. 在相机画面上绘制标记点和连线

### 1.3 数学原理

**坐标变换链：**

$$
\mathbf{p}_{\text{camera}} = T_C^S \cdot (T_W^S)^{-1} \cdot \mathbf{p}_{\text{world}}
$$

其中：

- $\mathbf{p}_{\text{world}}$：九宫格点的世界坐标 (4×1 齐次坐标)
- $T_W^S$：世界→传感器（雷达）的变换矩阵，由 FAST-LIVO2 实时输出
- $T_C^S$：传感器（雷达）→相机的外参矩阵（标定得到）
- $\mathbf{p}_{\text{camera}}$：九宫格点在相机坐标系下的坐标

若启用第三层，AR 可直接订阅 `/camera_pose_in_map`，此时配置单位外参矩阵后有：

$$
\mathbf{p}_{\text{camera}} = (T_W^C)^{-1} \cdot \mathbf{p}_{\text{world}}
$$

**针孔投影模型：**

$$
\begin{bmatrix} u \\ v \end{bmatrix} = \begin{bmatrix} f_x \cdot \frac{x_c}{z_c} + c_x \\ f_y \cdot \frac{y_c}{z_c} + c_y \end{bmatrix}
$$

其中 $(f_x, f_y, c_x, c_y)$ 为相机内参。

---

## 2. 硬件要求

| 设备     | 型号                       | 用途                                |
| -------- | -------------------------- | ----------------------------------- |
| 激光雷达 | Livox MID360               | 3D 点云采集、环境感知               |
| 相机     | Intel RealSense D435       | RGB 图像采集、AR 显示（准确的内参） |
| IMU      | MID360 内置                | 惯性测量、运动估计                  |
| 主机     | Ubuntu 22.04 + ROS2 Humble | 算法运行平台                        |

---

## 3. 软件依赖

### 3.1 已安装组件

```bash
# 基础 ROS2 包
ros-humble-pcl-ros
ros-humble-octomap-ros
ros-humble-image-transport-plugins
ros-humble-sophus

# 相机驱动
ros-humble-realsense2-camera

# 本项目包含
FAST-LIVO2              # 激光-视觉-惯性里程计
livox_ros_driver2       # Livox 雷达驱动
relocalization          # 重定位模块
small_gicp              # 高效 GICP 配准
TEASER-plusplus         # 全局鲁棒配准
ar_calculate            # AR 九宫格叠加
```

### 3.2 核心组件详解

本系统的技术核心由三个主要组件构成，它们协同工作形成完整的定位-重定位-显示链路。

#### 3.2.1 Livox 雷达驱动 (livox_ros_driver2)

**功能定位：**

- Livox 雷达驱动是系统的**底层传感器接口**，负责与 Livox MID360 激光雷达硬件通信
- 实时采集原始点云数据和 IMU 测量数据，并发布为 ROS2 话题供上层算法使用

**关键输出话题：**

- `/livox/lidar`：PointCloud2 格式的 3D 点云数据（~19968 点/帧，10Hz）
- `/livox/imu`：IMU 传感器数据（加速度、角速度，200Hz）

**核心配置：**

- 雷达 IP 地址配置（默认 `192.168.1.146`）
- 端口映射（点云数据端口、IMU 数据端口等）
- 扫描模式（非重复扫描/重复扫描）
- 点云数据类型（笛卡尔坐标/球坐标）

**在系统中的角色：**

```
硬件层：MID360 激光雷达
    ↓ （以太网通信）
驱动层：livox_ros_driver2
    ↓ （ROS2 话题）
算法层：FAST-LIVO2 / Relocalization
```

驱动层是硬件和算法的桥梁，确保点云数据的稳定采集和精确时间戳同步。没有稳定的驱动层，后续所有定位算法都无法正常工作。

---

#### 3.2.2 FAST-LIVO2 激光-视觉-惯性里程计

**功能定位：**

- FAST-LIVO2 是系统的**实时定位引擎**，采用紧耦合的多传感器融合 SLAM 算法
- 融合激光雷达点云、相机图像和 IMU 数据，实时估计传感器在世界坐标系中的 6DoF 位姿（位置+姿态）
- 同时构建环境的 3D 点云地图，用于后续的重定位和地图复用

**核心技术特点：**

1. **iKD-Tree 增量式点云地图**：高效的动态空间索引结构，支持快速点云插入和最近邻搜索
2. **IEKF（迭代扩展卡尔曼滤波）**：紧耦合融合激光、视觉、IMU 三种传感器数据
3. **视觉特征辅助**：利用图像特征增强纹理丰富区域的定位精度
4. **IMU 预积分**：处理高频 IMU 数据，提供运动预测和初值

**关键输出：**

- `/aft_mapped_in_map`：Odometry 消息，包含传感器当前时刻相对于先验地图初始坐标系（世界坐标系）的位姿 $T_W^S$
- `/cloud_registered`：配准后的点云地图
- `/path`：传感器运动轨迹

**在系统中的角色：**

```
输入：
  ├─ /livox/lidar    （点云）
  ├─ /livox/imu      （惯性）
  └─ /camera/image   （图像）
        ↓
  【FAST-LIVO2 SLAM】
        ↓
输出：
  ├─ /aft_mapped_in_map  → AR 叠加节点使用
  └─ /cloud_registered    → 保存为先验地图
```

FAST-LIVO2 是系统的"大脑"，它告诉系统"我在哪里"。AR 九宫格叠加节点依赖此位姿信息将虚拟内容正确投影到现实世界。

---

#### 3.2.3 TEASER++GICP 重定位模块

**功能定位：**

- 重定位模块是系统的**全局定位恢复机制**，解决 SLAM 系统重启或跨会话定位问题
- 当系统重新启动或移动到已建图区域的新位置时，需要将当前传感器位姿**对齐到先验地图坐标系**

**两阶段配准流程：**

**第一阶段 - TEASER++（全局粗配准）：**

- **作用**：在无初始值的情况下，通过鲁棒的特征匹配估计粗略位姿
- **算法原理**：基于 FPFH（Fast Point Feature Histograms）特征描述符的全局配准，使用 GNC-TLS（Graduated Non-Convexity Truncated Least Squares）求解器剔除外点
- **优势**：对噪声和外点具有极强鲁棒性，不需要初始位姿猜测
- **输出**：粗略的 4x4 变换矩阵 $T_{rough}$（精度约 10-30cm）

**第二阶段 - small_gicp（局部精配准）：**

- **作用**：在 TEASER++ 提供的初值基础上，进行精细的 ICP（Iterative Closest Point）配准
- **算法原理**：基于点到面的距离度量，迭代优化位姿使点云配准误差最小化
- **优势**：收敛速度快，精度高（可达 1-2cm）
- **输出**：精确的 4x4 变换矩阵 $T_{precise}$

**重定位成功判据：**

- TEASER++ 阶段：内点数量 > `teaser_success_count`
- GICP 阶段：配准适应度分数 < `fitness_score_thre`，连续收敛次数 > `converged_count_thre`

**在系统中的角色：**

```
场景 1：首次建图
  FAST-LIVO2 → 从零开始构建世界坐标系 → 保存地图 .pcd

场景 2：跨会话重定位（使用重定位）
  加载先验地图 .pcd
        ↓
  当前点云 + 先验地图
        ↓
  【TEASER++ 全局配准】→ 粗略位姿
        ↓
  【small_gicp 精配准】→ 精确位姿 T_relocalize
        ↓
  发布为 /relocalize_transform
        ↓
  FAST-LIVO2 订阅并初始化世界坐标系
```

重定位模块让 SLAM 系统具备"记忆"功能，可以识别"这是我来过的地方"，并将自己定位到之前建立的世界坐标系中。

---

#### 3.2.4 三者之间的协作关系

**完整数据流图：**

```
┌─────────────────────────────────────────────────────────────────────┐
│                        系统完整工作流程                              │
└─────────────────────────────────────────────────────────────────────┘

【硬件采集层】
  Livox MID360 (雷达+IMU) + RealSense D435 (相机)
        ↓
  livox_ros_driver2
        ├─→ /livox/lidar (点云 10Hz)
        └─→ /livox/imu   (惯性 200Hz)

【核心定位层】
  FAST-LIVO2 (实时 SLAM)
        ├─ 输入：点云 + IMU + 图像
        ├─ 输出：/aft_mapped_to_init (位姿)
        └─ 副产物：点云地图 → 保存为 .pcd

【重定位层】（可选，跨会话时启用）
  先验地图 .pcd + 当前点云
        ↓
  TEASER++ (粗配准) → small_gicp (精配准)
        ↓
  /relocalize_transform → FAST-LIVO2 初始化

【应用层】
  AR Overlay Node
        ├─ 订阅：/aft_mapped_in_map(位姿)
        ├─ 订阅：/camera/image (图像)
        ├─ 读取：九宫格世界坐标 .yaml
        └─ 输出：/ar_overlay/image (叠加后图像)
```

**关键依赖关系：**

1. **FAST-LIVO2 依赖 livox_ros_driver2**

   - 无雷达驱动 → 无点云fdf数据 → SLAM 无法运行
   - 时间戳不同步 → IMU/LiDAR 融合失败 → 定位漂移
2. **Relocalization 依赖 FAST-LIVO2**

   - 需要 FAST-LIVO2 提供当前点云 `/cloud_registered`
   - 重定位成功后，FAST-LIVO2 使用重定位结果初始化世界坐标系
3. **AR Overlay 依赖 FAST-LIVO2**

   - 完全依赖 `/aft_mapped_in_map` 的位姿精度
   - FAST-LIVO2 漂移 → AR 九宫格错位

**工作模式对比：**

| 模式                 | 组件启动                             | 适用场景                           | 世界坐标系来源      |
| -------------------- | ------------------------------------ | ---------------------------------- | ------------------- |
| **纯建图**     | Driver + FAST-LIVO2                  | 首次使用，建立新地图               | FAST-LIVO2 启动时刻 |
| **重定位建图** | Driver + FAST-LIVO2 + Relocalization | 在已知区域重新定位                 | 先验地图坐标系      |
| 完整 AR + 重定位     | 全部启动跨会话 AR                    | 使用先验地图坐标系，可改动启动位置 | 先验地图坐标系      |

**典型使用流程：**

```
第一次使用（建图）：
  1. 启动 Driver + FAST-LIVO2
  2. 移动传感器建图
  3. 保存地图 .pcd
  4. 标定九宫格世界坐标（基于此次建图的世界坐标系）
  5. 启动 AR Overlay 查看效果

第二次使用（重定位+AR）：
  1. 启动 Driver + FAST-LIVO2
  2. 启动 Relocalization（加载上次保存的 .pcd）
  3. 等待重定位成功（FAST-LIVO2 通过tf变换，发布到先验地图的原点的坐标变换）
  4. 启动 AR Overlay
```

---

#### 3.2.5 第三层：感知空间变换层（Sensor Spatial Transform）

该层用于“雷达和相机存在可变机械关系（电机/云台）”的场景，目标是在不改 Layer1/Layer2 主逻辑下输出相机全局位姿。

**新增节点：**

- `gimbal_tf_handler`：订阅 `/gimbal/target_cmd`，发布动态 TF（Pitch/Yaw）和静态机械外参链。
- `ar_pose_adapter`：查询 `map -> camera_color_optical_frame`，发布 `/camera_pose_in_map`。

**默认配置文件：**

- `ar_calculate/src/sensor_spatial_transform/config/sensor_spatial_transform.params.yaml`
- `ar_calculate/src/ar_grid_detector/config/ar_grid.layer3.params.yaml`

**启用效果：**

- Layer1 仍发布 `/aft_mapped_in_map`。
- Layer3 合成相机位姿 `/camera_pose_in_map`。
- Layer2 把 `odom_topic` 切换到 `/camera_pose_in_map` 即可获得动态补偿投影。

---

## 4. 使用流程

### 4.1 第一步快速配置依赖指南

### 4.2 第二步：准备先验地图和世界坐标

#### 方法 A：手动测量（需要场地的标准尺寸，使用先验地图的原点来算）

#### 方法 B：通过 SLAM 实时标定（推荐）

**原理：** 通过 SLAM 系统实时估计的位姿，记录物理空间中九宫格关键点的世界坐标，自动生成配置文件。

##### 第 1 步：启动 SLAM 建图系统

打开一个终端，运行一键启动脚本（或禁用 AR 模块以减少干扰）：

```bash
cd /home/r1/9_grid_ar_detection/ar_calculate(改一下路径)
ENABLE_AR_OVERLAY=0 ./start_all.sh
```

##### 第 2 步：启动世界坐标记录工具

使用record_lidar_poses.py，可以采集任意多的点

### 4.3 第三步：配置参数

### 4.4 第四步：启动系统

#### 方式 A：一键启动脚本

```bash
cd /home/r1/Slam/ar_calculate
./start_all.sh
```

该脚本会依次启动：

1. RealSense 相机驱动
2. Livox 雷达驱动
3. FAST-LIVO2 里程计
4. TEASER++GICP 重定位
5. Layer3 感知空间变换（可选，默认关闭）
6. AR投影计算

启用第三层示例：

```bash
cd /home/r1/9_grid_ar_detection/ar_calculate
ENABLE_SENSOR_SPATIAL_TRANSFORM=1 \
AR_GRID_LAUNCH_CMD="ros2 launch ar_grid_detector ar_grid.launch.py params_file:=/home/r1/9_grid_ar_detection/ar_calculate/src/ar_grid_detector/config/ar_grid.layer3.params.yaml" \
./start_all.sh
```

#### 方式 B：手动分步启动（推荐调试时使用）

**终端 1 - 雷达驱动：**

```bash
cd /home/r1/Slam/FAST_LIVO2_ROS2_relocation_ultra
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

**终端 2 - 相机驱动：**

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=false \
  rgb_camera.color_profile:=640x480x30
```

**终端 3 - FAST-LIVO2 里程计：**

```bash
cd /home/r1/Slam/FAST_LIVO2_ROS2_relocation_ultra
source install/setup.bash
ros2 launch fast_livo mapping_avia.launch.py use_rviz:=True
```

**终端 4 - 重定位（可选，跨场景时启用）：**

```bash
cd /home/r1/Slam/FAST_LIVO2_ROS2_relocation_ultra
source install/setup.bash
ros2 launch example_teaser_gicp.launch.py
```

**终端 5 - Layer3（可选）：**

```bash
cd /home/r1/9_grid_ar_detection/ar_calculate
source install/setup.bash
ros2 launch sensor_spatial_transform sensor_spatial_transform.launch.py
```

**终端 6 - AR 叠加显示：**

```bash
cd /home/r1/Slam/ar_calculate
python3 ar_overlay_node.py --ros-args --params-file ar_overlay.params.yaml
```

或使用 launch 文件：

```bash
ros2 launch ar_overlay.launch.py params_file:=/home/r1/Slam/ar_calculate/ar_overlay.params.yaml
```

### 4.5 第五步：查看结果

**本地窗口显示**：如果 `show_window: true`，会弹出 OpenCV 窗口

### 4.6 第六步：如何修改配置（实战）

本方案主要涉及 4 份配置文件，建议按“重定位 → Layer3 外参 → AR 参数”的顺序修改。

#### A. 雷达重定位相关主配置

##### 1.MID360配置

##### 2. fast-livo2 参数配置

##### 3. TEASER + GICP 启动参数

文件：`/home/r1/Slam/FAST_LIVO2_ROS2_relocation_ultra/example_teaser_gicp.launch.py`

#### B. AR 叠加配置(内外参必须要准确)

文件：`/home/r1/Slam/ar_calculate/ar_grid.params.yaml`

#### C. Layer3 感知空间变换配置（启用云台补偿时）

文件：`/home/r1/9_grid_ar_detection/ar_calculate/src/sensor_spatial_transform/config/sensor_spatial_transform.params.yaml`

关键参数：

- `lidar_to_motor_2006_base_*`
- `motor_2006_shaft_to_scs0009_base_*`
- `camera_link_to_optical_*`
- `camera_frame`（默认 `camera_color_optical_frame`）

AR 使用 Layer3 的参数模板：

- `/home/r1/9_grid_ar_detection/ar_calculate/src/ar_grid_detector/config/ar_grid.layer3.params.yaml`

重点参数：

- 常见修改场景：

#### 修改后如何生效

—— 一定要重新编译然后source

修改后快速自检

```bash
# 1) 核对话题是否存在
ros2 topic list | grep -E "aft_mapped_to_init|camera/color/image_raw|livox/lidar|livox/imu"

# 2) 核对相机分辨率/内参
ros2 topic echo /camera/color/camera_info --once

# 3) 核对里程计是否正常
ros2 topic echo /aft_mapped_to_init --once

# 4) 核对 AR 输出
ros2 topic hz /ar_overlay/image
```

建议每次只改一类参数并记录结果，避免多处同时改动导致难以定位问题。

---

## 5. 九宫格编号说明

```
          ← X 方向（1→3）→
  
    ┌─────┬─────┬─────┐   ↑
    │  1  │  2  │  3  │   │
    ├─────┼─────┼─────┤   Y 方向
    │  4  │  5  │  6  │   （1→7）
    ├─────┼─────┼─────┤   │
    │  7  │  8  │  9  │   ↓
    └─────┴─────┴─────┘

    1 = 左上    3 = 右上
    7 = 左下    9 = 右下
    5 = 中心
```

---

## 6. 常见问题排查（见issues.md）

## 7. 文件结构说明

## 8. 参考资料

- [FAST-LIVO2 论文](https://github.com/hku-mars/FAST-LIVO2)
- [TEASER++ 论文](https://github.com/MIT-SPARK/TEASER-plusplus)
- [small_gicp 库](https://github.com/koide3/small_gicp)
- [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2)
- [RealSense ROS 2](https://github.com/IntelRealSense/realsense-ros)
