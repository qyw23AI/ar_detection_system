# AR 定位九宫格系统使用指南

## 1. 系统概述

本系统通过 **激光雷达 + 相机 + IMU** 的多传感器融合方案，实现精确的 AR 定位九宫格功能。系统能够在真实世界中定义一个九宫格区域，并将其实时叠加显示在相机画面上。

### 1.1 系统架构

```
┌────────────────────────────────────────────────────────────────────────┐
│                         AR 定位九宫格系统                               │
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
│         │           │ /aft_mapped_to_  │               │              │
│         │           │ init (里程计位姿) │<──────────────┘              │
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

#### 步骤二：重定位（可选，用于跨场景）

1. 如果系统重启或移动到新位置，**Relocalization** 模块启动
2. **TEASER++** 进行全局特征匹配，估计粗略位姿（无需初始值）
3. **small_gicp** 进行精细配准，获得高精度位姿

#### 步骤三：九宫格世界坐标定义

1. 在世界坐标系中选取 3 个关键点（如 1号、3号、7号 或 3号、9号、7号）
2. `nine_grid.py` 根据 3 点坐标和格子间距，计算出完整的 9 个格子中心点
3. 结果保存为 YAML 配置文件

#### 步骤四：AR 叠加显示

1. **ar_overlay_node** 实时读取里程计位姿 $T_{W}^{S}$（世界→传感器）
2. 结合相机-雷达外参 $T_{C}^{S}$，计算相机在世界坐标系的位姿
3. 将九宫格世界坐标通过针孔相机模型投影到图像像素坐标
4. 在相机画面上绘制九宫格标记点和连线

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

**针孔投影模型：**

$$
\begin{bmatrix} u \\ v \end{bmatrix} = \begin{bmatrix} f_x \cdot \frac{x_c}{z_c} + c_x \\ f_y \cdot \frac{y_c}{z_c} + c_y \end{bmatrix}
$$

其中 $(f_x, f_y, c_x, c_y)$ 为相机内参。

---

## 2. 硬件要求

| 设备     | 型号                       | 用途                  |
| -------- | -------------------------- | --------------------- |
| 激光雷达 | Livox MID360               | 3D 点云采集、环境感知 |
| 相机     | Intel RealSense D435       | RGB 图像采集、AR 显示 |
| IMU      | MID360 内置                | 惯性测量、运动估计    |
| 主机     | Ubuntu 22.04 + ROS2 Humble | 算法运行平台          |

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

- `/aft_mapped_to_init`：Odometry 消息，包含传感器当前时刻相对于初始坐标系（世界坐标系）的位姿 $T_W^S$
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
  ├─ /aft_mapped_to_init  → AR 叠加节点使用
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
场景 1：首次建图（不使用重定位）
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
        ├─ 订阅：/aft_mapped_to_init (位姿)
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

   - 完全依赖 `/aft_mapped_to_init` 的位姿精度
   - FAST-LIVO2 漂移 → AR 九宫格错位

**工作模式对比：**

| 模式                       | 组件启动                             | 适用场景                          | 世界坐标系来源        |
| -------------------------- | ------------------------------------ | --------------------------------- | --------------------- |
| **纯建图**           | Driver + FAST-LIVO2                  | 首次使用，建立新地图              | FAST-LIVO2 启动时刻   |
| **重定位建图**       | Driver + FAST-LIVO2 + Relocalization | 在已知区域重新定位                | 先验地图坐标系        |
| **完整 AR**          | Driver + FAST-LIVO2 + AR Overlay     | 实时 AR 显示                      | FAST-LIVO2 当前坐标系 |
| **完整 AR + 重定位** | 全部启动                             | 跨会话 AR，需恢复到先验地图坐标系 | 先验地图坐标系        |

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
  3. 等待重定位成功（FAST-LIVO2 自动切换到先验坐标系）
  4. 启动 AR Overlay（九宫格坐标自动对齐）
```

---

## 4. 使用流程

### 快速选择指南

在完善九宫格世界坐标时，选择最适合你的方法：

| 方法                            | 优点                                     | 缺点                                     | 适用场景                           |
| ------------------------------- | ---------------------------------------- | ---------------------------------------- | ---------------------------------- |
| **方法 A：手动测量**      | 不需要启动 SLAM；简单快速；无需编程      | 需要精确测量工具；易出错；无法自适应环境 | 相对简单、稳定的场景；快速原型     |
| **方法 B：SLAM 实时标定** | 高精度；自适应环境；记录完整点云；可重复 | 需启动 SLAM；耗时较长；需要学习工具使用  | 生产环境；需要高精度；环境动态变化 |

**推荐：**

- 🔰 首次使用：任选其一，推荐 **方法 A** 快速验证
- 🎯 生产环境：**方法 B**（精度高，可重复标定）

---

### 4.1 第一步：准备九宫格世界坐标

#### 方法 A：手动测量（适合快速验证）

1. 在实际场地确定九宫格区域
2. 使用卷尺测量 3 个关键点的世界坐标
3. 编辑 JSON 配置文件：

```bash
cd /home/r1/Slam/ar_calculate
cp nine_grid_input_template_unified.json my_grid_config.json
```

编辑 `my_grid_config.json`：

```json
{
  "point_mode": "3-9-7",
  "size_mode": "cell",
  "grid_size": 0.5,
  "points": {
    "p3": [2.0, 0.0, 0.5],
    "p9": [2.0, 1.0, 0.5],
    "p7": [1.0, 1.0, 0.5]
  }
}
```

**参数说明：**

- `point_mode`:
  - `"3-9-7"`: 使用右上(3)、右下(9)、左下(7) 三点
  - `"1-3-7"`: 使用左上(1)、右上(3)、左下(7) 三点
- `size_mode`:
  - `"cell"`: `grid_size` 表示相邻格子中心间距（米）
  - `"total"`: `grid_size` 表示整体尺寸 [宽, 高]
- `grid_size`: 0.5 表示每格中心间距 50cm

4. 生成 YAML 配置：

```bash
python3 nine_grid.py --input-json my_grid_config.json --output-yaml nine_grid_points.yaml
```

#### 方法 B：通过 SLAM 实时标定（推荐）

**原理：** 通过 SLAM 系统实时估计的位姿，记录物理空间中九宫格关键点的世界坐标，自动生成配置文件。

**适用场景：**

- 无法精确物理测量的场景
- 需要高精度相对位置关系的应用
- 环境已建图或有参考地图的情况

##### 第 1 步：启动 SLAM 建图系统

打开一个终端，运行一键启动脚本（或禁用 AR 模块以减少干扰）：

```bash
cd /home/r1/Slam/ar_calculate
ENABLE_AR_OVERLAY=0 ./start_all_with_offset.sh
```

或禁用重定位（纯建图）：

```bash
ENABLE_RELOCALIZATION=0 ENABLE_AR_OVERLAY=0 ./start_all_with_offset.sh
```

**等待系统完全启动：** 观察控制台，确保看到：

```
  ✓ Camera driver:      ENABLED
  ✓ Livox driver:       ENABLED
  ✓ FAST-LIVO2:         ENABLED
  ✓ /aft_mapped_to_init  (已发布话题)
```

##### 第 2 步：启动位姿记录工具

打开另一个终端，进入 AR 计算目录并启动位姿记录工具：

```bash
cd /home/r1/Slam/ar_calculate
# 先确保环境已配置
source install/local_setup.bash

# 启动交互式位姿记录工具
python3 record_lidar_poses.py
```

**工具启动成功的标志：**

```
════════════════════════════════════════════════════
         雷达位姿录制工具
════════════════════════════════════════════════════
订阅话题: /aft_mapped_to_init

等待位姿消息... ✓
```

##### 第 3 步：记录九宫格关键点

在位姿记录工具的交互菜单中，按以下步骤操作：

**1) 选择点位模式**

在菜单中选择 **选项 4**，然后选择标定模式：

- **选项 1：使用 1-3-7**

  ```
  点 1（左上） → 点 3（右上） → 点 7（左下）
  ```
- **选项 2：使用 3-9-7**

  ```
  点 3（右上） → 点 9（右下） → 点 7（左下）
  ```

以 **1-3-7** 为例：

**2) 移动雷达到点 1（左上）**

```
在实际九宫格区域左上角放置雷达，确保其稳定。
在工具菜单选择：选项 2 → 输入点号：1
工具会显示当前位置并提示确认。
```

位姿记录工具会实时显示：

```
当前位置: X=  1.0123  Y=  2.0456  Z=  0.5234
确认记录点 1? (y/n): y
✓ 已记录点 1: [1.0123, 2.0456, 0.5234]
```

**3) 重复步骤 2，记录点 3 和点 7**

```
移动到点 3（右上）
  → 选项 2 → 3 → y

移动到点 7（左下）
  → 选项 2 → 7 → y
```

**4) 查看已记录的点位**

在工具菜单选择 **选项 3**，确认所有点位都已正确记录：

```
════════════════════════════════════════════════════
           已记录的点位
════════════════════════════════════════════════════
点 1: X=  1.0123  Y=  2.0456  Z=  0.5234
点 3: X=  1.4234  Y=  2.0456  Z=  0.5234
点 7: X=  1.0123  Y=  2.4567  Z=  0.5234
```

##### 第 4 步：生成 JSON 配置文件

在工具菜单选择 **选项 4** 生成配置文件：

```
选择操作: 4
选择点位模式: 1 (使用 1-3-7)
输入网格大小 (保持 cell 模式，单位米): 0.2
输入输出文件名 (默认自动生成): 
✓ 配置文件已生成: /home/r1/Slam/ar_calculate/nine_grid_calibration_20260306_165000.json
```

**生成的 JSON 文件示例：**

```json
{
  "point_mode": "1-3-7",
  "size_mode": "cell",
  "grid_size": 0.2,
  "points": {
    "p1": [1.0123, 2.0456, 0.5234],
    "p3": [1.4234, 2.0456, 0.5234],
    "p7": [1.0123, 2.4567, 0.5234]
  },
  "recorded_at": "2026-03-06T16:53:31.234567",
  "all_recorded_points": {
    "p1": [1.0123, 2.0456, 0.5234],
    "p3": [1.4234, 2.0456, 0.5234],
    "p7": [1.0123, 2.4567, 0.5234]
  }
}
```

##### 第 5 步：转换为最终 YAML 配置

在第二个终端中运行转换工具：

```bash
cd /home/r1/Slam/ar_calculate
python3 calibrate_nine_grid.py nine_grid_calibration_20260306_165000.json
```

**交互输入：**

```
════════════════════════════════════════════════════
       九宫格标定快速转换工具
════════════════════════════════════════════════════

加载配置文件: nine_grid_calibration_20260306_165000.json
✓ 配置文件验证成功

════════════════════════════════════════════════════
           配置摘要
════════════════════════════════════════════════════
点位模式: 1-3-7
大小模式: cell
网格大小: 0.2

关键点位:
  p1: X=  1.0123  Y=  2.0456  Z=  0.5234
  p3: X=  1.4234  Y=  2.0456  Z=  0.5234
  p7: X=  1.0123  Y=  2.4567  Z=  0.5234

是否继续生成 YAML 配置? (y/n): y
```

**脚本会自动调用 `nine_grid.py` 生成最终配置：**

```
✓ YAML 配置生成成功!

输出文件: /home/r1/Slam/ar_calculate/nine_grid_points.yaml

════════════════════════════════════════════════════
           标定完成!
════════════════════════════════════════════════════

✓ YAML 配置文件已生成: nine_grid_points.yaml

后续步骤:
  1. 查看生成的配置:
     cat nine_grid_points.yaml

  2. 启动完整系统 (启用 AR 叠加):
     ENABLE_AR_OVERLAY=1 ./start_all_with_offset.sh

  3. 在另一个终端查看 AR 输出:
     ros2 run rqt_image_view rqt_image_view /ar_overlay/image
```

**高级用法 - 覆盖网格大小：**

```bash
# 如果需要手动调整网格大小（例如改为 0.25 米）
python3 calibrate_nine_grid.py nine_grid_calibration_20260306_165000.json --grid-size 0.25

# 指定输出文件名
python3 calibrate_nine_grid.py nine_grid_calibration_20260306_165000.json \
  --grid-size 0.3 \
  --output-yaml nine_grid_points_v2.yaml
```

##### 第 6 步：验证配置（可选）

查看生成的 YAML 配置文件：

```bash
cat nine_grid_points.yaml
```

输出示例（9 个格子中心点的世界坐标）：

```yaml
nine_grid_points:
  p1: [1.0123, 2.0456, 0.5234]
  p2: [1.2179, 2.0456, 0.5234]
  p3: [1.4234, 2.0456, 0.5234]
  p4: [1.0123, 2.2512, 0.5234]
  p5: [1.2179, 2.2512, 0.5234]
  p6: [1.4234, 2.2512, 0.5234]
  p7: [1.0123, 2.4567, 0.5234]
  p8: [1.2179, 2.4567, 0.5234]
  p9: [1.4234, 2.4567, 0.5234]
```

##### 第 7 步：启动完整系统进行验证

结束之前的建图进程后，启动完整的 AR 叠加系统：

```bash
# 在 AR 计算目录
cd /home/r1/Slam/ar_calculate
ENABLE_AR_OVERLAY=1 ./start_all_with_offset.sh
```

在另一个终端查看 AR 实时输出：

```bash
ros2 run rqt_image_view rqt_image_view /ar_overlay/image
```

**验证九宫格是否正确显示：**

- ✓ 九宫格应该正确叠加在相机画面上
- ✓ 部分点可能超出相机视野（取决于相机位置）
- ✓ 网格线条应该与实际地面对齐

如果位置偏差较大，可以在方法 A 中【手动调整坐标】或重新执行方法 B。

##### 故障排除

| 问题                     | 原因                | 解决方案                                 |
| ------------------------ | ------------------- | ---------------------------------------- |
| 位姿记录工具无法启动     | ROS2 环境未配置     | `source install/local_setup.bash`      |
| 无法接收位姿消息         | SLAM 系统未正常运行 | 检查 FAST-LIVO2 是否已启动，查看日志     |
| 生成的九宫格位置严重偏差 | 标定点位置不准确    | 重新执行方法 B，确保点位放置准确         |
| 转换脚本报错             | JSON 文件格式错误   | 检查 JSON 文件内容，或重新用位姿工具录制 |

### 4.2 第二步：配置相机参数

编辑 `/home/r1/Slam/ar_calculate/d435.json`：

```json
{
  "camera_matrix": {
    "fx": 606.50,
    "fy": 605.77,
    "cx": 325.77,
    "cy": 256.54
  },
  "image_size": {
    "width": 640,
    "height": 480
  }
}
```

**获取相机内参的方法：**

```bash
# 启动相机，查看内参话题
ros2 launch realsense2_camera rs_launch.py
ros2 topic echo /camera/color/camera_info
```

### 4.3 第三步：配置雷达-相机外参

编辑 `/home/r1/Slam/ar_calculate/ar_overlay.params.yaml`：

```yaml
ar_overlay_node:
  ros__parameters:
    # 九宫格配置文件路径
    grid_yaml: /home/r1/Slam/ar_calculate/nine_grid_points.yaml
    camera_json: /home/r1/Slam/ar_calculate/d435.json

    # ROS 话题
    odom_topic: /aft_mapped_to_init
    image_topic: /camera/color/image_raw
    output_image_topic: /ar_overlay/image

    # 绘制参数
    draw_radius: 6
    point_color_bgr: [0, 0, 255]  # 红色点
    draw_grid_lines: true
    line_color_bgr: [255, 0, 0]   # 蓝色线
    show_labels: true
    show_window: true

    # 雷达→相机外参（4x4矩阵，按行展开）
    # 典型配置：雷达系(x前,y左,z上) → 相机光学系(x右,y下,z前)
    extrinsic_matrix_4x4: [
      0.0, -1.0, 0.0, 0.0,
      0.0, 0.0, -1.0, 0.0,
      1.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0
    ]
```

**外参矩阵说明：**

- 需要通过联合标定获取精确值
- 默认值假设雷达和相机光轴方向关系为标准配置
- 如果有物理偏移，需要在最后一列添加平移量 [tx, ty, tz, 1]

### 4.4 第四步：启动系统

#### 方式 A：一键启动脚本

```bash
cd /home/r1/Slam/ar_calculate
./start_all_with_offset.sh
```

该脚本会依次启动：

1. RealSense 相机驱动
2. Livox 雷达驱动
3. FAST-LIVO2 里程计
4. TEASER++GICP 重定位
5. 位置偏移发布器

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

**终端 5 - AR 叠加显示：**

```bash
cd /home/r1/Slam/ar_calculate
python3 ar_overlay_node.py --ros-args --params-file ar_overlay.params.yaml
```

或使用 launch 文件：

```bash
ros2 launch ar_overlay.launch.py params_file:=/home/r1/Slam/ar_calculate/ar_overlay.params.yaml
```

### 4.5 第五步：查看结果

1. **本地窗口显示**：如果 `show_window: true`，会弹出 OpenCV 窗口
2. **ROS 话题查看**：
   ```bash
   ros2 run rqt_image_view rqt_image_view
   # 选择 /ar_overlay/image 话题
   ```
3. **RViz 可视化**：在 RViz 中添加 Image 显示，订阅 `/ar_overlay/image`

### 4.6 第六步：如何修改配置（实战）

本方案主要涉及 3 份配置文件，建议按“话题 → 内参/外参 → 重定位参数”的顺序修改。

#### A. FAST-LIVO2 主配置

文件：`/home/r1/Slam/FAST_LIVO2_ROS2_relocation_ultra/src/FAST-LIVO2/config/avia_relocation.yaml`

重点参数：

- `common.img_topic`：相机图像话题，必须与相机驱动一致。
- `common.lid_topic` / `common.imu_topic`：雷达和 IMU 话题。
- `common.img_en`：是否启用视觉前端，这里一律关闭视觉前端。
- `camera.*`：相机内参和分辨率，必须与当前相机真实输出匹配。
- `extrin_calib.Rcl/Pcl`：雷达-相机外参（标定值）。
- `locate_in_prior_map`：是否启用先验地图定位。
- `prior_map_path`：先验地图 pcd 路径。

常见修改场景：

1. 保证这个是false

- `locate_in_prior_map: false`

2. **切换到另一台相机**

- 更新 `camera.width/height/fx/fy/cx/cy/d0~d3`
- 同时确认 `img_topic` 对应到新相机话题

3. **纯激光模式**

- `img_en: 0`

#### B. AR 叠加配置

文件：`/home/r1/Slam/ar_calculate/ar_overlay.params.yaml`

重点参数：

- `grid_yaml`：九宫格世界坐标文件路径。
- `camera_json`：相机内参 JSON 文件路径。
- `odom_topic`：里程计输入（一般 `/aft_mapped_to_init`）。
- `image_topic`：图像输入（一般 `/camera/color/image_raw`）。
- `extrinsic_matrix_4x4`：雷达到相机的 4x4 外参矩阵（优先级高于 `extrinsic_rpy_xyz`）。
- `show_window`：无桌面环境时建议 `false`。

常见修改场景：

1. **九宫格位置不准**

- 先检查 `grid_yaml` 是否是最新生成
- 再检查 `extrinsic_matrix_4x4` 是否为当前设备标定值

2. **看不到图像窗口**

- 服务器环境下将 `show_window: false`
- 用 `/ar_overlay/image` 话题在 RViz 或 `rqt_image_view` 查看

#### C. TEASER + GICP 启动参数

文件：`/home/r1/Slam/FAST_LIVO2_ROS2_relocation_ultra/example_teaser_gicp.launch.py`

重点参数：

- `map_path`：重定位地图路径。
- `map_voxel_leaf_size` / `cloud_voxel_leaf_size`：TEASER 阶段降采样。
- `gicp_map_voxel_leaf_size` / `gicp_cloud_voxel_leaf_size`：GICP 阶段降采样。
- `teaser_inlier_threshold` / `teaser_success_count`：TEASER 切换到 GICP 的门槛。
- `fitness_score_thre` / `converged_count_thre`：GICP 收敛判据。

调参建议（先快后准）：

- 初次全局重定位困难：适当增大 TEASER 体素和 `noise_bound`。
- 已有较好初值但精度不够：减小 GICP 体素，适当降低 `fitness_score_thre`。
- 计算太慢：增大体素、降低迭代次数、减少线程争用。

#### D. 修改后如何生效

```bash
cd /home/r1/Slam/FAST_LIVO2_ROS2_relocation_ultra
source /opt/ros/humble/setup.bash
colcon build --packages-select fast_livo relocalization
source install/setup.bash
```

若只改了 `ar_overlay.params.yaml`，通常无需重编译，直接重启对应节点即可。

#### E. 修改后快速自检

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

## 6. 常见问题排查

### 6.1 九宫格点不显示

**检查清单：**

1. 确认里程计话题有数据：
   ```bash
   ros2 topic echo /aft_mapped_to_init --once
   ```
2. 检查九宫格坐标是否在相机视野内
3. 验证外参矩阵方向是否正确

### 6.2 九宫格位置偏移

**可能原因：**

- 雷达-相机外参不准确
- 九宫格世界坐标测量误差
- SLAM 漂移

**解决方法：**

1. 重新进行联合标定
2. 使用更精确的测量工具
3. 启用重定位模块纠正漂移

### 6.3 雷达启动失败 (bind failed)

修改 `/home/r1/Slam/FAST_LIVO2_ROS2_relocation_ultra/src/livox_ros_driver2/config/MID360_config.json`：

```json
"host_net_info": {
  "cmd_data_port": 56100,
  "push_msg_port": 56200,
  "point_data_port": 56300,
  "imu_data_port": 56400,
  "log_data_port": 56500
}
```

确保：

1. 雷达通过以太网连接
2. 主机网卡配置为与雷达同网段（默认 192.168.1.x）
3. 没有端口冲突

### 6.4 图像压缩插件缺失

```bash
sudo apt install -y ros-humble-image-transport-plugins
```

---

## 7. 文件结构说明

```
/home/r1/Slam/
├── ar_calculate/                    # AR 九宫格叠加模块
│   ├── ar_overlay_node.py           # 主节点：订阅里程计和图像，输出叠加结果
│   ├── nine_grid.py                 # 九宫格坐标计算工具
│   ├── ar_overlay.params.yaml       # 节点参数配置
│   ├── nine_grid_points.yaml        # 九宫格世界坐标
│   ├── d435.json                    # 相机内参
│   └── start_all_with_offset.sh     # 一键启动脚本
│
├── FAST_LIVO2_ROS2_relocation_ultra/  # 主项目
│   ├── src/
│   │   ├── FAST-LIVO2/              # 激光-视觉-惯性里程计
│   │   ├── livox_ros_driver2/       # Livox 雷达驱动
│   │   ├── relocalization/          # 重定位模块
│   │   ├── small_gicp/              # GICP 配准库
│   │   └── TEASER-plusplus/         # 全局配准库
│   ├── example_teaser_gicp.launch.py
│   └── readme.md
│
└── Livox-SDK2/                       # Livox SDK
```

---

## 8. 进阶：定制化开发

### 8.1 添加更多 AR 元素

在 `ar_overlay_node.py` 的 `image_callback` 方法中添加自定义绘制逻辑：

```python
# 示例：在第5格（中心）绘制特殊标记
if 5 in projected_pixels:
    cx, cy = projected_pixels[5]
    cv2.drawMarker(image, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 20, 3)
```

### 8.2 动态更新九宫格位置

创建新的 ROS 服务或订阅者，在运行时更新 `self.grid_points_w`。

### 8.3 多九宫格支持

扩展配置文件支持多组九宫格，在渲染时使用不同颜色区分。

---

## 9. 参考资料

- [FAST-LIVO2 论文](https://github.com/hku-mars/FAST-LIVO2)
- [TEASER++ 论文](https://github.com/MIT-SPARK/TEASER-plusplus)
- [small_gicp 库](https://github.com/koide3/small_gicp)
- [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2)
- [RealSense ROS 2](https://github.com/IntelRealSense/realsense-ros)
