"""
工具函数模块
Utility Functions Module
"""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np


def quat_to_rot_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """
    将四元数 (qx, qy, qz, qw) 转换为 3x3 旋转矩阵。

    说明：先对四元数归一化以保证数值稳定性，然后按常见四元数到旋转矩阵的公式构造矩阵。

    输入/输出：
    - 输入为四元数组件（任意顺序归一化会被处理）。
    - 返回 3x3 的 `np.ndarray` 旋转矩阵。

    在本项目中的用途：
    - 在 `ar_grid_node.py` 的 `odom_callback` 中使用四元数构造里程计旋转矩阵（见 ar_grid_node.py L430），
      进一步用于构建传感器位姿 `T_w_s`，用于世界坐标到相机坐标的变换链路。
    """
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        raise ValueError("Quaternion norm is too small")

    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
    r01 = 2.0 * (qx * qy - qz * qw)
    r02 = 2.0 * (qx * qz + qy * qw)

    r10 = 2.0 * (qx * qy + qz * qw)
    r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
    r12 = 2.0 * (qy * qz - qx * qw)

    r20 = 2.0 * (qx * qz - qy * qw)
    r21 = 2.0 * (qy * qz + qx * qw)
    r22 = 1.0 - 2.0 * (qx * qx + qy * qy)

    return np.array([
        [r00, r01, r02],
        [r10, r11, r12],
        [r20, r21, r22],
    ], dtype=np.float64)


def rot_matrix_to_quat(R: np.ndarray) -> Tuple[float, float, float, float]:
    """
    将 3x3 旋转矩阵转换回四元数 (qx, qy, qz, qw)。

    说明：实现采用常见数值稳定的分支法（根据矩阵主对角线元素的最大值分支），
    保证在不同旋转情况下仍较为稳定。

    在本项目中的用途：
    - 主要作为通用工具导出，可能用于需要四元数表示（例如保存/发布外参）场景。
    """
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    
    return (qx, qy, qz, qw)


def make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """
    创建 4x4 齐次变换矩阵（SE(3)）:

        [ R  t ]
        [ 0  1 ]

    Args:
        rotation: 3x3 旋转矩阵
        translation: 长度为3的平移向量

    Returns:
        4x4 变换矩阵（np.ndarray）

    用途：在 `ar_grid_node.py` 中用于根据 `extrinsic_rpy_xyz` 构建相机外参 `T_c_s`（见 L265）。
    """
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation.flatten()
    return transform


def rot_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    根据 Roll-Pitch-Yaw (绕 X, Y, Z) 构造旋转矩阵。

    实现细节：按 ZYX 顺序（先 roll 绕 X，随后 pitch 绕 Y，最后 yaw 绕 Z）构造，
    最终返回 `R = Rz * Ry * Rx`。

    在本项目中的用途：
    - 用于从参数 `extrinsic_rpy_xyz` 构建外参旋转矩阵（`ar_grid_node._load_extrinsic_t_c_s`），
      进而构成 4x4 外参变换矩阵。
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([
        [1.0, 0.0, 0.0],
        [0.0, cr, -sr],
        [0.0, sr, cr],
    ], dtype=np.float64)
    
    ry = np.array([
        [cp, 0.0, sp],
        [0.0, 1.0, 0.0],
        [-sp, 0.0, cp],
    ], dtype=np.float64)
    
    rz = np.array([
        [cy, -sy, 0.0],
        [sy, cy, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    
    return rz @ ry @ rx


def rpy_from_rot(R: np.ndarray) -> Tuple[float, float, float]:
    """
    从 3x3 旋转矩阵提取 Roll-Pitch-Yaw（弧度）。

    说明：对奇异情形（pitch 接近 +/-90°）做了判定并采取备用计算分支以避免数值不稳定。

    在本项目中的用途：
    - 导出为通用工具，便于在需要把旋转矩阵转回 Euler 角（例如调试/可视化）时使用。
    """
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    
    singular = sy < 1e-6
    
    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0
    
    return (roll, pitch, yaw)


def transform_point(transform: np.ndarray, point: np.ndarray) -> np.ndarray:
    """
    使用 4x4 变换矩阵对 3D 点进行仿射变换。

    输入为 3 元向量，函数内部构造齐次坐标并乘以变换矩阵，最后返回去齐次后的 3 元向量。

    在本项目中的用途：
    - 通用变换函数；在 `ar_grid_node.py` 内部直接使用 `T @ p_h` 的写法较多，
      但该函数可用于单点变换的便捷调用和外部模块复用。
    """
    point_h = np.array([point[0], point[1], point[2], 1.0], dtype=np.float64)
    result_h = transform @ point_h
    return result_h[:3]


def inverse_transform(transform: np.ndarray) -> np.ndarray:
    """
    计算 4x4 变换矩阵的逆（利用旋转矩阵的转置作为逆），

    若 T = [R t; 0 1]，则 T_inv = [R^T -R^T t; 0 1]。

    在本项目中的用途（关键）：
    - 在 `ar_grid_node.image_callback` 中，将里程计位姿 `T_w_s` 取逆得到 `T_s_w`（见 ar_grid_node.py L466），
      进而与相机外参 `T_c_s` 相乘得到 `T_c_w`（相机到世界的变换链）。这是 AR 投影的核心变换链路：
          P_c = T_c_w * P_w = T_c_s * (T_w_s)^{-1} * P_w
    """
    R = transform[:3, :3]
    t = transform[:3, 3]
    
    R_inv = R.T
    t_inv = -R_inv @ t
    
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = R_inv
    result[:3, 3] = t_inv
    
    return result


def point_in_polygon(point: Tuple[float, float], polygon: list) -> bool:
    """
    判断二维点是否在多边形内，采用射线法（ray casting）。

    输入：`point` 为 `(x,y)`，`polygon` 为顶点列表（顺时针或逆时针均可）。

    在本项目中的用途：
    - 可用于判断投影后的像素点（例如格子中心或角点）是否位于图像内某个多边形区域，
      或用于更复杂的可见性/遮挡检测（目前导出为工具，`ar_grid_node.py` 使用像素边界检测逻辑时可复用）。
    """
    x, y = point
    n = len(polygon)
    inside = False
    
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    
    return inside


def compute_polygon_area(polygon: list) -> float:
    """
    使用鞋带公式（Gauss area formula）计算二维多边形的有向面积，返回非负面积值。

    在本项目中的用途：
    - 可用于评估投影后的格子在图像上的占据面积（例如检测是否足够大以判定可见），
      或用于统计/诊断信息（当前实现作为通用工具导出）。
    """
    n = len(polygon)
    if n < 3:
        return 0.0
    
    area = 0.0
    j = n - 1
    for i in range(n):
        area += (polygon[j][0] + polygon[i][0]) * (polygon[j][1] - polygon[i][1])
        j = i
    
    return abs(area) / 2.0


def clip_polygon_to_rect(polygon: list, rect: Tuple[float, float, float, float]) -> list:
    """
    将任意多边形裁剪到给定矩形内，采用 Sutherland-Hodgman 多边形裁剪算法。

    Args:
        polygon: 顶点列表 [(x,y), ...]
        rect: 矩形边界 `(x_min, y_min, x_max, y_max)`

    返回值：裁剪后的顶点列表（可能为空）。

    在本项目中的用途：
    - 用于将投影到像素平面的格子轮廓裁剪到图像边界，便于计算可见区域、面积或用于绘制裁剪后的多边形。
    """
    x_min, y_min, x_max, y_max = rect
    
    def clip_edge(polygon, x1, y1, x2, y2):
        """对单条边进行裁剪"""
        if not polygon:
            return []
        
        result = []
        for i in range(len(polygon)):
            curr = polygon[i]
            prev = polygon[i - 1]
            
            # 计算点相对于边的位置
            def inside(p):
                return (x2 - x1) * (p[1] - y1) - (y2 - y1) * (p[0] - x1) >= 0
            
            def intersection(p1, p2):
                dc = (p1[0] - p2[0], p1[1] - p2[1])
                dp = (x1 - x2, y1 - y2)
                n1 = (x2 - x1) * (p1[1] - y1) - (y2 - y1) * (p1[0] - x1)
                n2 = dc[0] * dp[1] - dc[1] * dp[0]
                if abs(n2) < 1e-10:
                    return p1
                t = n1 / n2
                return (p1[0] + t * dc[0], p1[1] + t * dc[1])
            
            curr_inside = inside(curr)
            prev_inside = inside(prev)
            
            if curr_inside:
                if not prev_inside:
                    result.append(intersection(prev, curr))
                result.append(curr)
            elif prev_inside:
                result.append(intersection(prev, curr))
        
        return result
    
    # 依次对四条边进行裁剪
    result = polygon
    result = clip_edge(result, x_min, y_min, x_max, y_min)  # 下边
    result = clip_edge(result, x_max, y_min, x_max, y_max)  # 右边
    result = clip_edge(result, x_max, y_max, x_min, y_max)  # 上边
    result = clip_edge(result, x_min, y_max, x_min, y_min)  # 左边
    
    return result
