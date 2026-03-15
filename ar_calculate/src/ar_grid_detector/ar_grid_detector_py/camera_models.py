"""
相机模型模块 - 支持针孔相机和多种鱼眼相机模型
Camera Models Module - Supports pinhole and various fisheye camera models

鱼眼相机畸变模型:
1. 等距投影 (Equidistant): r = f * theta
2. 等立体角投影 (Equisolid): r = 2 * f * sin(theta/2)
3. 正交投影 (Orthographic): r = f * sin(theta)
4. 立体投影 (Stereographic): r = 2 * f * tan(theta/2)
5. OpenCV 鱼眼模型 (Kannala-Brandt): theta_d = theta + k1*theta^3 + k2*theta^5 + k3*theta^7 + k4*theta^9
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

import numpy as np


class CameraModelType(Enum):
    """相机模型类型枚举"""
    PINHOLE = "pinhole"
    FISHEYE_EQUIDISTANT = "fisheye_equidistant"
    FISHEYE_EQUISOLID = "fisheye_equisolid"
    FISHEYE_ORTHOGRAPHIC = "fisheye_orthographic"
    FISHEYE_STEREOGRAPHIC = "fisheye_stereographic"
    FISHEYE_KANNALA_BRANDT = "fisheye_kannala_brandt"  # OpenCV fisheye model


@dataclass
class CameraIntrinsics:
    """相机内参数据类"""
    fx: float  # x方向焦距
    fy: float  # y方向焦距
    cx: float  # 主点x坐标
    cy: float  # 主点y坐标
    width: int  # 图像宽度
    height: int  # 图像高度
    # 畸变参数 (针孔相机使用k1-k3, p1-p2; 鱼眼相机使用k1-k4)
    k1: float = 0.0
    k2: float = 0.0
    k3: float = 0.0
    k4: float = 0.0
    p1: float = 0.0  # 切向畸变
    p2: float = 0.0  # 切向畸变


class CameraModel(ABC):
    """相机模型抽象基类"""
    
    def __init__(self, intrinsics: CameraIntrinsics):
        self.intrinsics = intrinsics
    
    @abstractmethod
    def project_point(self, point_3d: np.ndarray) -> Optional[Tuple[float, float]]:
        """
        将3D点投影到2D图像平面
        Args:
            point_3d: 相机坐标系下的3D点 [x, y, z]
        Returns:
            (u, v) 像素坐标，如果点在相机后方则返回None
        """
        pass
    
    @abstractmethod
    def unproject_point(self, pixel: Tuple[float, float], depth: float) -> np.ndarray:
        """
        将2D像素点反投影到3D空间
        Args:
            pixel: (u, v) 像素坐标
            depth: 深度值
        Returns:
            相机坐标系下的3D点 [x, y, z]
        """
        pass
    
    def is_in_image(self, u: float, v: float) -> bool:
        """检查像素坐标是否在图像范围内"""
        return 0 <= u < self.intrinsics.width and 0 <= v < self.intrinsics.height
    
    def get_camera_matrix(self) -> np.ndarray:
        """获取3x3相机内参矩阵"""
        return np.array([
            [self.intrinsics.fx, 0, self.intrinsics.cx],
            [0, self.intrinsics.fy, self.intrinsics.cy],
            [0, 0, 1]
        ], dtype=np.float64)


class PinholeCamera(CameraModel):
    """针孔相机模型（支持径向和切向畸变校正）"""
    
    def project_point(self, point_3d: np.ndarray) -> Optional[Tuple[float, float]]:
        x, y, z = point_3d[0], point_3d[1], point_3d[2]
        
        # 点在相机后方
        if z <= 1e-6:
            return None
        
        # 归一化坐标
        x_n = x / z
        y_n = y / z
        
        # 应用畸变校正（如果有畸变参数）
        if self._has_distortion():
            x_d, y_d = self._apply_distortion(x_n, y_n)
        else:
            x_d, y_d = x_n, y_n
        
        # 投影到像素平面
        u = self.intrinsics.fx * x_d + self.intrinsics.cx
        v = self.intrinsics.fy * y_d + self.intrinsics.cy
        
        return (u, v)
    
    def unproject_point(self, pixel: Tuple[float, float], depth: float) -> np.ndarray:
        u, v = pixel
        x_n = (u - self.intrinsics.cx) / self.intrinsics.fx
        y_n = (v - self.intrinsics.cy) / self.intrinsics.fy
        
        # 如果有畸变，需要去畸变（迭代求解）
        if self._has_distortion():
            x_n, y_n = self._remove_distortion(x_n, y_n)
        
        return np.array([x_n * depth, y_n * depth, depth], dtype=np.float64)
    
    def _has_distortion(self) -> bool:
        intr = self.intrinsics
        return abs(intr.k1) > 1e-10 or abs(intr.k2) > 1e-10 or \
               abs(intr.k3) > 1e-10 or abs(intr.p1) > 1e-10 or abs(intr.p2) > 1e-10
    
    def _apply_distortion(self, x: float, y: float) -> Tuple[float, float]:
        """应用径向和切向畸变。

        输入为归一化平面坐标 (x, y)，输出为畸变后的归一化坐标 (x_d, y_d)。
        采用常见的径向多项式(radial) + 切向项(tangential) 模型。
        """
        intr = self.intrinsics
        r2 = x * x + y * y
        r4 = r2 * r2
        r6 = r4 * r2
        
        # 径向畸变
        radial = 1 + intr.k1 * r2 + intr.k2 * r4 + intr.k3 * r6
        
        # 切向畸变
        x_d = x * radial + 2 * intr.p1 * x * y + intr.p2 * (r2 + 2 * x * x)
        y_d = y * radial + intr.p1 * (r2 + 2 * y * y) + 2 * intr.p2 * x * y
        
        return x_d, y_d
    
    def _remove_distortion(self, x_d: float, y_d: float, iterations: int = 10) -> Tuple[float, float]:
        """迭代去畸变（逆向求解）。

        使用简单的固定迭代方法：每次根据当前估计计算正向畸变，
        并调整估计以逼近给定的畸变坐标 (x_d, y_d)。
        该方法适用于畸变较小或良性收敛的情况。
        """
        x, y = x_d, y_d
        for _ in range(iterations):
            x_est, y_est = self._apply_distortion(x, y)
            x = x_d - (x_est - x)
            y = y_d - (y_est - y)
        return x, y


class FisheyeCamera(CameraModel):
    """鱼眼相机模型基类"""
    
    def __init__(self, intrinsics: CameraIntrinsics, model_type: CameraModelType):
        super().__init__(intrinsics)
        self.model_type = model_type
    
    @abstractmethod
    def _theta_to_r(self, theta: float) -> float:
        """角度到径向距离的映射函数"""
        pass
    
    @abstractmethod
    def _r_to_theta(self, r: float) -> float:
        """径向距离到角度的映射函数（反函数）"""
        pass
    
    def project_point(self, point_3d: np.ndarray) -> Optional[Tuple[float, float]]:
        x, y, z = point_3d[0], point_3d[1], point_3d[2]
        
        # 计算入射角
        r_3d = math.sqrt(x * x + y * y)
        theta = math.atan2(r_3d, z)
        
        # 超过180度视场角的点无法投影
        if theta > math.pi:
            return None
        
        # 使用投影模型计算径向距离
        r = self._theta_to_r(theta)
        
        # 计算归一化方向
        if r_3d < 1e-10:
            # 点在光轴上
            u = self.intrinsics.cx
            v = self.intrinsics.cy
        else:
            # 归一化xy方向
            x_n = x / r_3d
            y_n = y / r_3d
            
            # 投影到像素平面
            u = self.intrinsics.fx * r * x_n + self.intrinsics.cx
            v = self.intrinsics.fy * r * y_n + self.intrinsics.cy
        
        return (u, v)
    
    def unproject_point(self, pixel: Tuple[float, float], depth: float) -> np.ndarray:
        u, v = pixel
        
        # 归一化坐标
        x_n = (u - self.intrinsics.cx) / self.intrinsics.fx
        y_n = (v - self.intrinsics.cy) / self.intrinsics.fy
        
        r = math.sqrt(x_n * x_n + y_n * y_n)
        
        if r < 1e-10:
            return np.array([0, 0, depth], dtype=np.float64)
        
        theta = self._r_to_theta(r)
        
        # 重建3D方向
        sin_theta = math.sin(theta)
        cos_theta = math.cos(theta)
        
        # 在光线方向上找到指定深度的点
        scale = depth / cos_theta if abs(cos_theta) > 1e-6 else depth
        
        x = scale * sin_theta * (x_n / r)
        y = scale * sin_theta * (y_n / r)
        z = depth
        
        return np.array([x, y, z], dtype=np.float64)


class EquidistantFisheye(FisheyeCamera):
    """等距投影鱼眼相机 r = f * theta"""
    
    def __init__(self, intrinsics: CameraIntrinsics):
        super().__init__(intrinsics, CameraModelType.FISHEYE_EQUIDISTANT)
    
    def _theta_to_r(self, theta: float) -> float:
        return theta  # 归一化后的r，实际使用时乘以焦距
    
    def _r_to_theta(self, r: float) -> float:
        return r


class EquisolidFisheye(FisheyeCamera):
    """等立体角投影鱼眼相机 r = 2 * f * sin(theta/2)"""
    
    def __init__(self, intrinsics: CameraIntrinsics):
        super().__init__(intrinsics, CameraModelType.FISHEYE_EQUISOLID)
    
    def _theta_to_r(self, theta: float) -> float:
        return 2 * math.sin(theta / 2)
    
    def _r_to_theta(self, r: float) -> float:
        # r = 2 * sin(theta/2), so theta = 2 * arcsin(r/2)
        r_clamped = max(-2.0, min(2.0, r))
        return 2 * math.asin(r_clamped / 2)


class OrthographicFisheye(FisheyeCamera):
    """正交投影鱼眼相机 r = f * sin(theta)"""
    
    def __init__(self, intrinsics: CameraIntrinsics):
        super().__init__(intrinsics, CameraModelType.FISHEYE_ORTHOGRAPHIC)
    
    def _theta_to_r(self, theta: float) -> float:
        return math.sin(theta)
    
    def _r_to_theta(self, r: float) -> float:
        r_clamped = max(-1.0, min(1.0, r))
        return math.asin(r_clamped)


class StereographicFisheye(FisheyeCamera):
    """立体投影鱼眼相机 r = 2 * f * tan(theta/2)"""
    
    def __init__(self, intrinsics: CameraIntrinsics):
        super().__init__(intrinsics, CameraModelType.FISHEYE_STEREOGRAPHIC)
    
    def _theta_to_r(self, theta: float) -> float:
        return 2 * math.tan(theta / 2)
    
    def _r_to_theta(self, r: float) -> float:
        return 2 * math.atan(r / 2)


class KannalaBrandtFisheye(CameraModel):
    """
    OpenCV 鱼眼相机模型 (Kannala-Brandt)
    theta_d = theta + k1*theta^3 + k2*theta^5 + k3*theta^7 + k4*theta^9
    
    这是 OpenCV cv2.fisheye 函数使用的模型
    """
    
    def __init__(self, intrinsics: CameraIntrinsics):
        super().__init__(intrinsics)
        self.model_type = CameraModelType.FISHEYE_KANNALA_BRANDT
    
    def _distort_theta(self, theta: float) -> float:
        """将真实入射角 theta 映射为畸变角 theta_d（按多项式展开）。"""
        k1, k2, k3, k4 = (self.intrinsics.k1, self.intrinsics.k2, 
                          self.intrinsics.k3, self.intrinsics.k4)
        theta2 = theta * theta
        theta4 = theta2 * theta2
        theta6 = theta4 * theta2
        theta8 = theta6 * theta2
        
        theta_d = theta * (1 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8)
        return theta_d
    
    def _undistort_theta(self, theta_d: float, iterations: int = 10) -> float:
        """迭代求解未畸变角 theta，给定畸变角 theta_d。

        使用简化牛顿/固定步长迭代近似求解逆函数。
        """
        theta = theta_d
        for _ in range(iterations):
            theta_est = self._distort_theta(theta)
            # 简化的牛顿迭代
            theta = theta - (theta_est - theta_d) * 0.5
        return theta
    
    def project_point(self, point_3d: np.ndarray) -> Optional[Tuple[float, float]]:
        x, y, z = point_3d[0], point_3d[1], point_3d[2]
        
        r_3d = math.sqrt(x * x + y * y)
        theta = math.atan2(r_3d, z)
        
        # 超过视场角范围
        if theta > math.pi:
            return None
        
        # 应用畸变
        theta_d = self._distort_theta(theta)
        
        if r_3d < 1e-10:
            u = self.intrinsics.cx
            v = self.intrinsics.cy
        else:
            x_n = x / r_3d
            y_n = y / r_3d
            
            u = self.intrinsics.fx * theta_d * x_n + self.intrinsics.cx
            v = self.intrinsics.fy * theta_d * y_n + self.intrinsics.cy
        
        return (u, v)
    
    def unproject_point(self, pixel: Tuple[float, float], depth: float) -> np.ndarray:
        u, v = pixel
        
        x_n = (u - self.intrinsics.cx) / self.intrinsics.fx
        y_n = (v - self.intrinsics.cy) / self.intrinsics.fy
        
        theta_d = math.sqrt(x_n * x_n + y_n * y_n)
        
        if theta_d < 1e-10:
            return np.array([0, 0, depth], dtype=np.float64)
        
        theta = self._undistort_theta(theta_d)
        
        sin_theta = math.sin(theta)
        cos_theta = math.cos(theta)
        
        scale = depth / cos_theta if abs(cos_theta) > 1e-6 else depth
        
        x = scale * sin_theta * (x_n / theta_d)
        y = scale * sin_theta * (y_n / theta_d)
        z = depth
        
        return np.array([x, y, z], dtype=np.float64)


def create_camera_model(model_type: CameraModelType, intrinsics: CameraIntrinsics) -> CameraModel:
    """
    工厂函数：根据类型创建相机模型
    
    Args:
        model_type: 相机模型类型
        intrinsics: 相机内参
    
    Returns:
        对应的相机模型实例
    """
    if model_type == CameraModelType.PINHOLE:
        return PinholeCamera(intrinsics)
    elif model_type == CameraModelType.FISHEYE_EQUIDISTANT:
        return EquidistantFisheye(intrinsics)
    elif model_type == CameraModelType.FISHEYE_EQUISOLID:
        return EquisolidFisheye(intrinsics)
    elif model_type == CameraModelType.FISHEYE_ORTHOGRAPHIC:
        return OrthographicFisheye(intrinsics)
    elif model_type == CameraModelType.FISHEYE_STEREOGRAPHIC:
        return StereographicFisheye(intrinsics)
    elif model_type == CameraModelType.FISHEYE_KANNALA_BRANDT:
        return KannalaBrandtFisheye(intrinsics)
    else:
        raise ValueError(f"Unknown camera model type: {model_type}")


def create_camera_from_params(
    model_type_str: str,
    fx: float, fy: float, cx: float, cy: float,
    width: int, height: int,
    k1: float = 0.0, k2: float = 0.0, k3: float = 0.0, k4: float = 0.0,
    p1: float = 0.0, p2: float = 0.0
) -> CameraModel:
    """
    便捷函数：从参数直接创建相机模型
    
    Args:
        model_type_str: 模型类型字符串 ("pinhole", "fisheye_equidistant", etc.)
        fx, fy: 焦距
        cx, cy: 主点
        width, height: 图像尺寸
        k1, k2, k3, k4: 畸变参数
        p1, p2: 切向畸变参数 (仅针孔模型使用)
    
    Returns:
        相机模型实例
    """
    model_type = CameraModelType(model_type_str.lower())
    intrinsics = CameraIntrinsics(
        fx=fx, fy=fy, cx=cx, cy=cy,
        width=width, height=height,
        k1=k1, k2=k2, k3=k3, k4=k4,
        p1=p1, p2=p2
    )
    return create_camera_model(model_type, intrinsics)
