#!/usr/bin/env python3
from __future__ import annotations

from collections import deque
import json
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Point32
from nav_msgs.msg import Odometry
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from ar_grid_detector_py.camera_models import CameraModelType, create_camera_from_params
from ar_grid_detector_py.grid_generator import (
    GridFrame,
    GridFrameGenerator,
    convert_legacy_nine_grid_points,
)
from ar_grid_detector.msg import GridCell, GridCellArray
from ar_grid_detector_py.utils import (
    inverse_transform,
    make_transform,
    quat_to_rot_matrix,
    rot_from_rpy,
)


class ArGridNode(Node):
    def __init__(self) -> None:
        super().__init__("ar_grid_node")

        self._declare_params()
        self.bridge = CvBridge()
        self._lock = threading.Lock()

        self._latest_t_w_s = np.eye(4, dtype=np.float64)
        self._latest_odom_time = None
        self._odom_received_once = False
        self._window_error_logged = False
        self._last_diag_log_time = 0.0
        self._last_pose_age_warn_time = 0.0
        self._odom_history = deque(maxlen=int(self.get_parameter("odom_sync.max_history").value))

        self.t_c_s = self._load_extrinsic_t_c_s()
        self.camera_model = self._load_camera_model()
        self.grid_groups = self._load_or_generate_grid_groups()
        self.total_cells = int(sum(frame.total_cells for _, frame in self.grid_groups))

        odom_topic = str(self.get_parameter("odom_topic").value)
        image_topic = str(self.get_parameter("image_topic").value)
        output_image_topic = str(self.get_parameter("output_image_topic").value)
        visible_cells_topic = str(self.get_parameter("visible_cells_topic").value)

        self.create_subscription(Odometry, odom_topic, self.odom_callback, 1)
        self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.image_pub = self.create_publisher(Image, output_image_topic, 1)
        self.visible_cells_pub = self.create_publisher(GridCellArray, visible_cells_topic, 5)

        self.get_logger().info(f"Camera model: {self.get_parameter('camera_model').value}")
        group_summary = ", ".join(
            f"G{group_id}:{frame.rows}x{frame.cols}({frame.total_cells})"
            for group_id, frame in self.grid_groups
        )
        self.get_logger().info(
            f"Grid groups={len(self.grid_groups)}, total_cells={self.total_cells}, details=[{group_summary}]"
        )
        self.get_logger().info(f"Subscribed odom={odom_topic}, image={image_topic}")
        self.get_logger().info(f"Publishing overlay image to {output_image_topic}")
        self.get_logger().info(f"Publishing visible cells to {visible_cells_topic}")

    def _declare_params(self) -> None:
        self.declare_parameter("odom_topic", "/aft_mapped_in_map")
        self.declare_parameter("image_topic", "/fisheye_camera/image_raw")
        self.declare_parameter("output_image_topic", "/ar_grid/image")
        self.declare_parameter("visible_cells_topic", "/ar_grid/visible_cells")

        self.declare_parameter("camera_model", "pinhole")
        self.declare_parameter("camera_json", "")
        self.declare_parameter("camera.fx", 606.5)
        self.declare_parameter("camera.fy", 605.77)
        self.declare_parameter("camera.cx", 325.77)
        self.declare_parameter("camera.cy", 256.54)
        self.declare_parameter("camera.width", 640)
        self.declare_parameter("camera.height", 480)
        self.declare_parameter("camera.k1", 0.0)
        self.declare_parameter("camera.k2", 0.0)
        self.declare_parameter("camera.k3", 0.0)
        self.declare_parameter("camera.k4", 0.0)
        self.declare_parameter("camera.p1", 0.0)
        self.declare_parameter("camera.p2", 0.0)

        self.declare_parameter("grid.use_yaml", False)
        self.declare_parameter("grid.yaml_path", "")
        self.declare_parameter("grid.use_legacy_nine_points", False)
        self.declare_parameter("grid.legacy_nine_points_yaml", "/home/r1/Slam/ar_calculate/nine_grid_points.yaml")
        self.declare_parameter("grid.default_group_id", 0)
        self.declare_parameter("grid.groups_yaml", "")
        self.declare_parameter("grid.rows", 3)
        self.declare_parameter("grid.cols", 3)
        self.declare_parameter("grid.cell_width", 0.3)
        self.declare_parameter("grid.cell_height", 0.3)
        self.declare_parameter("grid.centers_size_source", "inferred")
        self.declare_parameter("grid.strict_size_check", False)
        self.declare_parameter("grid.size_tolerance", 1e-4)
        self.declare_parameter("grid.corner_top_left", [0.0, 0.0, 0.0])
        self.declare_parameter("grid.corner_top_right", [0.9, 0.0, 0.0])
        self.declare_parameter("grid.corner_bottom_left", [0.0, 0.9, 0.0])
        self.declare_parameter("grid.input_is_cell_centers", False)
        self.declare_parameter("grid.single_row_down_axis", [0.0, 0.0, -1.0])

        self.declare_parameter("draw.cell_border_color_bgr", [255, 0, 0])
        self.declare_parameter("draw.cell_border_thickness", 2)
        self.declare_parameter("draw.cell_center_color_bgr", [0, 0, 255])
        self.declare_parameter("draw.cell_center_radius", 4)
        self.declare_parameter("draw.cell_center_thickness", -1)
        self.declare_parameter("draw.label_color_bgr", [0, 255, 255])
        self.declare_parameter("draw.show_labels", True)
        self.declare_parameter("draw.show_status_text", True)
        self.declare_parameter("draw.show_window", True)
        self.declare_parameter("draw.window_name", "AR Grid Overlay")
        self.declare_parameter("draw.curve_samples_per_edge", 16)

        self.declare_parameter("extrinsic_rpy_xyz", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("extrinsic_matrix_4x4", Parameter.Type.DOUBLE_ARRAY)

        self.declare_parameter("odom_sync.use_closest_by_stamp", True)
        self.declare_parameter("odom_sync.max_history", 400)
        self.declare_parameter("odom_sync.max_pose_age_sec", 0.25)

    def _load_camera_model(self):
        camera_json = str(self.get_parameter("camera_json").value).strip()
        if camera_json:
            self._load_camera_from_json(Path(camera_json))

        model_str = str(self.get_parameter("camera_model").value).strip().lower()
        return create_camera_from_params(
            model_str,
            fx=float(self.get_parameter("camera.fx").value),
            fy=float(self.get_parameter("camera.fy").value),
            cx=float(self.get_parameter("camera.cx").value),
            cy=float(self.get_parameter("camera.cy").value),
            width=int(self.get_parameter("camera.width").value),
            height=int(self.get_parameter("camera.height").value),
            k1=float(self.get_parameter("camera.k1").value),
            k2=float(self.get_parameter("camera.k2").value),
            k3=float(self.get_parameter("camera.k3").value),
            k4=float(self.get_parameter("camera.k4").value),
            p1=float(self.get_parameter("camera.p1").value),
            p2=float(self.get_parameter("camera.p2").value),
        )

    def _is_fisheye_model_selected(self) -> bool:
        model_str = str(self.get_parameter("camera_model").value).strip().lower()
        return model_str.startswith("fisheye_")

    def _draw_sampled_world_edge(
        self,
        image: np.ndarray,
        start_w: np.ndarray,
        end_w: np.ndarray,
        t_c_w: np.ndarray,
        color: Tuple[int, int, int],
        thickness: int,
        samples_per_edge: int,
    ) -> None:
        samples = max(2, int(samples_per_edge))
        points: List[Tuple[int, int]] = []

        for i in range(samples + 1):
            alpha = float(i) / float(samples)
            p_w = (1.0 - alpha) * start_w + alpha * end_w
            p_w_h = np.array([p_w[0], p_w[1], p_w[2], 1.0], dtype=np.float64)
            p_c_h = t_c_w @ p_w_h
            p_c = p_c_h[:3]

            if p_c[2] <= 1e-6:
                continue

            uv = self.camera_model.project_point(p_c)
            if uv is None:
                continue

            pt = (int(round(uv[0])), int(round(uv[1])))
            if not points or pt != points[-1]:
                points.append(pt)

        for idx in range(len(points) - 1):
            cv2.line(
                image,
                points[idx],
                points[idx + 1],
                color,
                thickness,
                lineType=cv2.LINE_AA,
            )

    def _compute_projected_visible_area_ratio(
        self,
        corners_px: List[Tuple[float, float]],
        image_width: int,
        image_height: int,
    ) -> float:
        if len(corners_px) != 4 or image_width <= 1 or image_height <= 1:
            return 0.0

        quad = np.asarray(corners_px, dtype=np.float32).reshape(-1, 1, 2)
        quad_area = float(abs(cv2.contourArea(quad)))
        if quad_area <= 1e-6:
            return 0.0

        image_rect = np.array(
            [[0.0, 0.0], [float(image_width - 1), 0.0], [float(image_width - 1), float(image_height - 1)], [0.0, float(image_height - 1)]],
            dtype=np.float32,
        ).reshape(-1, 1, 2)

        try:
            inter_area, _ = cv2.intersectConvexConvex(quad, image_rect)
        except cv2.error:
            return 0.0

        if inter_area <= 0.0:
            return 0.0

        ratio = float(inter_area) / quad_area
        return max(0.0, min(1.0, ratio))

    def _load_camera_from_json(self, json_path: Path) -> None:
        if not json_path.exists():
            raise FileNotFoundError(f"camera_json not found: {json_path}")

        data = json.loads(json_path.read_text(encoding="utf-8"))
        camera_matrix = data.get("camera_matrix", {})
        distortion = data.get("distortion_coefficients", {})

        if "fx" in camera_matrix:
            fx = camera_matrix["fx"]
            fy = camera_matrix["fy"]
            cx = camera_matrix["cx"]
            cy = camera_matrix["cy"]
        elif "data" in camera_matrix:
            m = np.asarray(camera_matrix["data"], dtype=np.float64).reshape(3, 3)
            fx, fy, cx, cy = float(m[0, 0]), float(m[1, 1]), float(m[0, 2]), float(m[1, 2])
        else:
            raise ValueError("camera_json missing camera_matrix")

        self.set_parameters([
            Parameter("camera.fx", value=float(fx)),
            Parameter("camera.fy", value=float(fy)),
            Parameter("camera.cx", value=float(cx)),
            Parameter("camera.cy", value=float(cy)),
        ])

        image_width = data.get("image_width")
        image_height = data.get("image_height")
        if image_width is not None and image_height is not None:
            self.set_parameters([
                Parameter("camera.width", value=int(image_width)),
                Parameter("camera.height", value=int(image_height)),
            ])

        dist_data = distortion.get("data", [])
        if len(dist_data) >= 4:
            self.set_parameters([
                Parameter("camera.k1", value=float(dist_data[0])),
                Parameter("camera.k2", value=float(dist_data[1])),
                Parameter("camera.k3", value=float(dist_data[2])),
                Parameter("camera.k4", value=float(dist_data[3])),
            ])
            if len(dist_data) >= 6:
                self.set_parameters([
                    Parameter("camera.p1", value=float(dist_data[4])),
                    Parameter("camera.p2", value=float(dist_data[5])),
                ])

    def _load_extrinsic_t_c_s(self) -> np.ndarray:
        # 外参语义：T_c_s 表示“传感器系 -> 相机系”的刚体变换。
        # 后续投影统一使用：P_c = T_c_s * T_s_w * P_w。
        # 若这里的坐标轴约定和里程计/相机约定不一致，会造成整体偏移、旋转或镜像。
        try:
            matrix_raw = list(self.get_parameter("extrinsic_matrix_4x4").value)
        except ParameterUninitializedException:
            matrix_raw = []

        if len(matrix_raw) == 16:
            return np.asarray(matrix_raw, dtype=np.float64).reshape(4, 4)
        if len(matrix_raw) not in (0,):
            raise ValueError("extrinsic_matrix_4x4 must have 16 numbers or be empty")

        rpy_xyz_raw = list(self.get_parameter("extrinsic_rpy_xyz").value)
        if len(rpy_xyz_raw) != 6:
            raise ValueError("extrinsic_rpy_xyz must have 6 numbers: [roll, pitch, yaw, x, y, z]")

        roll, pitch, yaw, x, y, z = [float(v) for v in rpy_xyz_raw]
        rot = rot_from_rpy(roll, pitch, yaw)
        trans = np.array([x, y, z], dtype=np.float64)
        return make_transform(rot, trans)

    def _load_or_generate_grid_groups(self) -> List[Tuple[int, GridFrame]]:
        groups_yaml_raw = str(self.get_parameter("grid.groups_yaml").value).strip()
        if groups_yaml_raw:
            return self._load_grid_groups_from_inline_yaml(groups_yaml_raw)

        default_group_id = int(self.get_parameter("grid.default_group_id").value)
        single_frame = self._load_or_generate_grid_frame()
        return [(default_group_id, single_frame)]

    def _load_grid_groups_from_inline_yaml(self, groups_yaml_raw: str) -> List[Tuple[int, GridFrame]]:
        parsed = yaml.safe_load(groups_yaml_raw)
        if not isinstance(parsed, list) or len(parsed) == 0:
            raise ValueError("grid.groups_yaml must be a non-empty YAML list")

        groups: List[Tuple[int, GridFrame]] = []
        seen_group_ids: Set[int] = set()
        for idx, item in enumerate(parsed):
            if not isinstance(item, dict):
                raise ValueError(f"grid.groups_yaml[{idx}] must be a mapping")

            group_id = int(item.get("group_id", idx))
            if group_id in seen_group_ids:
                raise ValueError(f"duplicate group_id in grid.groups_yaml: {group_id}")
            seen_group_ids.add(group_id)

            rows = int(item["rows"])
            cols = int(item["cols"])
            if rows <= 0 or cols <= 0:
                raise ValueError(f"group_id={group_id}: rows/cols must be > 0")

            cell_width_cfg = item.get("cell_width", None)
            cell_height_cfg = item.get("cell_height", None)
            strict_size_check = bool(item.get("strict_size_check", False))
            size_tolerance = float(item.get("size_tolerance", 1e-4))
            input_is_cell_centers = bool(item.get("input_is_cell_centers", False))
            size_source = str(item.get("centers_size_source", self.get_parameter("grid.centers_size_source").value)).strip().lower()

            cell_width = float(cell_width_cfg) if cell_width_cfg is not None else None
            cell_height = float(cell_height_cfg) if cell_height_cfg is not None else None
            if cell_width is not None and cell_width <= 0.0:
                cell_width = None
            if cell_height is not None and cell_height <= 0.0:
                cell_height = None

            corner_top_left = np.asarray(item["corner_top_left"], dtype=np.float64)
            corner_top_right = np.asarray(item["corner_top_right"], dtype=np.float64)
            raw_bottom_left = item.get("corner_bottom_left", None)
            corner_bottom_left = np.asarray(raw_bottom_left, dtype=np.float64) if raw_bottom_left is not None else np.zeros(3, dtype=np.float64)

            if input_is_cell_centers:
                single_row_down_axis = np.asarray(
                    item.get("single_row_down_axis", [0.0, 0.0, -1.0]),
                    dtype=np.float64,
                )
                frame = GridFrameGenerator.generate_from_center_points(
                    center_top_left=corner_top_left,
                    center_top_right=corner_top_right,
                    center_bottom_left=corner_bottom_left if raw_bottom_left is not None else None,
                    rows=rows,
                    cols=cols,
                    cell_width=cell_width,
                    cell_height=cell_height,
                    size_source=size_source,
                    strict_size_check=strict_size_check,
                    size_tolerance=size_tolerance,
                    single_row_down_axis=single_row_down_axis,
                )
            else:
                if corner_top_left.size != 3 or corner_top_right.size != 3 or corner_bottom_left.size != 3:
                    raise ValueError(f"group_id={group_id}: corners must be [x,y,z]")
                frame = GridFrameGenerator.generate_from_three_corners(
                    corner_top_left=corner_top_left,
                    corner_top_right=corner_top_right,
                    corner_bottom_left=corner_bottom_left,
                    rows=rows,
                    cols=cols,
                    cell_width=cell_width,
                    cell_height=cell_height,
                    strict_size_check=strict_size_check,
                    size_tolerance=size_tolerance,
                )
            groups.append((group_id, frame))

        return groups

    def _load_or_generate_grid_frame(self) -> GridFrame:
        use_legacy_nine_points = bool(self.get_parameter("grid.use_legacy_nine_points").value)
        if use_legacy_nine_points:
            legacy_yaml_path = Path(str(self.get_parameter("grid.legacy_nine_points_yaml").value))
            return self._load_grid_from_legacy_yaml(legacy_yaml_path)

        use_yaml = bool(self.get_parameter("grid.use_yaml").value)
        if use_yaml:
            yaml_path_str = str(self.get_parameter("grid.yaml_path").value).strip()
            if yaml_path_str:
                yaml_path = Path(yaml_path_str)
                if yaml_path.is_file():
                    return self._load_grid_from_yaml(yaml_path)
                self.get_logger().warn(
                    f"grid.use_yaml=true but grid.yaml_path is not a file: '{yaml_path}'. "
                    "Falling back to three-corner grid generation."
                )
            else:
                self.get_logger().warn(
                    "grid.use_yaml=true but grid.yaml_path is empty. "
                    "Falling back to three-corner grid generation."
                )

        rows = int(self.get_parameter("grid.rows").value)
        cols = int(self.get_parameter("grid.cols").value)
        cell_width = float(self.get_parameter("grid.cell_width").value)
        cell_height = float(self.get_parameter("grid.cell_height").value)
        centers_size_source = str(self.get_parameter("grid.centers_size_source").value).strip().lower()
        strict_size_check = bool(self.get_parameter("grid.strict_size_check").value)
        size_tolerance = float(self.get_parameter("grid.size_tolerance").value)
        input_is_cell_centers = bool(self.get_parameter("grid.input_is_cell_centers").value)
        corner_top_left = np.asarray(self.get_parameter("grid.corner_top_left").value, dtype=np.float64)
        corner_top_right = np.asarray(self.get_parameter("grid.corner_top_right").value, dtype=np.float64)
        corner_bottom_left = np.asarray(self.get_parameter("grid.corner_bottom_left").value, dtype=np.float64)

        if corner_top_left.size != 3 or corner_top_right.size != 3 or corner_bottom_left.size != 3:
            raise ValueError("grid corners must be [x,y,z]")

        # 参数语义说明：
        # - input_is_cell_centers=False：corner_* 直接表示“外框角点”
        # - input_is_cell_centers=True：corner_* 实际表示“左上/右上/左下格子的中心点”
        #   直接使用中心点进行网格构建，不再先转外角点再反推尺寸。
        if input_is_cell_centers:
            single_row_down_axis = np.asarray(self.get_parameter("grid.single_row_down_axis").value, dtype=np.float64)
            self.get_logger().info(
                f"Generating grid from center points directly (rows={rows}, cols={cols}, size_source={centers_size_source})"
            )
            center_bottom_left_input = None if rows == 1 else corner_bottom_left
            return GridFrameGenerator.generate_from_center_points(
                center_top_left=corner_top_left,
                center_top_right=corner_top_right,
                center_bottom_left=center_bottom_left_input,
                rows=rows,
                cols=cols,
                cell_width=cell_width,
                cell_height=cell_height,
                size_source=centers_size_source,
                strict_size_check=strict_size_check,
                size_tolerance=size_tolerance,
                single_row_down_axis=single_row_down_axis,
            )

        return GridFrameGenerator.generate_from_three_corners(
            corner_top_left=corner_top_left,
            corner_top_right=corner_top_right,
            corner_bottom_left=corner_bottom_left,
            rows=rows,
            cols=cols,
            cell_width=cell_width,
            cell_height=cell_height,
            strict_size_check=strict_size_check,
            size_tolerance=size_tolerance,
        )

    def _load_grid_from_legacy_yaml(self, yaml_path: Path) -> GridFrame:
        if not yaml_path.exists():
            raise FileNotFoundError(f"legacy nine grid yaml not found: {yaml_path}")

        data = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        if "grid_points" not in data:
            raise ValueError(f"legacy yaml missing 'grid_points': {yaml_path}")

        raw_grid_points = data["grid_points"]
        nine_points: Dict[int, np.ndarray] = {}
        for idx in range(1, 10):
            key_i = idx
            key_s = str(idx)
            if key_i in raw_grid_points:
                raw = raw_grid_points[key_i]
            elif key_s in raw_grid_points:
                raw = raw_grid_points[key_s]
            else:
                raise ValueError(f"legacy grid_points missing index {idx}")
            arr = np.asarray(raw, dtype=np.float64).reshape(-1)
            if arr.size != 3:
                raise ValueError(f"legacy grid_points[{idx}] must contain 3 values")
            nine_points[idx] = arr

        corner_top_left, corner_top_right, corner_bottom_left = convert_legacy_nine_grid_points(nine_points)
        return GridFrameGenerator.generate_from_three_corners(
            corner_top_left=corner_top_left,
            corner_top_right=corner_top_right,
            corner_bottom_left=corner_bottom_left,
            rows=3,
            cols=3,
            cell_width=None,
            cell_height=None,
            strict_size_check=False,
            size_tolerance=1e-4,
        )

    def _stamp_to_sec(self, stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    # def _select_pose_for_image(self, image_stamp_sec: float) -> Tuple[np.ndarray, Optional[float]]:
    #     with self._lock:
    #         latest_pose = self._latest_t_w_s.copy()
    #         history = list(self._odom_history)

    #     if not history:
    #         return latest_pose, None

    #     use_closest = bool(self.get_parameter("odom_sync.use_closest_by_stamp").value)
    #     if not use_closest:
    #         odom_stamp_sec, pose = history[-1]
    #         return pose.copy(), abs(image_stamp_sec - odom_stamp_sec)

    #     odom_stamp_sec, pose = min(history, key=lambda item: abs(item[0] - image_stamp_sec))
    #     return pose.copy(), abs(image_stamp_sec - odom_stamp_sec)
    def _select_pose_for_image(self, image_stamp_sec: float) -> Tuple[np.ndarray, Optional[float]]:
        # 获取锁并复制当前最新 pose 与历史队列的快照，避免在无锁状态下访问共享数据结构
        with self._lock:
            latest_pose = self._latest_t_w_s.copy()    # 最近一次收到的 T_w_s（4x4 矩阵）
            history = list(self._odom_history)          # odom 历史：deque[(stamp_sec, T_w_s), ...]

        # 如果历史为空，返回最新 pose（初始值为单位矩阵）和 None 表示没有 age 信息
        if not history:
            return latest_pose, None

        # 参数控制：使用最近（closest by stamp）策略还是直接使用最新（last）策略
        use_closest = bool(self.get_parameter("odom_sync.use_closest_by_stamp").value)

        # 如果不使用“closest by stamp”，直接取历史中最后一条（最新的里程计）
        # 返回值：
        #  - pose.copy(): 选择的 pose（4x4 矩阵副本，调用方可安全修改）
        #  - abs(image_stamp_sec - odom_stamp_sec): 图像时间与该 pose 时间的绝对差（秒）
        if not use_closest:
            odom_stamp_sec, pose = history[-1]
            return pose.copy(), abs(image_stamp_sec - odom_stamp_sec)

        # 使用“closest by stamp”策略：在历史中找到与 image_stamp_sec 差值最小的一条
        # min(..., key=...) 返回 (stamp_sec, pose) 对；这里按时间差的绝对值比较
        odom_stamp_sec, pose = min(history, key=lambda item: abs(item[0] - image_stamp_sec))
        # 返回所选 pose 的副本以及对应的时间差（秒）
        return pose.copy(), abs(image_stamp_sec - odom_stamp_sec)

    def _load_grid_from_yaml(self, yaml_path: Path) -> GridFrame:
        if not yaml_path.exists():
            raise FileNotFoundError(f"grid yaml not found: {yaml_path}")

        data = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        if "grid" not in data:
            raise ValueError("grid yaml missing root key 'grid'")

        cfg = data["grid"]
        rows = int(cfg["rows"])
        cols = int(cfg["cols"])
        cell_width = float(cfg.get("cell_width", 0.0))
        cell_height = float(cfg.get("cell_height", 0.0))
        strict_size_check = bool(cfg.get("strict_size_check", False))
        size_tolerance = float(cfg.get("size_tolerance", 1e-4))
        corner_top_left = np.asarray(cfg["corner_top_left"], dtype=np.float64)
        corner_top_right = np.asarray(cfg["corner_top_right"], dtype=np.float64)
        corner_bottom_left = np.asarray(cfg["corner_bottom_left"], dtype=np.float64)

        return GridFrameGenerator.generate_from_three_corners(
            corner_top_left=corner_top_left,
            corner_top_right=corner_top_right,
            corner_bottom_left=corner_bottom_left,
            rows=rows,
            cols=cols,
            cell_width=cell_width if cell_width > 0 else None,
            cell_height=cell_height if cell_height > 0 else None,
            strict_size_check=strict_size_check,
            size_tolerance=size_tolerance,
        )

    # def odom_callback(self, msg: Odometry) -> None:
    #     p = msg.pose.pose.position
    #     q = msg.pose.pose.orientation

    #     # 从里程计消息构造 T_w_s（sensor 在 world 中的位姿）：
    #     # P_w = T_w_s * P_s
    #     # 后续会求逆得到 T_s_w，用于把世界点变换到传感器系。
    #     rot_w_s = quat_to_rot_matrix(q.x, q.y, q.z, q.w)
    #     trans_w_s = np.array([p.x, p.y, p.z], dtype=np.float64)
    #     t_w_s = make_transform(rot_w_s, trans_w_s)

    #     with self._lock:
    #         self._latest_t_w_s = t_w_s
    #         self._latest_odom_time = msg.header.stamp
    #         self._odom_history.append((self._stamp_to_sec(msg.header.stamp), t_w_s.copy()))

    #     if not self._odom_received_once:
    #         self.get_logger().info("Received first odom message")
    #         self._odom_received_once = True
    def odom_callback(self, msg: Odometry) -> None:
        # 从里程计消息中提取位置与朝向（四元数）
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        # 构造传感器（sensor）在世界坐标系（world）中的位姿 T_w_s
        # 这里使用四元数转旋转矩阵函数 quat_to_rot_matrix，返回 3x3 旋转矩阵
        rot_w_s = quat_to_rot_matrix(q.x, q.y, q.z, q.w)
        # 平移向量 tx,ty,tz（确保为 float64 类型，便于后续矩阵运算）
        trans_w_s = np.array([p.x, p.y, p.z], dtype=np.float64)
        # 将旋转矩阵与平移组合为 4x4 齐次变换矩阵 T_w_s
        # 语义：将传感器系下一个点 P_s（齐次坐标）变换到世界系 P_w = T_w_s * P_s
        t_w_s = make_transform(rot_w_s, trans_w_s)

        # 使用锁保护共享状态更新（线程安全）
        # 因为回调可能并发触发，而其它线程/回调会读取这些字段（例如 image_callback）
        with self._lock:
            # 保存最新的变换（用以对图像做投影）
            self._latest_t_w_s = t_w_s
            # 保存最新里程计时间戳（ROS Header 时间），用于时间同步/匹配
            self._latest_odom_time = msg.header.stamp
            # 将 (timestamp_sec, transform) 推入历史队列，用于按时间选择最接近的 pose
            # 注意这里存的是秒级浮点时间，便于后续对比 image 时间戳
            self._odom_history.append((self._stamp_to_sec(msg.header.stamp), t_w_s.copy()))

        # 第一次收到里程计时记录一次日志（只记录一次以免刷屏）
        if not self._odom_received_once:
            self.get_logger().info("Received first odom message")
            self._odom_received_once = True
            
    def image_callback(self, msg: Image) -> None:
        # ============== 第一步：获取最新 odom 时间戳快照 ==============
        # 使用锁获取最近一次收到的里程计时间戳（用于后续状态显示）
        with self._lock:
            latest_odom_time = self._latest_odom_time

        # ============== 第二步：时间戳转换与 pose 选择 ==============
        # 将 ROS Header 中的 stamp（秒+纳秒）转为浮点秒
        # 时间戳语义：该图像在哪个时间点被采集的（相机的曝光中点时间）
        image_stamp_sec = self._stamp_to_sec(msg.header.stamp)
        
        # 根据图像时间从历史 odom deque 中选择最接近的 pose
        # 返回 (T_w_s, pose_age_sec)：
        #   - T_w_s: sensor 在 world 坐标系中的 4x4 齐次变换矩阵
        #   - pose_age_sec: 所选 pose 与图像的时间差（秒），None 表示历史为空
        t_w_s, pose_age_sec = self._select_pose_for_image(image_stamp_sec)

        # ============== 第三步：时间同步检查与告警 ==============
        # 参数 odom_sync.max_pose_age_sec（默认 0.25 秒，即 250 ms）
        # 含义：能接受的最大 pose-image 时间差；超过阈值表示两者不同步
        # 影响：若超过阈值会在画面显示警告（表示投影可能抖动或不准确）
        max_pose_age_sec = float(self.get_parameter("odom_sync.max_pose_age_sec").value)
        if pose_age_sec is not None and pose_age_sec > max_pose_age_sec:
            now = time.time()
            # 节流告警（不要刷屏），每 1.5 秒最多一次
            if now - self._last_pose_age_warn_time > 1.5:
                self.get_logger().warn(
                    f"Pose/Image timestamp gap is large: {pose_age_sec * 1000.0:.1f} ms. "
                    "Projection may jitter. Check sensor sync or odom/image rates."
                )
                self._last_pose_age_warn_time = now

        # ============== 第四步：AR 投影变换主链路 ==============
        # *** 这是 AR 定位的核心计算 ***
        # 已知条件：
        #   - 世界坐标系 {world}：九宫格所在的参考坐标系（来自 SLAM 重定位）
        #   - 传感器坐标系 {sensor}：里程计输出的坐标系（lidar origin）
        #   - 相机坐标系 {camera}：鱼眼相机的光学中心
        #   - T_w_s：sensor 在 world 中的位姿（来自 _select_pose_for_image）
        #   - T_c_s：camera 在 sensor 中的外参（已标定，存在 self.t_c_s）
        #
        # 投影流程（坐标系变换链）：
        #  1) 世界点 P_w 通过 T_c_w 变换到相机系：P_c = T_c_w * P_w
        #  2) T_c_w = T_c_s * T_s_w = T_c_s * inv(T_w_s)
        #     其中 T_s_w 是 T_w_s 的逆矩阵（world 到 sensor 的变换）
        #
        # 如果外参 T_c_s 标定有误（旋转/平移/坐标系约定），会导致：
        #   - 整体位移：外参平移错误
        #   - 旋转不对：外参旋转矩阵错误或四元数顺序错误
        #   - 镜像/翻转：坐标系轴方向约定不一致
        
        # 求逆得到从 world 指向 sensor 的变换
        t_s_w = inverse_transform(t_w_s)  # T_s_w = inv(T_w_s)
        # 最终的世界到相机变换（用于所有点的投影）
        t_c_w = self.t_c_s @ t_s_w      # T_c_w = T_c_s * T_s_w

        # ============== 第五步：图像获取与尺寸 ==============
        # 将 ROS Image 消息转为 OpenCV 格式（BGR8）
        # 后续所有绘制都在这个 image 上进行（最后发布或显示）
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = image.shape[:2]  # 图像高度和宽度（像素）

        # ============== 第六步：加载绘制参数 ==============
        # 以下参数控制九宫格在画面上的外观：
        # - draw.cell_border_color_bgr：格子边框颜色（BGR 顺序）
        # - draw.cell_border_thickness：格子边框线宽（像素）
        # - draw.cell_center_color_bgr：格子中心点颜色
        # - draw.cell_center_radius：格子中心点半径
        # - draw.label_color_bgr：格子 ID 文字颜色
        # - draw.show_labels：是否显示格子 ID 编号
        # - draw.show_status_text：是否显示上方状态栏（odom/front/visible/dt_ms）
        # - draw.curve_samples_per_edge：鱼眼相机下格子边界的采样点数（越多越光滑但越慢）
        
        border_color = tuple(int(v) for v in self.get_parameter("draw.cell_border_color_bgr").value)
        border_thickness = int(self.get_parameter("draw.cell_border_thickness").value)
        center_color = tuple(int(v) for v in self.get_parameter("draw.cell_center_color_bgr").value)
        center_radius = int(self.get_parameter("draw.cell_center_radius").value)
        center_thickness = int(self.get_parameter("draw.cell_center_thickness").value)
        label_color = tuple(int(v) for v in self.get_parameter("draw.label_color_bgr").value)
        show_labels = bool(self.get_parameter("draw.show_labels").value)
        show_status_text = bool(self.get_parameter("draw.show_status_text").value)
        curve_samples_per_edge = int(self.get_parameter("draw.curve_samples_per_edge").value)
        use_curved_grid = self._is_fisheye_model_selected()  # 鱼眼相机需要曲线采样边界

        # ============== 第七步：初始化消息与计数器 ==============
        # 构建 ROS GridCellArray 消息，用于发布到 /ar_grid/visible_cells 话题
        cells_msg = GridCellArray()
        cells_msg.header = msg.header  # 继承图像时间戳
        cells_msg.rows = -1
        cells_msg.cols = -1
        cells_msg.total_groups = len(self.grid_groups)
        cells_msg.visible_group_ids = []
        cells_msg.total_cells = self.total_cells
        cells_msg.visible_cells = 0  # 后续会累加
        cells_msg.cells = []
        cells_msg.visible_cell_ids = []

        visible_count = 0  # 统计有多少个格子成功投影到图像内
        front_count = 0    # 统计有多少个格子的中心点在相机前方（z > 0）
        visible_group_ids: Set[int] = set()

        if len(self.grid_groups) == 1:
            cells_msg.rows = self.grid_groups[0][1].rows
            cells_msg.cols = self.grid_groups[0][1].cols

        # ============== 第八步：逐格子投影处理 ==============
        # 对每个九宫格格子进行投影、可见性判定、绘制标注
        for group_id, grid_frame in self.grid_groups:
            for cell_id in sorted(grid_frame.cells.keys()):
                cell = grid_frame.cells[cell_id]

                # -------- 8.1) 格子中心点投影：world -> camera --------
                # 齐次坐标：加上第 4 个分量 1.0（便于 4x4 矩阵乘法）
                center_w_h = np.array([
                    cell.center_world[0],
                    cell.center_world[1],
                    cell.center_world[2],
                    1.0,
                ], dtype=np.float64)
                # 中心点在相机系的坐标（齐次）
                center_c_h = t_c_w @ center_w_h
                # 提取 xyz，丢弃齐次分量
                center_c = center_c_h[:3]
                # center_c[0], center_c[1]: 相机系中的 x, y 坐标
                # center_c[2]: 深度（z 需要 > 0 才能投影到图像前方）

                corners_px: List[Tuple[float, float]] = []  # 所有 4 个角点在图像中的像素坐标 (u, v)

                # -------- 8.2) 格子四角点投影与深度检查 --------
                corners_front = True  # 标志：是否所有 4 个角点都在相机前方
                for cw in cell.corners_world:
                    # 投影该角点：world -> camera
                    p_w_h = np.array([cw[0], cw[1], cw[2], 1.0], dtype=np.float64)
                    p_c_h = t_c_w @ p_w_h
                    p_c = p_c_h[:3]

                    # -------- 8.3) 深度判定 --------
                    # 如果深度 z <= 0，说明该点在相机后方或极近处，无法投影到图像
                    # 阈值 1e-6 是为了避免舍入误差（点在相机光心非常近）
                    if p_c[2] <= 1e-6:
                        corners_front = False
                        break

                    # -------- 8.4) 相机模型投影：camera 3D -> pixel 2D --------
                    # 使用已加载的相机模型（pinhole 或 fisheye）进行投影
                    # 返回 (u, v) 像素坐标，或 None 如果投影失败（畸变模型异常、点太靠近边缘等）
                    uv = self.camera_model.project_point(p_c)
                    if uv is None:
                        corners_front = False
                        break
                    corners_px.append(uv)

                # -------- 8.5) 中心点投影与计数 --------
                # 统计有多少格子的中心点在相机前方
                if center_c[2] > 1e-6:
                    front_count += 1

                # 中心点也需要投影检查（深度合法性 + 投影失败处理）
                center_uv = self.camera_model.project_point(center_c) if center_c[2] > 1e-6 else None
                # 判断中心点是否落在图像边界内
                center_in_image = (
                    center_uv is not None and 0 <= center_uv[0] < w and 0 <= center_uv[1] < h
                )
            
                # -------- 8.6) 可见性判定逻辑 --------
                # 可见条件A：至少一个角点在图像内，且格子投影面积有至少 1/4 落在图像内
                # 可见条件B：格子中心点在图像内（近距离包裹图像场景）
                has_valid_corners = corners_front and len(corners_px) == 4
                any_corner_in_image = has_valid_corners and any(
                    0 <= u < w and 0 <= v < h for (u, v) in corners_px
                )
                visible_area_ratio = (
                    self._compute_projected_visible_area_ratio(corners_px, w, h)
                    if has_valid_corners else 0.0
                )
                visible_by_area = bool(any_corner_in_image and visible_area_ratio >= 0.25)
                visible_by_center = bool(center_in_image)

                visible = bool(has_valid_corners and (visible_by_area or visible_by_center))
                if visible:
                    visible_count += 1
                    visible_group_ids.add(int(group_id))

                # -------- 8.7) 构建 GridCell 消息 --------
                # 每个格子对应一条 GridCell 消息，包含其投影结果与可见性
                cell_msg = GridCell()
                cell_msg.header = msg.header  # 继承图像时间戳
                cell_msg.group_id = int(group_id)
                cell_msg.cell_id = int(cell.cell_id)  # 组内格子编号
                cell_msg.row = int(cell.row)  # 行号
                cell_msg.col = int(cell.col)  # 列号
                cell_msg.is_visible = bool(visible)

                # 格子在世界坐标系中的中心位置（固定，不随相机移动变化）
                cell_msg.position_world_frame = Point(
                    x=float(cell.center_world[0]),
                    y=float(cell.center_world[1]),
                    z=float(cell.center_world[2]),
                )

                # -------- 8.8) 可见格子的详细投影信息 --------
                if visible:
                    # 格子在相机坐标系中的中心位置（深度信息用于距离计算）
                    cell_msg.position_camera_frame = Point(
                        x=float(center_c[0]),
                        y=float(center_c[1]),
                        z=float(center_c[2]),  # 深度，单位：米
                    )
                    # 格子四个角点的像素坐标（这是最终要在图像上绘制的位置）
                    # ordered[i] = (u, v) 表示第 i 角的像素坐标
                    ordered = [
                        (float(corners_px[0][0]), float(corners_px[0][1])),
                        (float(corners_px[1][0]), float(corners_px[1][1])),
                        (float(corners_px[2][0]), float(corners_px[2][1])),
                        (float(corners_px[3][0]), float(corners_px[3][1])),
                    ]

                    # 保存四个角点的像素坐标到消息
                    cell_msg.corners_pixel = [
                        Point32(x=float(ordered[0][0]), y=float(ordered[0][1]), z=0.0),
                        Point32(x=float(ordered[1][0]), y=float(ordered[1][1]), z=0.0),
                        Point32(x=float(ordered[2][0]), y=float(ordered[2][1]), z=0.0),
                        Point32(x=float(ordered[3][0]), y=float(ordered[3][1]), z=0.0),
                    ]

                    # 中心像素优先采用直接投影值，避免近距离大视角下角点均值失真
                    if center_uv is not None:
                        center_u = float(center_uv[0])
                        center_v = float(center_uv[1])
                    else:
                        center_u = sum(p[0] for p in ordered) / 4.0
                        center_v = sum(p[1] for p in ordered) / 4.0
                    cell_msg.center_pixel = Point32(x=float(center_u), y=float(center_v), z=0.0)

                    cv_pts = np.array([[int(round(u)), int(round(v))] for (u, v) in ordered], dtype=np.int32)
                    if use_curved_grid:
                        world_corners = cell.corners_world
                        self._draw_sampled_world_edge(
                            image,
                            world_corners[0],
                            world_corners[1],
                            t_c_w,
                            border_color,
                            border_thickness,
                            curve_samples_per_edge,
                        )
                        self._draw_sampled_world_edge(
                            image,
                            world_corners[1],
                            world_corners[2],
                            t_c_w,
                            border_color,
                            border_thickness,
                            curve_samples_per_edge,
                        )
                        self._draw_sampled_world_edge(
                            image,
                            world_corners[2],
                            world_corners[3],
                            t_c_w,
                            border_color,
                            border_thickness,
                            curve_samples_per_edge,
                        )
                        self._draw_sampled_world_edge(
                            image,
                            world_corners[3],
                            world_corners[0],
                            t_c_w,
                            border_color,
                            border_thickness,
                            curve_samples_per_edge,
                        )
                    else:
                        cv2.polylines(image, [cv_pts], isClosed=True, color=border_color, thickness=border_thickness, lineType=cv2.LINE_AA)

                    c_u_i, c_v_i = int(round(center_u)), int(round(center_v))
                    # -------- 8.10) 格子中心点绘制 - 用圆点标记格子中心 --------
                    cv2.circle(image, (c_u_i, c_v_i), center_radius, center_color, center_thickness, lineType=cv2.LINE_AA)
                    
                    # -------- 8.11) 可选的格子标签 --------
                    # 显示“组号+编号”，便于多组调试
                    if show_labels:
                        cv2.putText(
                            image,
                            f"G{group_id}-C{cell.cell_id}",
                            (c_u_i + 5, c_v_i - 5),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            label_color,
                            2,
                            lineType=cv2.LINE_AA,
                        )

                    # -------- 8.12) 记录可见格子信息 --------
                    # 跟踪此可见格子的 ID 和深度，用于下游处理
                    cells_msg.visible_cell_ids.append(int(cell.cell_id))
                    cell_msg.depth = float(center_c[2])
                else:
                    # -------- 8.13) 不可见格子用零值填充 --------
                    # 对于不可见的格子，用零值占位来减小消息大小
                    # （约减少 50% 的消息量，改善 ROS 带宽效率）
                    cell_msg.position_camera_frame = Point(x=0.0, y=0.0, z=0.0)
                    cell_msg.corners_pixel = [
                        Point32(x=0.0, y=0.0, z=0.0),
                        Point32(x=0.0, y=0.0, z=0.0),
                        Point32(x=0.0, y=0.0, z=0.0),
                        Point32(x=0.0, y=0.0, z=0.0),
                    ]
                    cell_msg.center_pixel = Point32(x=0.0, y=0.0, z=0.0)
                    cell_msg.depth = 0.0
                cells_msg.cells.append(cell_msg)

        # ============== 第九步：完成消息并发布 GridCellArray ==============
        # 设置可见格子计数并发布完整的投影结果
        # 下游节点（如 UI 可视化、坐标转换）可订阅此话题
        cells_msg.visible_group_ids = sorted(visible_group_ids)
        cells_msg.visible_cells = int(visible_count)
        self.visible_cells_pub.publish(cells_msg)

        if show_status_text:
            # ============== 第十步：状态栏文本生成 ============== 
            # 在图像上显示实时状态：里程计状态 + 格子计数 + 时间戳差
            has_odom = latest_odom_time is not None
            # 状态格式：odom_状态 正面个数/总数 可见个数/总数 时间差_ms
            # 示例："odom=ok front=7/9 visible=6/9 dt_ms=12.5"
            # - odom: 'ok' 表示已收到里程计，'missing' 表示超时
            # - front: 有多少格子的中心在相机前方 (z>0)
            # - visible: 有多少格子实际投影到图像上
            # - dt_ms: 图像与里程计的时间差（毫秒）
            status = (
                f"odom={'ok' if has_odom else 'missing'} "
                f"front={front_count}/{self.total_cells} "
                f"visible={visible_count}/{self.total_cells} "
                f"dt_ms={(pose_age_sec * 1000.0):.1f}" if pose_age_sec is not None else
                f"odom={'ok' if has_odom else 'missing'} front={front_count}/{self.total_cells} visible={visible_count}/{self.total_cells} dt_ms=n/a"
            )
            
            # 在图像左上角 (10, 28) 绘制状态文本
            # 颜色：里程计 ok 时为绿色，missing 时为红色（警告）
            cv2.putText(
                image,
                status,
                (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0) if has_odom else (0, 0, 255),
                2,
                lineType=cv2.LINE_AA,
            )

            if not has_odom:
                cv2.putText(
                    image,
                    "未收到里程计数据，请检查配置的话题",
                    (10, 56),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2,
                    lineType=cv2.LINE_AA,
                )

        # ============== 第十一步：诊断日志 ============== 
        # 如果 2+ 秒内没有可见格子，则记录警告
        # 表示可能存在：传感器故障、参数不匹配、坐标系约定错误
        now = time.time()
        if now - self._last_diag_log_time > 2.0 and visible_count == 0:
            self.get_logger().warn(
                "没有投影的可见格子。请检查：里程计话题、外参、相机模型/内参和坐标系约定。"
            )
            self._last_diag_log_time = now

        # ============== 第十二步：准备输出图像消息 ============== 
        # 将标注后的 OpenCV 图像转换回 ROS 消息格式
        # out_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        # out_msg.header = msg.header
        # self.image_pub.publish(out_msg)

        # ============== 第十三步：可选的 OpenCV 窗口显示 ============== 
        # 如果启用 draw.show_window，显示实时可视化（Headless 系统可禁用）
        if bool(self.get_parameter("draw.show_window").value):
            window_name = str(self.get_parameter("draw.window_name").value)
            try:
                cv2.imshow(window_name, image)
                cv2.waitKey(1)  # 1ms 延迟用于处理窗口事件
            except cv2.error as exc:
                if not self._window_error_logged:
                    self.get_logger().warn(
                        f"OpenCV 窗口显示失败（Headless/无 GUI）：{exc}。"
                        "请设置 draw.show_window:=false 来禁用此警告。"
                    )
                    self._window_error_logged = True


def main() -> None:
    rclpy.init()
    node = ArGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
