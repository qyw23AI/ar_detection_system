"""
格子架生成器模块
Grid Frame Generator Module

根据三个角点（左上、右上、左下）和格子参数生成任意大小的格子架
Generate arbitrary grid frames from three corner points (top-left, top-right, bottom-left) and grid parameters
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np


@dataclass
class GridCell:
    """
    单个格子单元的数据结构。

    字段:
    - `cell_id`: 格子全局编号，按行优先（row * cols + col）。
    - `row`, `col`: 行/列索引，从0开始。
    - `center_world`: 格子中心在世界坐标系下的三维坐标 `[x, y, z]`。
    - `corners_world`: 四个角点在世界坐标系下的坐标列表，顺序为
        `[top_left, top_right, bottom_right, bottom_left]`。

    说明:
    - 构造后会将输入坐标转换为 `np.ndarray(dtype=float64)` 以保证数值计算一致性。
    - 此结构仅承载几何数据，不包含可见性或像素信息。
    """
    cell_id: int           # 格子编号 (行优先: row * cols + col)
    row: int               # 行索引 (从0开始)
    col: int               # 列索引 (从0开始)
    center_world: np.ndarray  # 格子中心世界坐标 [x, y, z]
    corners_world: List[np.ndarray]  # 四个角点世界坐标 [左上, 右上, 右下, 左下]

    def __post_init__(self):
        # 统一为 numpy 数组，便于后续线性代数运算
        self.center_world = np.asarray(self.center_world, dtype=np.float64)
        self.corners_world = [np.asarray(c, dtype=np.float64) for c in self.corners_world]


@dataclass
class GridFrame:
    """
    整个格子架的数据结构。

    字段:
    - `rows`, `cols`: 格子数（行、列）。
    - `cell_width`, `cell_height`: 单元格的几何尺寸（米）。这些值由生成函数
      从输入角点计算得到，并保存在此结构中以供外部查询。
    - `cells`: 一个字典，键为 `cell_id`，值为 `GridCell` 实例，包含每个单元的几何信息。
    - `corner_top_left`, `corner_top_right`, `corner_bottom_left`: 定义外框的三个世界坐标角点，
      与 `GridFrameGenerator.generate_from_three_corners` 的输入语义一致。

    方法说明:
    - `total_cells`: 返回格子总数（rows * cols）。
    - `get_cell(row, col)`: 根据行列索引返回对应的 `GridCell`（若不存在返回 None）。
    - `get_cell_by_id(cell_id)`: 根据 `cell_id` 返回 `GridCell`。
    - `get_all_corners_world()`: 返回所有格子四个角点的扁平列表，顺序未保证，但适合批量投影。
    - `get_all_centers_world()`: 返回 `(cell_id, center_world)` 的列表，便于迭代处理。
    """
    rows: int                    # 行数
    cols: int                    # 列数
    cell_width: float            # 单元格宽度 (米)
    cell_height: float           # 单元格高度 (米)
    cells: Dict[int, GridCell] = field(default_factory=dict)  # 所有格子 {cell_id: GridCell}
    
    # 定义格子架的三个角点 (世界坐标)
    corner_top_left: np.ndarray = field(default_factory=lambda: np.zeros(3))
    corner_top_right: np.ndarray = field(default_factory=lambda: np.zeros(3))
    corner_bottom_left: np.ndarray = field(default_factory=lambda: np.zeros(3))
    
    @property
    def total_cells(self) -> int:
        return self.rows * self.cols
    
    def get_cell(self, row: int, col: int) -> Optional[GridCell]:
        """根据行列索引返回 `GridCell`，若越界或不存在返回 None。"""
        cell_id = row * self.cols + col
        return self.cells.get(cell_id)
    
    def get_cell_by_id(self, cell_id: int) -> Optional[GridCell]:
        """根据 `cell_id` 返回 `GridCell`。"""
        return self.cells.get(cell_id)
    
    def get_all_corners_world(self) -> List[np.ndarray]:
        """
        返回所有格子角点的世界坐标扁平列表：
        [cell0.top_left, cell0.top_right, cell0.bottom_right, cell0.bottom_left, cell1.top_left, ...]
        该输出便于一次性投影所有角点以提高效率。
        """
        corners = []
        for cell in self.cells.values():
            corners.extend(cell.corners_world)
        return corners
    
    def get_all_centers_world(self) -> List[Tuple[int, np.ndarray]]:
        """
        返回每个格子的 `(cell_id, center_world)` 列表，便于按 id 访问和后续变换。
        """
        return [(cell.cell_id, cell.center_world) for cell in self.cells.values()]


class GridFrameGenerator:
    """
    格子架生成器
    
    通过三个角点（左上、右上、左下）定义格子架平面，
    再按 rows/cols 将“外框”均匀切分生成所有格子。
    
    坐标系约定:
    - 左上角(top_left)为原点
    - 右方向为列增加方向 (X正向)
    - 下方向为行增加方向 (Y正向)
    """
    
    @staticmethod
    def generate_from_three_corners(
        corner_top_left: np.ndarray,
        corner_top_right: np.ndarray,
        corner_bottom_left: np.ndarray,
        rows: int,
        cols: int,
        cell_width: Optional[float] = None,
        cell_height: Optional[float] = None,
        strict_size_check: bool = False,
        size_tolerance: float = 1e-4,
    ) -> GridFrame:
        """
        从三个外角点生成一个完整的 `GridFrame`（按行列均匀分割外框）。

        数学与实现要点：
        - 输入点：TL (top-left), TR (top-right), BL (bottom-left)，均为世界坐标系下的 3D 向量。
        - 计算两条边向量：
            vec_right = TR - TL
            vec_down  = BL - TL
        - 总宽高：
            total_width = ||vec_right||
            total_height = ||vec_down||
        - 单元尺寸（从外框推导）：
            inferred_cell_width  = total_width / cols
            inferred_cell_height = total_height / rows
        - 单元格内任意格子 (r, c) 的左上角坐标：
            P_tl(r,c) = TL + c * inferred_cell_width * unit_right + r * inferred_cell_height * unit_down
          其余三个角按列/行索引 +1 推导得到。

        参数说明：
        - `cell_width`/`cell_height` 为可选的用户显式尺寸，仅用于与推导值比较以检测配置错误；
          若 `strict_size_check=True` 且差异超出 `size_tolerance` 则抛出 `ValueError`。
        - 若输入的两个主方向长度接近 0，会使用默认轴避免除零（但此时几何无意义，应由上层检查）。

        边界情况：
        - `rows` 或 `cols` 必须为正整数（调用处通常会确保）；
        - 当 `total_width` 或 `total_height` 非法时（接近 0），会回退为单位轴，生成的格子可能不是用户期望的。

        Returns:
            已填充 `GridFrame` 对象，包含 `cells` 字典，每个 `GridCell` 含中心与四角世界坐标。
        """
        tl = np.asarray(corner_top_left, dtype=np.float64)
        tr = np.asarray(corner_top_right, dtype=np.float64)
        bl = np.asarray(corner_bottom_left, dtype=np.float64)
        
        # 计算外框两个主方向向量（注意：这里假设输入是“外角点”）
        # vec_right: 从左上到右上，在格子定义中对应“列方向”的向量
        vec_right = tr - tl  # 向右方向（沿列增加）
        # vec_down: 从左上到左下，在格子定义中对应“行方向”的向量
        vec_down = bl - tl   # 向下方向（沿行增加）
        
        # 计算整体尺寸
        total_width = np.linalg.norm(vec_right)
        total_height = np.linalg.norm(vec_down)
        
        # 归一化得到单位方向向量（若长度接近0，使用默认轴以避免除0）
        unit_right = vec_right / total_width if total_width > 1e-10 else np.array([1, 0, 0])
        unit_down = vec_down / total_height if total_height > 1e-10 else np.array([0, 1, 0])
        
        # 由外框角点和行列数直接推导单格尺寸：
        # cell_width  = ||TR - TL|| / cols
        # cell_height = ||BL - TL|| / rows
        inferred_cell_width = total_width / cols
        inferred_cell_height = total_height / rows

        # 外部传入的 cell_width/cell_height 仅用于校验（不参与构造）。
        # 如果用户要求严格校验（strict_size_check=True），且传入值与推导值
        # 差异超过容差（size_tolerance），则抛出异常以提示配置不一致。
        if cell_width is not None and abs(cell_width - inferred_cell_width) > size_tolerance and strict_size_check:
            raise ValueError(
                f"grid.cell_width ({cell_width}) != inferred width ({inferred_cell_width}) from three corners"
            )
        if cell_height is not None and abs(cell_height - inferred_cell_height) > size_tolerance and strict_size_check:
            raise ValueError(
                f"grid.cell_height ({cell_height}) != inferred height ({inferred_cell_height}) from three corners"
            )

        cell_width = inferred_cell_width
        cell_height = inferred_cell_height
        
        # 创建格子架对象并保存外框角点与单元尺寸
        grid_frame = GridFrame(
            rows=rows,
            cols=cols,
            cell_width=cell_width,
            cell_height=cell_height,
            corner_top_left=tl,
            corner_top_right=tr,
            corner_bottom_left=bl
        )
        
        # 生成所有格子
        for row in range(rows):
            for col in range(cols):
                cell_id = row * cols + col
                
                # 计算格子四个角点
                # 左上角
                # 使用向量参数化得到四个角点的世界坐标（按行列索引偏移）
                corner_tl = tl + unit_right * (col * cell_width) + unit_down * (row * cell_height)
                # 右上角：列索引 +1，行索引不变
                corner_tr = tl + unit_right * ((col + 1) * cell_width) + unit_down * (row * cell_height)
                # 右下角：列+1，行+1
                corner_br = tl + unit_right * ((col + 1) * cell_width) + unit_down * ((row + 1) * cell_height)
                # 左下角：列不变，行+1
                corner_bl = tl + unit_right * (col * cell_width) + unit_down * ((row + 1) * cell_height)
                
                # 计算中心点
                # 中心点取四角点均值（在平面网格上等价于单元的几何中心）
                center = (corner_tl + corner_tr + corner_br + corner_bl) / 4.0
                
                cell = GridCell(
                    cell_id=cell_id,
                    row=row,
                    col=col,
                    center_world=center,
                    corners_world=[corner_tl, corner_tr, corner_br, corner_bl]
                )
                
                grid_frame.cells[cell_id] = cell
        
        return grid_frame
    
    @staticmethod
    def generate_nine_grid(
        corner_top_left: np.ndarray,
        corner_top_right: np.ndarray,
        corner_bottom_left: np.ndarray,
        cell_width: Optional[float] = None,
        cell_height: Optional[float] = None
    ) -> GridFrame:
        """
        生成标准九宫格（3x3）。

        这是 `generate_from_three_corners` 的薄封装，固定 `rows=3, cols=3`。

        Args:
            corner_top_left, corner_top_right, corner_bottom_left: 三个外角点（世界坐标）。
            cell_width/cell_height: 可选尺寸校验参数，传入 None 则仅使用角点推导。

        Returns:
            `GridFrame` 实例（3x3）。
        """
        return GridFrameGenerator.generate_from_three_corners(
            corner_top_left, corner_top_right, corner_bottom_left,
            rows=3, cols=3,
            cell_width=cell_width, cell_height=cell_height
        )

    @staticmethod
    def generate_from_center_points(
        center_top_left: np.ndarray,
        center_top_right: np.ndarray,
        center_bottom_left: Optional[np.ndarray],
        rows: int,
        cols: int,
        cell_width: Optional[float] = None,
        cell_height: Optional[float] = None,
        size_source: str = "inferred",
        strict_size_check: bool = False,
        size_tolerance: float = 1e-4,
        single_row_down_axis: Optional[np.ndarray] = None,
    ) -> GridFrame:
        if rows <= 0 or cols <= 0:
            raise ValueError("rows/cols must be > 0")

        ctl = np.asarray(center_top_left, dtype=np.float64)
        ctr = np.asarray(center_top_right, dtype=np.float64)
        if ctl.size != 3 or ctr.size != 3:
            raise ValueError("center_top_left/center_top_right must be [x,y,z]")

        if center_bottom_left is not None:
            cbl = np.asarray(center_bottom_left, dtype=np.float64)
            if cbl.size != 3:
                raise ValueError("center_bottom_left must be [x,y,z]")
        else:
            cbl = None

        vec_right_total = ctr - ctl
        right_total_norm = np.linalg.norm(vec_right_total)
        if cols > 1:
            if right_total_norm <= 1e-10:
                raise ValueError("center_top_left and center_top_right are too close")
            unit_right = vec_right_total / right_total_norm
            inferred_cell_width = right_total_norm / float(cols - 1)
        else:
            inferred_cell_width = cell_width if (cell_width is not None and cell_width > 0.0) else 0.0
            unit_right = np.array([1.0, 0.0, 0.0], dtype=np.float64)

        if rows > 1:
            if cbl is None:
                raise ValueError("center_bottom_left is required when rows > 1")
            vec_down_total = cbl - ctl
            down_total_norm = np.linalg.norm(vec_down_total)
            if down_total_norm <= 1e-10:
                raise ValueError("center_top_left and center_bottom_left are too close")
            unit_down = vec_down_total / down_total_norm
            inferred_cell_height = down_total_norm / float(rows - 1)
        else:
            down_hint = np.asarray(
                single_row_down_axis if single_row_down_axis is not None else [0.0, 0.0, -1.0],
                dtype=np.float64,
            ).reshape(-1)
            if down_hint.size != 3:
                raise ValueError("single_row_down_axis must be [x,y,z]")
            down_proj = down_hint - np.dot(down_hint, unit_right) * unit_right
            down_proj_norm = np.linalg.norm(down_proj)
            if down_proj_norm <= 1e-10:
                fallback = np.array([0.0, 0.0, 1.0], dtype=np.float64)
                if abs(np.dot(fallback, unit_right)) > 0.95:
                    fallback = np.array([0.0, 1.0, 0.0], dtype=np.float64)
                down_proj = fallback - np.dot(fallback, unit_right) * unit_right
                down_proj_norm = np.linalg.norm(down_proj)
            if down_proj_norm <= 1e-10:
                raise ValueError("single_row_down_axis is degenerate")
            unit_down = down_proj / down_proj_norm
            if cell_height is not None and cell_height > 0.0:
                inferred_cell_height = float(cell_height)
            elif inferred_cell_width > 0.0:
                inferred_cell_height = float(inferred_cell_width)
            else:
                raise ValueError("rows=1 requires cell_height or valid horizontal center spacing")

        normalized_size_source = str(size_source).strip().lower()
        if normalized_size_source not in ("inferred", "configured"):
            raise ValueError("size_source must be 'inferred' or 'configured'")

        cfg_cell_width = float(cell_width) if (cell_width is not None and cell_width > 0.0) else None
        cfg_cell_height = float(cell_height) if (cell_height is not None and cell_height > 0.0) else None

        if normalized_size_source == "configured":
            if cfg_cell_width is None:
                raise ValueError("size_source=configured requires positive cell_width")
            if cfg_cell_height is None:
                raise ValueError("size_source=configured requires positive cell_height")
            resolved_cell_width = cfg_cell_width
            resolved_cell_height = cfg_cell_height
        else:
            resolved_cell_width = inferred_cell_width
            resolved_cell_height = inferred_cell_height
            if strict_size_check:
                if cfg_cell_width is not None and abs(cfg_cell_width - inferred_cell_width) > size_tolerance:
                    raise ValueError(
                        f"grid.cell_width ({cfg_cell_width}) != inferred width ({inferred_cell_width}) from center points"
                    )
                if cfg_cell_height is not None and abs(cfg_cell_height - inferred_cell_height) > size_tolerance:
                    raise ValueError(
                        f"grid.cell_height ({cfg_cell_height}) != inferred height ({inferred_cell_height}) from center points"
                    )

        if resolved_cell_width <= 1e-10 or resolved_cell_height <= 1e-10:
            raise ValueError("resolved cell size must be positive")

        half_right = unit_right * (resolved_cell_width / 2.0)
        half_down = unit_down * (resolved_cell_height / 2.0)

        corner_top_left = ctl - half_right - half_down
        corner_top_right = ctl + unit_right * (cols * resolved_cell_width - resolved_cell_width / 2.0) - half_down
        corner_bottom_left = ctl + unit_down * (rows * resolved_cell_height - resolved_cell_height / 2.0) - half_right

        grid_frame = GridFrame(
            rows=rows,
            cols=cols,
            cell_width=resolved_cell_width,
            cell_height=resolved_cell_height,
            corner_top_left=corner_top_left,
            corner_top_right=corner_top_right,
            corner_bottom_left=corner_bottom_left,
        )

        for row in range(rows):
            for col in range(cols):
                cell_id = row * cols + col
                center = ctl + unit_right * (col * resolved_cell_width) + unit_down * (row * resolved_cell_height)
                corner_tl = center - half_right - half_down
                corner_tr = center + half_right - half_down
                corner_br = center + half_right + half_down
                corner_bl = center - half_right + half_down

                grid_frame.cells[cell_id] = GridCell(
                    cell_id=cell_id,
                    row=row,
                    col=col,
                    center_world=center,
                    corners_world=[corner_tl, corner_tr, corner_br, corner_bl],
                )

        return grid_frame


def convert_cell_centers_to_corners(
    center_top_left: np.ndarray,
    center_top_right: np.ndarray,
    center_bottom_left: np.ndarray,
    rows: int,
    cols: int,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    将三个格子中心坐标转换为格子架的三个外角点坐标（外框角点）。

    场景与目的：用户可能在标定时记录的是每个格子中心的位置（例如左上格子中心、
    右上格子中心、左下格子中心）。而 `GridFrameGenerator` 期望使用外框的三个角点来
    构造整个网格。此函数完成二者语义的转换。

    计算方法：设 ctl, ctr, cbl 分别为左上/右上/左下格子中心。
    - 总水平向量（跨越 cols-1 个格子）： vec_right_total = ctr - ctl
    - 总垂直向量（跨越 rows-1 个格子）：   vec_down_total  = cbl - ctl
    - 单个格子宽/高向量分别为：
        cell_width_vec  = vec_right_total / (cols - 1)   (若 cols==1 则为零向量)
        cell_height_vec = vec_down_total  / (rows - 1)   (若 rows==1 则为零向量)
    - 外角点推导：
        corner_top_left  = ctl - 0.5*cell_width_vec - 0.5*cell_height_vec
        corner_top_right = ctr + 0.5*cell_width_vec - 0.5*cell_height_vec
        corner_bottom_left= cbl - 0.5*cell_width_vec + 0.5*cell_height_vec

    边界处理：
    - 当 `cols == 1`（仅一列）时，水平向量无法从两个中心点推断，函数使用零向量，
      这会导致外角点在水平方向上与中心对齐；调用方应注意这种特殊情况。
    - 当 `rows == 1`（仅一行）时，垂直向量同理处理为零向量。

    Args:
        center_top_left: 左上格子的中心坐标 [x, y, z]
        center_top_right: 右上格子的中心坐标 [x, y, z]
        center_bottom_left: 左下格子的中心坐标 [x, y, z]
        rows: 格子架行数
        cols: 格子架列数

    Returns:
        三个外角点 `(corner_top_left, corner_top_right, corner_bottom_left)`（world coords）。
    """
    ctl = np.asarray(center_top_left, dtype=np.float64)
    ctr = np.asarray(center_top_right, dtype=np.float64)
    cbl = np.asarray(center_bottom_left, dtype=np.float64)
    
    # 从左上中心到右上中心的向量，跨越 (cols-1) 个格子
    vec_right_total = ctr - ctl
    # 从左上中心到左下中心的向量，跨越 (rows-1) 个格子
    vec_down_total = cbl - ctl
    
    # 单个格子的宽度和高度向量
    if cols > 1:
        cell_width_vec = vec_right_total / (cols - 1)
    else:
        # 只有一列时，无法从水平方向推断宽度，使用默认或抛异常
        cell_width_vec = np.zeros(3)
    
    if rows > 1:
        cell_height_vec = vec_down_total / (rows - 1)
    else:
        # 只有一行时，无法从垂直方向推断高度，使用默认或抛异常
        cell_height_vec = np.zeros(3)
    
    # 格子架的外角点 = 对应格子中心 - 半个格子尺寸（向外偏移）
    # 左上外角 = 左上格子中心 - 0.5*宽度向量 - 0.5*高度向量
    corner_top_left = ctl - cell_width_vec / 2 - cell_height_vec / 2
    # 右上外角 = 右上格子中心 + 0.5*宽度向量 - 0.5*高度向量
    corner_top_right = ctr + cell_width_vec / 2 - cell_height_vec / 2
    # 左下外角 = 左下格子中心 - 0.5*宽度向量 + 0.5*高度向量
    corner_bottom_left = cbl - cell_width_vec / 2 + cell_height_vec / 2
    
    return corner_top_left, corner_top_right, corner_bottom_left


def convert_legacy_nine_grid_points(nine_points: Dict[int, np.ndarray]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    将旧版九宫格点（按 1..9 编号）转换为三角点（TL/TR/BL）格式。

    旧编号布局（每个点语义为对应格子的中心）：
        1  2  3
        4  5  6
        7  8  9

    本函数从中抽取 1, 3, 7（三个角落的格子中心）并调用
    `convert_cell_centers_to_corners(..., rows=3, cols=3)` 以得到外角点。

    Args:
        nine_points: 字典，键为 1..9，值为对应点的 `[x,y,z]` 数组或可转换向量。

    Returns:
        `(corner_top_left, corner_top_right, corner_bottom_left)` 三个外角点（world coords）。

    备注：如果输入缺失或格式不正确（例如某个索引不存在或长度不是3），
    上层调用应当在加载阶段进行校验并抛出异常。
    """
    p1 = np.asarray(nine_points[1], dtype=np.float64)  # 左上格子中心
    p3 = np.asarray(nine_points[3], dtype=np.float64)  # 右上格子中心
    p7 = np.asarray(nine_points[7], dtype=np.float64)  # 左下格子中心

    # 对于 3x3 九宫格，使用通用转换函数
    return convert_cell_centers_to_corners(p1, p3, p7, rows=3, cols=3)
