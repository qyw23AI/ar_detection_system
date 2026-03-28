# 单个格子单元信息
# Single grid cell information

std_msgs/Header header

# 格子编号 (行优先，从0开始: row * cols + col)
# Grid cell ID (row-major, starting from 0: row * cols + col)
int32 cell_id

# 格子组编号（由配置给定）
# Grid group ID (configured per group)
int32 group_id

# 格子行列索引
# Row and column indices
int32 row
int32 col

# 是否在相机视野内可见
# Whether the cell is visible in camera view
bool is_visible

# 格子中心在相机坐标系下的位置 (x, y, z in meters)
# Cell center position in camera frame (x, y, z in meters)
geometry_msgs/Point position_camera_frame

# 格子中心在世界坐标系下的位置
# Cell center position in world frame
geometry_msgs/Point position_world_frame

# 格子四个角点在图像上的像素坐标 (u, v)
# 顺序: 左上(0), 右上(1), 右下(2), 左下(3)
# Four corner pixel coordinates in image (u, v)
# Order: top-left(0), top-right(1), bottom-right(2), bottom-left(3)
geometry_msgs/Point32[4] corners_pixel

# 格子中心在图像上的像素坐标
# Cell center pixel coordinates in image
geometry_msgs/Point32 center_pixel

# 格子在相机坐标系下的深度 (z值，单位米)
# Cell depth in camera frame (z value, in meters)
float32 depth
