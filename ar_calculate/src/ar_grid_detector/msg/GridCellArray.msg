# 格子数组消息
# Grid cell array message

std_msgs/Header header

# 格子架的行数和列数
# Number of rows and columns in the grid frame
# 多组场景下该字段不再表示全局统一尺寸，固定为 -1
int32 rows
int32 cols

# 组总数
# Total number of configured grid groups
int32 total_groups

# 当前帧可见的组编号列表
# Group IDs that have at least one visible cell in current frame
int32[] visible_group_ids

# 总格子数
# Total number of cells
int32 total_cells

# 可见格子数
# Number of visible cells
int32 visible_cells

# 所有格子信息（包括不可见的）
# All cell information (including invisible ones)
ar_grid_detector/GridCell[] cells

# 可见格子ID列表（方便快速索引）
# List of visible cell IDs (for quick indexing)
int32[] visible_cell_ids
