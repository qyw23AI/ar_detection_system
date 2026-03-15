#!/usr/bin/env bash
set -euo pipefail

# 项目路径配置
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAIN_WORKSPACE="/home/r1/9_grid_ar_detection/FAST_LIVO2_ROS2_relocation_ultra"
AR_WORKSPACE="${SCRIPT_DIR}"
LOG_DIR="${SCRIPT_DIR}/log/one_click"
mkdir -p "${LOG_DIR}"
TS="$(date +%Y%m%d_%H%M%S)"

echo "[INFO] Script directory: ${SCRIPT_DIR}"
echo "[INFO] Main workspace: ${MAIN_WORKSPACE}"
echo "[INFO] AR workspace: ${AR_WORKSPACE}"

if command -v conda >/dev/null 2>&1; then
  conda deactivate >/dev/null 2>&1 || true
fi

# Source ROS2 基础环境
if [[ -f "/opt/ros/humble/setup.bash" ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  echo "[OK] ROS2 Humble sourced"
else
  echo "[ERROR] /opt/ros/humble/setup.bash not found"
  exit 1
fi

# Source 主工作空间 (FAST-LIVO2)
if [[ -f "${MAIN_WORKSPACE}/install/setup.bash" ]]; then
  set +u
  source "${MAIN_WORKSPACE}/install/setup.bash"
  set -u
  echo "[OK] Main workspace sourced: ${MAIN_WORKSPACE}"
else
  echo "[ERROR] ${MAIN_WORKSPACE}/install/setup.bash not found"
  echo "[ERROR] Please build the main workspace first:"
  echo "  cd ${MAIN_WORKSPACE} && colcon build"
  exit 1
fi

# Source AR 工作空间 (可选，如果存在)
if [[ -f "${AR_WORKSPACE}/install/setup.bash" ]]; then
  set +u
  source "${AR_WORKSPACE}/install/setup.bash"
  set -u
  echo "[OK] AR workspace sourced: ${AR_WORKSPACE}"
else
  echo "[WARN] AR workspace not built, AR overlay will not be available"
  echo "       To build: cd ${AR_WORKSPACE} && colcon build"
fi


PIDS=()
ENABLE_CAMERA_DRIVER="${ENABLE_CAMERA_DRIVER:-1}"
ENABLE_AR_DETECTOR="${ENABLE_AR_DETECTOR:-1}"
ENABLE_RELOCALIZATION="${ENABLE_RELOCALIZATION:-1}"

# 相机后端选择: realsense | usb | none
CAMERA_BACKEND="${CAMERA_BACKEND:-realsense}"

# 相机启动命令（可通过环境变量覆盖）
REALSENSE_CAMERA_LAUNCH_CMD="${REALSENSE_CAMERA_LAUNCH_CMD:-ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=false enable_infra1:=false enable_infra2:=false rgb_camera.color_profile:=640x480x30}"
# 180度鱼眼相机：1920x1080@30fps，当前usb_cam版本使用mjpeg2rgb解码格式
USB_CAMERA_LAUNCH_CMD="${USB_CAMERA_LAUNCH_CMD:-ros2 run usb_cam usb_cam_node_exe --ros-args -r image_raw:=/fisheye_camera/image_raw -p video_device:=/dev/video0 -p image_width:=640 -p image_height:=480 -p framerate:=30.0 -p pixel_format:=mjpeg2rgb -p camera_name:=fisheye_camera}"
# USB_CAMERA_LAUNCH_CMD="${USB_CAMERA_LAUNCH_CMD:-ros2 run usb_cam usb_cam_node_exe --ros-args -r image_raw:=/fisheye_camera/image_raw -p video_device:=/dev/video0 -p image_width:=640 -p image_height:=480}"
# AR 检测节点启动命令（180度鱼眼相机专用launch）
# AR_GRID_LAUNCH_CMD="${AR_GRID_LAUNCH_CMD:-ros2 launch ar_grid_detector ar_grid_fisheye.launch.py}"
AR_GRID_LAUNCH_CMD="${AR_GRID_LAUNCH_CMD:-ros2 launch ar_grid_detector ar_grid.launch.py}"
# 向后兼容旧变量：ENABLE_AR_OVERLAY
if [[ -n "${ENABLE_AR_OVERLAY:-}" ]]; then
  ENABLE_AR_DETECTOR="${ENABLE_AR_OVERLAY}"
fi

start_bg() {
  local name="$1"
  local cmd="$2"
  local log_file="${LOG_DIR}/${TS}_${name}.log"
  echo "[START] ${name}: ${cmd}"
  # 在子 shell 中重新 source 所有环境，确保路径正确
  bash -lc "set -e; \
    if command -v conda >/dev/null 2>&1; then conda deactivate >/dev/null 2>&1 || true; fi; \
    set +u; \
    source /opt/ros/humble/setup.bash; \
    source '${MAIN_WORKSPACE}/install/setup.bash'; \
    [[ -f '${AR_WORKSPACE}/install/setup.bash' ]] && source '${AR_WORKSPACE}/install/setup.bash'; \
    set -u; \
    ${cmd}" >"${log_file}" 2>&1 &
  local pid=$!
  PIDS+=("${pid}")
  echo "[PID] ${name}: ${pid}"
  echo "[LOG] ${name}: ${log_file}"
}

cleanup() {
  echo ""
  echo "[INFO] Stopping launched processes..."
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
  wait 2>/dev/null || true
  echo "[INFO] All processes stopped"
}
trap cleanup EXIT INT TERM

echo ""
echo "=========================================="
echo "  AR 检测系统一键启动（Grid Detector）"
echo "=========================================="
echo "[INFO] Log directory: ${LOG_DIR}"
echo "[INFO] Timestamp: ${TS}"
echo "[INFO] Camera backend: ${CAMERA_BACKEND}"
echo ""

# 1. 启动相机驱动（可选，支持 realsense/usb/none）
if [[ "${ENABLE_CAMERA_DRIVER}" == "1" ]]; then
  case "${CAMERA_BACKEND}" in
    realsense)
      echo "[STEP 1/6] Starting RealSense camera..."
      start_bg "camera_realsense" "${REALSENSE_CAMERA_LAUNCH_CMD}"
      sleep 3
      echo "  ✓ RealSense camera started"
      ;;
    usb)
      echo "[STEP 1/6] Starting USB camera (ROS2 package)..."
      if ! ros2 pkg prefix usb_cam >/dev/null 2>&1; then
        echo "[ERROR] ROS2 package 'usb_cam' not found"
        echo "[ERROR] Install with: sudo apt update && sudo apt install ros-humble-usb-cam"
        echo "[ERROR] Then re-source ROS env or reopen terminal"
        exit 1
      fi
      if [[ ! -e "/dev/video0" ]]; then
        echo "[ERROR] USB camera device /dev/video0 not found"
        echo "[ERROR] Check camera connection or override video device in USB_CAMERA_LAUNCH_CMD"
        exit 1
      fi
      start_bg "camera_usb" "${USB_CAMERA_LAUNCH_CMD}"
      sleep 3
      echo "  ✓ USB camera started"
      ;;
    none)
      echo "[STEP 1/6] Camera backend set to 'none' (skip camera startup)"
      ;;
    *)
      echo "[ERROR] Unsupported CAMERA_BACKEND='${CAMERA_BACKEND}', expected: realsense|usb|none"
      exit 1
      ;;
  esac
else
  echo "[STEP 1/6] Camera driver disabled (set ENABLE_CAMERA_DRIVER=1 to enable)"
fi

# 2. 启动 Livox 雷达驱动
echo "[STEP 2/6] Starting Livox MID360 driver..."
start_bg "livox_driver" "ros2 launch livox_ros_driver2 msg_MID360_launch.py"
sleep 3
echo "  ✓ Livox driver started"

# 3. 启动 FAST-LIVO2 里程计 (带或不带重定位配置)
echo "[STEP 3/6] Starting FAST-LIVO2 odometry..."
if [[ "${ENABLE_RELOCALIZATION}" == "1" ]]; then
  # 使用重定位配置启动 FAST-LIVO2
  # 注意：example_teaser_gicp.launch.py 会启动 FAST-LIVO2
  echo "  Note: FAST-LIVO2 will be started by relocalization launch file"
else
  # 纯建图模式
  start_bg "fast_livo" "ros2 launch fast_livo mapping_avia.launch.py use_rviz:=True"
  sleep 5
  echo "  ✓ FAST-LIVO2 started (mapping mode)"
fi

# 4. 启动 TEASER++GICP 重定位 (可选)
if [[ "${ENABLE_RELOCALIZATION}" == "1" ]]; then
  echo "[STEP 4/6] Starting TEASER++GICP relocalization..."
  start_bg "teaser_gicp" "cd '${MAIN_WORKSPACE}' && ros2 launch example_teaser_gicp.launch.py"
  sleep 5
  echo "  ✓ Relocalization module started (includes FAST-LIVO2)"
else
  echo "[STEP 4/6] Relocalization disabled (set ENABLE_RELOCALIZATION=1 to enable)"
fi

# 5. 启动 AR 检测节点（新功能包）
if [[ "${ENABLE_AR_DETECTOR}" == "1" ]]; then
  echo "[STEP 5/6] Starting AR grid detector node..."
  start_bg "ar_grid_detector" "${AR_GRID_LAUNCH_CMD}"
  sleep 2
  echo "  ✓ AR grid detector started"
else
  echo "[STEP 5/6] AR detector disabled (set ENABLE_AR_DETECTOR=1 to enable)"
fi

# 6. 启动位置偏移发布器 (如果需要)
# 这个节点用于发布位置偏移，可根据需要启用
# echo "[STEP 6/6] Starting position offset publisher..."
# start_bg "offset_publisher" "ros2 run fast_livo position_offset_publisher"
# sleep 2
# echo "  ✓ Offset publisher started"
echo "[STEP 6/6] Position offset publisher (optional, currently disabled)"
echo ""

echo "=========================================="
echo "[INFO] All components launched successfully!"
echo "=========================================="
echo ""
echo "System Status:"
echo "  • Camera driver:      $( [[ "${ENABLE_CAMERA_DRIVER}" == "1" ]] && echo "ENABLED (${CAMERA_BACKEND})" || echo "DISABLED" )"
echo "  • Livox driver:       ENABLED"
echo "  • FAST-LIVO2:         ENABLED"
echo "  • Relocalization:     $( [[ "${ENABLE_RELOCALIZATION}" == "1" ]] && echo "ENABLED" || echo "DISABLED" )"
echo "  • AR detector:        $( [[ "${ENABLE_AR_DETECTOR}" == "1" ]] && echo "ENABLED" || echo "DISABLED" )"
echo ""
echo "Checking ROS2 topics..."
sleep 3

# 显示关键话题状态
echo ""
echo "Key topics status:"
for topic in "/livox/lidar" "/livox/imu" "/aft_mapped_to_init" "/fisheye_camera/image_raw" "/ar_grid/image" "/ar_grid/visible_cells"; do
  if ros2 topic list 2>/dev/null | grep -qx "${topic}"; then
    echo "  ✓ ${topic}"
  else
    echo "  ✗ ${topic} (not available)"
  fi
done

echo ""
echo "=========================================="
echo "  System is running!"
echo "=========================================="
echo ""
echo "Tips:"
echo "  • View logs in: ${LOG_DIR}/"
echo "  • View topics: ros2 topic list"
echo "  • View transforms: ros2 run tf2_tools view_frames"
echo "  • Monitor odometry: ros2 topic echo /aft_mapped_to_init"
if [[ "${ENABLE_AR_DETECTOR}" == "1" ]]; then
  echo "  • View AR image: ros2 run rqt_image_view rqt_image_view /ar_grid/image"
  echo "  • View visible cells: ros2 topic echo /ar_grid/visible_cells"
fi
echo ""
echo "Environment variable examples:"
echo "  • CAMERA_BACKEND=realsense ./start_all_with_offset.sh"
echo "  • CAMERA_BACKEND=usb USB_CAMERA_LAUNCH_CMD='ros2 launch usb_cam camera.launch.py' ./start_all_with_offset.sh"
echo "  • ENABLE_CAMERA_DRIVER=0 ENABLE_AR_DETECTOR=1 ./start_all_with_offset.sh"
echo ""
echo "Press Ctrl+C to stop all processes..."
echo ""

# 保持脚本运行，监控子进程
while true; do
  sleep 5
  # 检查是否有进程意外退出
  for i in "${!PIDS[@]}"; do
    if ! kill -0 "${PIDS[$i]}" 2>/dev/null; then
      echo "[WARN] Process ${PIDS[$i]} has died unexpectedly"
    fi
  done
done
