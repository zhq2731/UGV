#!/usr/bin/env bash
set -Eeuo pipefail

# 一键启动比赛 Lite 界面与本工程开放空间泊车算法。
# Lite 后端仍由界面的 Start 按钮启动，车辆仍由 Add Vehicle 添加，
# 这样界面能够正确维护后端运行状态，且不会重复启动仿真节点。

readonly UGV_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly DEFAULT_LITE_ROOT="/home/zhq/SIM_CODE_LITE_ubuntu20.04_ros1_20260730/SIM_CODE_LITE_ubuntu20.04_ros1_20260729"
readonly LITE_ROOT="${LITE_SIM_ROOT:-${DEFAULT_LITE_ROOT}}"
readonly VEHICLE_ID="${LITE_VEHICLE_ID:-0}"

# rg 使用用户级安装路径；当前终端无需重新登录即可运行本脚本。
export PATH="/home/zhq/.local/bin:${PATH}"

ROSCORE_PID=""
ALGORITHM_PID=""
GUI_PID=""
STARTED_ROSCORE=false
LOG_DIR=""

die() {
    echo "[启动失败] $*" >&2
    exit 1
}

process_is_running() {
    [[ -n "$1" ]] && kill -0 "$1" 2>/dev/null
}

ros_master_is_ready() {
    rosparam get /rosversion >/dev/null 2>&1
}

wait_for_ros_master() {
    local attempt
    for attempt in {1..50}; do
        ros_master_is_ready && return 0
        process_is_running "$ROSCORE_PID" || return 1
        sleep 0.1
    done
    return 1
}

wait_for_ros_node() {
    local node_name="$1"
    local attempt
    for attempt in {1..100}; do
        rosnode ping -c 1 "$node_name" >/dev/null 2>&1 && return 0
        process_is_running "$ALGORITHM_PID" || return 1
        sleep 0.1
    done
    return 1
}

wait_for_gui_node() {
    local attempt
    for attempt in {1..100}; do
        rosnode list 2>/dev/null | rg -q '^/zheda_qt_frontend_' && return 0
        process_is_running "$GUI_PID" || return 1
        sleep 0.1
    done
    return 1
}

cleanup() {
    local exit_code=$?
    trap - EXIT INT TERM HUP

    echo
    echo "[退出] 正在停止本次启动的泊车仿真进程……"

    # 后端由 GUI 创建，不是本脚本的直接子进程；通过唯一 ROS 节点名精确关闭。
    if ros_master_is_ready &&
       rosnode list 2>/dev/null | rg -Fqx "/zheda_lite_backend"; then
        rosnode kill /zheda_lite_backend >/dev/null 2>&1 || true
    fi

    if process_is_running "$GUI_PID"; then
        kill -TERM "$GUI_PID" 2>/dev/null || true
    fi
    if process_is_running "$ALGORITHM_PID"; then
        kill -INT "$ALGORITHM_PID" 2>/dev/null || true
    fi
    if [[ "$STARTED_ROSCORE" == true ]] && process_is_running "$ROSCORE_PID"; then
        kill -INT "$ROSCORE_PID" 2>/dev/null || true
    fi

    [[ -n "$GUI_PID" ]] && wait "$GUI_PID" 2>/dev/null || true
    [[ -n "$ALGORITHM_PID" ]] && wait "$ALGORITHM_PID" 2>/dev/null || true
    [[ -n "$ROSCORE_PID" ]] && wait "$ROSCORE_PID" 2>/dev/null || true

    [[ -n "$LOG_DIR" ]] && echo "[退出] 本次日志目录：${LOG_DIR}"
    exit "$exit_code"
}

trap cleanup EXIT INT TERM HUP

[[ -f /opt/ros/noetic/setup.bash ]] ||
    die "未找到 /opt/ros/noetic/setup.bash，请先安装 ROS Noetic。"
[[ -f "${UGV_ROOT}/devel/setup.bash" ]] ||
    die "未找到 ${UGV_ROOT}/devel/setup.bash，请先在 UGV 根目录完成编译。"
[[ -f "${LITE_ROOT}/run_lite_gui.sh" ]] ||
    die "未找到比赛界面：${LITE_ROOT}/run_lite_gui.sh。可通过 LITE_SIM_ROOT 指定实际目录。"
[[ -n "${DISPLAY:-}" ]] ||
    die "当前没有可用的 DISPLAY，无法启动比赛图形界面。"

# ROS 环境脚本可能读取未定义变量，加载期间临时关闭 nounset。
set +u
source /opt/ros/noetic/setup.bash
source "${UGV_ROOT}/devel/setup.bash"
set -u

command -v rg >/dev/null 2>&1 || die "未安装 rg，无法执行节点启动检查。"
command -v roslaunch >/dev/null 2>&1 || die "ROS 环境中未找到 roslaunch。"

LOG_DIR="$(mktemp -d /tmp/ugv_lite_parking.XXXXXX)"
mkdir -p "${LOG_DIR}/ros"
export ROS_LOG_DIR="${LOG_DIR}/ros"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"

if ros_master_is_ready; then
    echo "[1/3] 复用现有 ROS Master：${ROS_MASTER_URI}"
else
    echo "[1/3] 启动 ROS Master：${ROS_MASTER_URI}"
    roscore >"${LOG_DIR}/roscore.log" 2>&1 &
    ROSCORE_PID=$!
    STARTED_ROSCORE=true
    wait_for_ros_master ||
        die "ROS Master 启动失败，详见 ${LOG_DIR}/roscore.log"
fi

# 避免同名节点互相顶替，防止旧仿真状态混入本次测试。
readonly ACTIVE_NODES="$(rosnode list 2>/dev/null || true)"
for node_name in /planner /trajectory_follower /lite_parking_bridge /zheda_lite_backend; do
    if rg -Fqx "$node_name" <<<"$ACTIVE_NODES"; then
        die "检测到遗留节点 ${node_name}，请先结束旧仿真后重新运行。"
    fi
done
if rg -q '^/zheda_qt_frontend_' <<<"$ACTIVE_NODES"; then
    die "检测到已运行的比赛界面，请先关闭旧界面后重新运行。"
fi

echo "[2/3] 启动开放空间规划与泊车控制，车辆 ID=${VEHICLE_ID}"
roslaunch launch_node lite-open-space-parking-planning.launch \
    vehicle_id:="${VEHICLE_ID}" &
ALGORITHM_PID=$!

for node_name in /lite_parking_bridge /planner /trajectory_follower; do
    wait_for_ros_node "$node_name" ||
        die "算法节点 ${node_name} 未正常启动，请检查上方 roslaunch 输出。"
done

echo "[3/3] 启动比赛 Lite 图形界面"
(
    cd -- "$LITE_ROOT"
    exec bash ./run_lite_gui.sh
) >"${LOG_DIR}/lite_gui.log" 2>&1 &
GUI_PID=$!

wait_for_gui_node ||
    die "比赛界面未正常连接 ROS，详见 ${LOG_DIR}/lite_gui.log"

echo
echo "全部入口已启动。请在比赛界面中依次操作："
echo "  1. 保持 ROS control 勾选，点击 Start；"
echo "  2. 点击 Add Vehicle 添加 0 号车；"
echo "  3. 使用 Set goal pose 设置泊车目标。"
echo "关闭比赛界面或在本终端按 Ctrl+C，将停止本次启动的全部进程。"
echo "日志目录：${LOG_DIR}"

# 任一主进程退出都结束本次会话，避免留下半套仿真环境。
while process_is_running "$ALGORITHM_PID" && process_is_running "$GUI_PID"; do
    sleep 1
done

if ! process_is_running "$ALGORITHM_PID"; then
    echo "[异常] 泊车算法进程已退出。" >&2
elif ! process_is_running "$GUI_PID"; then
    echo "[提示] 比赛界面已关闭。"
fi
