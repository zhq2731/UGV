#!/usr/bin/env bash
# 独立构建与展示，不覆盖主工程 build/devel，不启动 ROS 或发布控制指令。
set -e
liom_package_dir="$(cd "$(dirname "$0")/.." && pwd)"
liom_workspace_dir="$(cd "$liom_package_dir/../../.." && pwd)"
source /opt/ros/noetic/setup.bash
source "$liom_workspace_dir/devel/setup.bash"
liom_demo_dir="$(mktemp -d /tmp/ugv-liom-view-XXXXXX)"
liom_direction=forward
liom_plot_args=()
for arg in "$@"; do
  case "$arg" in
    --reverse) liom_direction=reverse ;;
    --no-show) liom_plot_args+=(--no-show) ;;
    *) echo "用法：bash $0 [--reverse] [--no-show]" >&2; exit 2 ;;
  esac
done
echo "独立构建与结果目录：$liom_demo_dir"
if ! cmake -S "$liom_package_dir" -B "$liom_demo_dir/build" \
    -DOPEN_SPACE_BUILD_LIOM=ON -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release \
    > "$liom_demo_dir/build.log" 2>&1 || \
   ! cmake --build "$liom_demo_dir/build" --target liom_visualization_demo -j2 \
    >> "$liom_demo_dir/build.log" 2>&1; then
  tail -n 40 "$liom_demo_dir/build.log"
  exit 1
fi
export LD_LIBRARY_PATH="$liom_demo_dir/build/devel/lib:$LD_LIBRARY_PATH"
"$liom_demo_dir/build/devel/lib/open_space_planner/liom_visualization_demo" \
  "$liom_package_dir/param/openspace.yaml" "$liom_demo_dir" "$liom_direction"
/usr/bin/python3 "$liom_package_dir/scripts/plot_liom.py" "$liom_demo_dir" "${liom_plot_args[@]}"
