#!/usr/bin/env bash
# 使用真实搜索检查程序导出地图和多段轨迹，独立构建后绘图。
set -e -o pipefail
liom_package_dir="$(cd "$(dirname "$0")/.." && pwd)"
liom_workspace_dir="$(cd "$liom_package_dir/../../.." && pwd)"
source /opt/ros/noetic/setup.bash
source "$liom_workspace_dir/devel/setup.bash"
liom_view_dir="$(mktemp -d /tmp/ugv-liom-search-view-XXXXXX)"
echo "独立构建与结果目录：$liom_view_dir"
if ! cmake -S "$liom_package_dir" -B "$liom_view_dir/build" \
    -DOPEN_SPACE_BUILD_LIOM=ON -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release \
    > "$liom_view_dir/build.log" 2>&1 || \
   ! cmake --build "$liom_view_dir/build" --target liom_search_check -j2 \
    >> "$liom_view_dir/build.log" 2>&1; then
  tail -n 40 "$liom_view_dir/build.log"
  exit 1
fi
export LD_LIBRARY_PATH="$liom_view_dir/build/devel/lib:$LD_LIBRARY_PATH"
"$liom_view_dir/build/devel/lib/open_space_planner/liom_search_check" \
  "$liom_package_dir/param/openspace.yaml" "$liom_view_dir" | tee "$liom_view_dir/search-check.log"
/usr/bin/python3 "$liom_package_dir/scripts/plot_liom_search.py" "$liom_view_dir" "$@"
