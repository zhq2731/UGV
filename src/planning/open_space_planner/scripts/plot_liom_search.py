#!/usr/bin/env python3
"""Plot actual Hybrid A* seeds, LIOM output and obstacle-clearance diagnostics."""
import argparse
import csv
import json
import os
from pathlib import Path

import matplotlib
import numpy as np

TITLES = {
    "forward_straight": "前进直行", "reverse_curve": "倒车曲线",
    "parking_cusp": "三段换挡", "obstacle_detour": "矩形障碍绕行",
    "narrow_straight": "窄通道直行", "search_tree_turn": "搜索树展开后的掉头",
}
FIELDS = ("x_m", "y_m", "heading_rad", "s_m", "kappa", "velocity_mps", "acceleration_mps2", "time_s")


def read_csv(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def paths_for(rows, name, stage, pose):
    rows = [r for r in rows if r["scene"] == name and r["stage"] == stage]
    paths = []
    c, s = np.cos(pose[2]), np.sin(pose[2])
    for segment in sorted({int(r["segment"]) for r in rows}):
        part = [r for r in rows if int(r["segment"]) == segment]
        p = np.array([[float(r[f]) for f in FIELDS] for r in part])
        p[:, :2] = (p[:, :2] - pose[:2]) @ np.array([[c, -s], [s, c]])
        p[:, 2] = np.arctan2(np.sin(p[:, 2] - pose[2]), np.cos(p[:, 2] - pose[2]))
        paths.append((int(part[0]["forward"]) != 0, p))
    return paths


def body(p, vehicle, margin=0):
    rear, front = -vehicle["rear_overhang"] - margin, vehicle["length"] - vehicle["rear_overhang"] + margin
    half = vehicle["width"] / 2 + margin
    c, s = np.cos(p[2]), np.sin(p[2])
    return np.array([[rear, half], [front, half], [front, -half], [rear, -half]]) @ np.array([[c, s], [-s, c]]) + p[:2]


def footprint_collides(p, vehicle, obstacles, margin):
    if len(obstacles) == 0:
        return False
    c, s = np.cos(p[2]), np.sin(p[2])
    hx, hy = vehicle["length"] / 2 + margin, vehicle["width"] / 2 + margin
    center = p[:2] + (vehicle["length"] / 2 - vehicle["rear_overhang"]) * np.array([c, s])
    half = obstacles[:, 2] / 2
    delta = obstacles[:, :2] + half[:, None] - center
    dx, dy = delta[:, 0], delta[:, 1]
    # 与 C++ 相同的整格/旋转矩形分离轴口径，绘图诊断不修改规划约束。
    overlap = ((np.abs(dx) <= hx * abs(c) + hy * abs(s) + half) &
               (np.abs(dy) <= hx * abs(s) + hy * abs(c) + half) &
               (np.abs(c * dx + s * dy) <= hx + half * (abs(c) + abs(s))) &
               (np.abs(-s * dx + c * dy) <= hy + half * (abs(c) + abs(s))))
    return bool(np.any(overlap))


def first_between_collision(points, vehicle, obstacles, resolution, margin):
    radius = np.hypot(max(vehicle["rear_overhang"], vehicle["length"] - vehicle["rear_overhang"]) + margin,
                      vehicle["width"] / 2 + margin)
    for i, (a, b) in enumerate(zip(points[:-1], points[1:])):
        angle = np.arctan2(np.sin(b[2] - a[2]), np.cos(b[2] - a[2]))
        steps = max(1, int(np.ceil((np.linalg.norm(b[:2] - a[:2]) + radius * abs(angle)) / (0.25 * resolution))))
        for k in range(steps + 1):
            t = k / steps
            p = np.array([*(a[:2] + t * (b[:2] - a[:2])), a[2] + t * angle])
            if footprint_collides(p, vehicle, obstacles, margin):
                return dict(interval=i, ratio=t, pose=p.tolist(), a=a[:3].tolist(), b=b[:3].tolist())
    return None


def draw_map(ax, obstacles):
    from matplotlib.collections import PatchCollection
    from matplotlib.patches import Rectangle
    if len(obstacles):
        ax.add_collection(PatchCollection([Rectangle((x, y), size, size) for x, y, size in obstacles],
                                         facecolor="#475569", edgecolor="none", zorder=1))
        ax.plot([], [], "s", color="#475569", markersize=6, label="占据栅格")
    ax.set(xlabel="地图局部 x / m", ylabel="地图局部 y / m")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(alpha=0.15)


def draw_body(ax, p, vehicle, margin, color, label=None, alpha=0.08):
    from matplotlib.patches import Polygon
    ax.add_patch(Polygon(body(p, vehicle, margin), closed=True, facecolor=color, alpha=alpha, zorder=2))
    corners = body(p, vehicle, margin)
    ax.plot(*np.vstack((corners, corners[0])).T, color=color, linewidth=1.2, label=label, zorder=3)


def draw_paths(ax, raw, stages, obstacles, vehicle):
    draw_map(ax, obstacles)
    ax.plot(raw[:, 0], raw[:, 1], ".-", color="#94a3b8", linewidth=1, markersize=2, label="原始搜索点", zorder=2)
    for paths, label, color in stages:
        for j, (forward, p) in enumerate(paths):
            ax.plot(p[:, 0], p[:, 1], "-" if forward else ":", color=color, linewidth=1.7,
                    label=label if j == 0 else None, zorder=4)
    seeds = stages[0][0]
    for j, (forward, p) in enumerate(seeds):
        ax.scatter(*p[0, :2], color="black", s=15, zorder=5)
        ax.annotate(f"{j + 1}:{'D' if forward else 'R'}", p[len(p) // 2, :2], xytext=(4, 6), textcoords="offset points", fontsize=8)
    draw_body(ax, seeds[0][1][0], vehicle, 0, "#16a34a", "起点车身")
    draw_body(ax, seeds[-1][1][-1], vehicle, 0, "#64748b", "终点车身")
    xy = np.vstack([raw[:, :2], *(p[:, :2] for paths, _, _ in stages for _, p in paths)])
    if len(obstacles):
        xy = np.vstack((xy, obstacles[:, :2], obstacles[:, :2] + obstacles[:, 2, None]))
    ax.set_xlim(xy[:, 0].min() - 4.5, xy[:, 0].max() + 4.5)
    ax.set_ylim(xy[:, 1].min() - 4.5, xy[:, 1].max() + 4.5)


def scene_figure(plt, scene, raw, stages, obstacles, vehicle):
    fig, axes = plt.subplots(3, 2, figsize=(14, 10), constrained_layout=True)
    fig.suptitle(f"真实 Hybrid A* → LIOM · {TITLES[scene['name']]}", fontsize=16)
    draw_paths(axes[0, 0], raw, stages, obstacles, vehicle)
    axes[0, 0].set_title("轨迹与占据栅格；D 前进 / R 倒车")
    for paths, label, color in stages:
        offset = 0.0
        for j, (forward, p) in enumerate(paths):
            time = p[:, 7] + offset
            steer = np.rad2deg(np.arctan(vehicle["wheel_base"] * (1 if forward else -1) * p[:, 4]))
            for ax, values in ((axes[0, 1], p[:, 5]), (axes[1, 0], p[:, 6]), (axes[1, 1], steer)):
                ax.plot(time, values, color=color, linewidth=1.5, label=label if j == 0 else None)
            rate = np.diff(steer) / np.diff(p[:, 7])
            axes[2, 0].plot((time[:-1] + time[1:]) / 2, rate, color=color, label=label if j == 0 else None)
            bounds = [-vehicle["max_deceleration"], vehicle["max_acceleration"]] if forward else [-vehicle["max_acceleration"], vehicle["max_deceleration"]]
            for bound in bounds:
                axes[1, 0].plot([time[0], time[-1]], [bound, bound], color="gray", linestyle="--", linewidth=0.7)
            if j:
                for ax in (axes[0, 1], axes[1, 0], axes[1, 1], axes[2, 0]):
                    ax.axvline(offset, color=color, linestyle=":", linewidth=0.7, alpha=0.45)
            offset += p[-1, 7]
    panels = ((axes[0, 1], "有符号速度", "m/s", [-vehicle["reverse_speed"], vehicle["forward_speed"]]),
              (axes[1, 0], "规划加速度（灰线按档位显示限值）", "m/s²", []),
              (axes[1, 1], "前轮转角", "deg", [-vehicle["steering_angle_deg"], vehicle["steering_angle_deg"]]),
              (axes[2, 0], "前轮转角变化率（段内差分）", "deg/s", np.rad2deg([-vehicle["steering_rate"], vehicle["steering_rate"]])))
    for ax, title, unit, limits in panels:
        ax.set(xlabel="各自轨迹的分段累计时间 / s", ylabel=unit, title=title)
        for value in limits:
            ax.axhline(value, color="gray", linestyle="--", linewidth=0.7)
        ax.grid(alpha=0.2)
        ax.legend(fontsize=9)
    axes[0, 0].legend(fontsize=8)
    axes[2, 1].axis("off")
    status = "优化成功，输出复核通过" if scene["optimized"] else "优化拒绝：没有优化输出"
    description = [status, f"分段数：{scene['segments']}；搜索树状态：{scene['visited']}",
                   f"搜索 / 优化耗时：{scene['search_ms']:.1f} / {scene['optimize_ms']:.1f} ms", ""]
    for paths, label, _ in stages:
        description.append(f"{label}：{sum(p[-1, 7] for _, p in paths):.2f} s / {sum(p[-1, 3] for _, p in paths):.2f} m")
    description += ["", "橙色：原速度规划生成的搜索初值", "蓝色：被接受的优化输出", "竖虚线：各自轨迹的换挡段边界",
                    "各段独立绘制，转角速率不跨换挡点差分。", "累计时间未计入停车调轮及换挡等待时间。",
                    f"车身安全余量：{vehicle['margin']:.2f} m", "加速度限制随前进/倒车方向分别应用。",
                    "本图仅展示独立接口结果，未接入车辆控制。"]
    axes[2, 1].text(0, 1, "\n".join(description), va="top", fontsize=10, linespacing=1.55)
    return fig


def safety_figure(plt, scene, raw, stages, obstacles, vehicle, model):
    from matplotlib.patches import Circle, Rectangle
    fig, axes = plt.subplots(2, 2, figsize=(14, 11), constrained_layout=True)
    fig.suptitle("绕障安全几何：旧圆包络与初值点间余量", fontsize=17)
    draw_paths(axes[0, 0], raw, stages, obstacles, vehicle)
    axes[0, 0].set_title("五段搜索初值与优化输出" if scene["optimized"] else "五段搜索初值；整条优化未通过")
    seeds = stages[0][0]
    start = seeds[0][1][0, :3]
    ax = axes[0, 1]
    draw_map(ax, obstacles)
    draw_body(ax, start, vehicle, 0, "#16a34a", "实际车身")
    draw_body(ax, start, vehicle, vehicle["margin"], "#d97706", "车身 + 安全余量")
    if model == "oriented_body_rectangle":
        draw_body(ax, start, vehicle, vehicle["margin"] + 0.25 * scene["resolution"],
                  "#1678b5", "当前矩形初始包络（含数值缓冲）")
    length = vehicle["length"] + 2 * vehicle["margin"]
    radius = 0.5 * np.hypot(length / 4, vehicle["width"] + 2 * vehicle["margin"])
    box_radius = radius + 0.5 * scene["resolution"]
    boxes = []
    for j in range(4):
        offset = -vehicle["rear_overhang"] - vehicle["margin"] + (j + 0.5) * length / 4
        center = start[:2] + offset * np.array([np.cos(start[2]), np.sin(start[2])])
        collision = bool(np.any((obstacles[:, 0] <= center[0] + box_radius) &
                               (obstacles[:, 0] + obstacles[:, 2] >= center[0] - box_radius) &
                               (obstacles[:, 1] <= center[1] + box_radius) &
                               (obstacles[:, 1] + obstacles[:, 2] >= center[1] - box_radius)))
        color = "#dc2626" if collision else "#0284c7"
        ax.add_patch(Circle(center, radius, fill=False, edgecolor=color, linewidth=1))
        ax.add_patch(Rectangle(center - box_radius, 2 * box_radius, 2 * box_radius,
                               fill=False, edgecolor=color, linestyle="--", linewidth=1.3))
        boxes.append(dict(disc=j, collides=collision, center=center.tolist()))
    ax.set(xlim=(-2.7, 7.5), ylim=(-3, 3), title="第一段起点：旧四圆/方框侵入障碍，矩形车身包络安全")
    ax.legend(fontsize=8)

    second = seeds[1][1]
    event = first_between_collision(second, vehicle, obstacles, scene["resolution"], vehicle["margin"])
    nodes_free = all(not footprint_collides(p[:3], vehicle, obstacles, vehicle["margin"]) for p in second)
    nominal_free = first_between_collision(second, vehicle, obstacles, scene["resolution"], 0) is None
    ax = axes[1, 0]
    draw_map(ax, obstacles)
    if event:
        p, a, b = (np.array(event[key]) for key in ("pose", "a", "b"))
        angle = np.arctan2(np.sin(b[2] - a[2]), np.cos(b[2] - a[2]))
        corners = [body(np.array([*(a[:2] + t * (b[:2] - a[:2])), a[2] + t * angle]), vehicle, vehicle["margin"])[1]
                   for t in np.linspace(0, 1, 61)]
        ax.plot(*np.array(corners).T, color="#0284c7", label="外扩车身左前角运动")
        ax.scatter(*np.array([corners[0], corners[-1]]).T, color="#16a34a", s=35, label="原节点的车身角点", zorder=5)
        corner = body(p, vehicle, vehicle["margin"])[1]
        draw_body(ax, p, vehicle, vehicle["margin"], "#dc2626", "中间位姿安全包络")
        ax.scatter(*corner, color="#dc2626", s=40, zorder=6, label="中间检查点")
        ax.set_xlim(corner[0] - 0.2, corner[0] + 0.2)
        ax.set_ylim(corner[1] - 0.2, corner[1] + 0.2)
        axes[0, 0].scatter(*p[:2], color="#dc2626", s=30, zorder=6)
    if scene["optimized"]:
        optimized = stages[1][0][1][1]
        trace = []
        for a, b in zip(optimized[:-1], optimized[1:]):
            angle = np.arctan2(np.sin(b[2] - a[2]), np.cos(b[2] - a[2]))
            for t in np.linspace(0, 1, 9):
                p = np.array([*(a[:2] + t * (b[:2] - a[:2])), a[2] + t * angle])
                trace.append(body(p, vehicle, vehicle["margin"])[1])
        ax.plot(*np.array(trace).T, color="#1678b5", linestyle="--", label="优化后外扩车身左前角运动")
    ax.set_title("第二段局部放大：初值点间相交；蓝虚线为优化后的角点运动")
    ax.legend(fontsize=8)
    axes[1, 1].axis("off")
    status = "全部五段优化及输出复核通过。" if scene["optimized"] else f"失败段：{scene['failed_segment'] + 1}；没有优化输出。"
    message = [f"实际返回：{status}", f"走廊模型：{model}", "",
               f"第一段：旧四圆方框碰撞 = {any(box['collides'] for box in boxes)}", "第二段搜索初值：",
               f"  原节点 + {vehicle['margin']:.2f} m 余量全部通过 = {nodes_free}",
               f"  点间插值 + 同样余量发现相交 = {event is not None}", f"  零余量实际车身点间检查通过 = {nominal_free}"]
    if event:
        message += [f"  首次相交区间：节点 {event['interval'] + 1} → {event['interval'] + 2}",
                    f"  区间位置：{event['ratio']:.3f}（此处未加扫掠附加外扩）"]
    if scene["optimized"]:
        message += [f"优化输出完整车身/扫掠复核通过 = {bool(scene['output_valid'])}"]
    message += ["", "右上四圆/方框仅用于解释旧模型问题；", "当前矩形初始包络不是扩展后的完整走廊。",
                "左下红色表示初值不满足 0.4 m 余量。",
                "绘图复查口径与 C++ 诊断一致；", "采用位姿插值，并非实际车辆跟踪仿真。"]
    axes[1, 1].text(0, 1, "\n".join(message), va="top", fontsize=11, linespacing=1.6)
    return fig, dict(corridor_model=model, optimized=bool(scene["optimized"]), legacy_circle_boxes=boxes, second_nodes_free=nodes_free,
                     second_first_between_collision=event, second_nominal_free=nominal_free)


def main():
    parser = argparse.ArgumentParser(description="真实搜索与 LIOM 多段优化可视化")
    parser.add_argument("directory", type=Path)
    parser.add_argument("--scene", choices=TITLES, default="obstacle_detour", help="选择打开的场景，所有场景都会保存图片")
    parser.add_argument("--no-show", action="store_true", help="只保存图片")
    args = parser.parse_args()
    headless = args.no_show or not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    if headless:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    fonts = {font.name for font in font_manager.fontManager.ttflist}
    for name in ("Noto Sans CJK JP", "Noto Sans CJK SC", "WenQuanYi Micro Hei"):
        if name in fonts:
            plt.rcParams["font.family"] = name
            break
    plt.rcParams["axes.unicode_minus"] = False
    directory = args.directory
    info = json.loads((directory / "search_metadata.json").read_text())
    rows = read_csv(directory / "search_paths.csv")
    raw_rows = read_csv(directory / "search_raw.csv")
    obstacle_rows = read_csv(directory / "search_obstacles.csv")
    overview, overview_axes = plt.subplots(2, 3, figsize=(17, 10), constrained_layout=True)
    accepted = sum(bool(scene["optimized"]) for scene in info["scenes"])
    overview.suptitle(f"真实 Hybrid A* 搜索输入验证：{accepted} 例优化成功，{len(info['scenes']) - accepted} 例拒绝", fontsize=17)
    for scene, ax in zip(info["scenes"], overview_axes.flat):
        name = scene["name"]
        raw = np.array([[float(r[f]) for f in ("x_m", "y_m", "heading_rad")] for r in raw_rows if r["scene"] == name])
        obstacles = np.array([[float(r[f]) for f in ("x_m", "y_m", "size_m")] for r in obstacle_rows if r["scene"] == name]).reshape(-1, 3)
        stages = [(paths_for(rows, name, "seed", info["map_pose"]), "搜索初值", "#d97706")]
        if scene["optimized"]:
            stages.append((paths_for(rows, name, "optimized", info["map_pose"]), "优化输出", "#1678b5"))
        draw_paths(ax, raw, stages, obstacles, info["vehicle"])
        ax.set_title(f"{TITLES[name]} · {scene['segments']} 段 · {scene['optimize_ms']:.0f} ms", fontsize=11)
        ax.legend(fontsize=7)
        fig = scene_figure(plt, scene, raw, stages, obstacles, info["vehicle"])
        fig.savefig(directory / f"{name}_comparison.png", dpi=150)
        if headless or name != args.scene:
            plt.close(fig)
        if name == "obstacle_detour":
            failure, diagnostics = safety_figure(plt, scene, raw, stages, obstacles, info["vehicle"],
                                               info.get("corridor_model", "four_discs"))
            failure.savefig(directory / "obstacle_detour_safety.png", dpi=160)
            (directory / "search_diagnostics.json").write_text(json.dumps(diagnostics, indent=2))
            if headless or name != args.scene:
                plt.close(failure)
    overview.savefig(directory / "search_overview.png", dpi=150)
    print(f"总览：{directory / 'search_overview.png'}\n安全几何：{directory / 'obstacle_detour_safety.png'}\n结果目录：{directory}")
    if not headless:
        plt.show()
    plt.close("all")


if __name__ == "__main__":
    main()
