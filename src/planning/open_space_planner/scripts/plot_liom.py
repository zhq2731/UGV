#!/usr/bin/env python3
"""Show LIOM before/after data without publishing any ROS/control topics."""
import argparse
import json
import os
from pathlib import Path

import matplotlib
import numpy as np


def main():
    parser = argparse.ArgumentParser(description="LIOM 优化前后轨迹及控制量对比")
    parser.add_argument("directory", type=Path)
    parser.add_argument("--no-show", action="store_true", help="仅保存图片，不打开窗口")
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
    info = json.loads((directory / "summary.json").read_text())
    paths = [np.genfromtxt(directory / f"{name}.csv", delimiter=",", names=True)
             for name in ("before", "after")]
    fig, axes = plt.subplots(3, 2, figsize=(14, 10), constrained_layout=True)
    direction = "倒车" if info["reverse"] else "前进"
    fig.suptitle(f"LIOM 单段优化对比 · {direction} · 无障碍合成示例", fontsize=17)
    for data, label, color in zip(paths, ("优化前", "优化后"), ("#d97706", "#1678b5")):
        t = data["time_s"]
        axes[0, 0].plot(data["x_m"], data["y_m"], ".-", color=color, label=label, markersize=3)
        for ax, field in ((axes[0, 1], "velocity_mps"), (axes[1, 0], "acceleration_mps2"),
                          (axes[1, 1], "steer_deg")):
            ax.plot(t, data[field], ".-", color=color, label=label, markersize=3)
        # 同一差分口径：区间平均前轮角速率，不把它冒充优化器内部节点 omega。
        rate = np.diff(data["steer_deg"]) / np.diff(t)
        midpoint = (t[1:] + t[:-1]) / 2
        axes[2, 0].plot(midpoint, rate, ".-", color=color, label=label, markersize=3)
        np.savetxt(directory / ("before_rate.csv" if label == "优化前" else "after_rate.csv"),
                   np.column_stack((midpoint, rate)), delimiter=",",
                   header="interval_mid_time_s,steer_rate_deg_s", comments="")

    axes[0, 0].set(xlabel="x / m", ylabel="y / m", title="轨迹（等比例坐标）")
    axes[0, 0].set_aspect("equal", adjustable="datalim")
    start, end = paths[0][0], paths[0][-1]
    axes[0, 0].scatter([start["x_m"], end["x_m"]], [start["y_m"], end["y_m"]], color="black", s=25)
    axes[0, 0].annotate("起点", (start["x_m"], start["y_m"]), xytext=(5, 8), textcoords="offset points")
    axes[0, 0].annotate("终点", (end["x_m"], end["y_m"]), xytext=(5, 8), textcoords="offset points")
    panels = [(axes[0, 1], "有符号速度", "m/s"), (axes[1, 0], "规划加速度", "m/s²"),
              (axes[1, 1], "规划前轮转角（非方向盘角）", "deg"),
              (axes[2, 0], "前轮转角变化率（区间差分）", "deg/s")]
    for ax, title, unit in panels:
        ax.set(xlabel="各自轨迹时间 / s", ylabel=unit, title=title)
    sign = -1 if info["reverse"] else 1
    limits = [(axes[0, 1], [0, sign*info["max_speed"]]),
              (axes[1, 0], [info["a_min"], info["a_max"]]),
              (axes[1, 1], [-info["steer_limit_deg"], info["steer_limit_deg"]]),
              (axes[2, 0], [-info["rate_limit_deg_s"], info["rate_limit_deg_s"]])]
    for ax, values in limits:
        for value in values:
            ax.axhline(value, color="gray", linestyle="--", linewidth=0.8)
    for ax in axes.flat:
        if ax is not axes[2, 1]:
            ax.grid(alpha=0.2)
            ax.legend(loc="best")
    axes[2, 1].axis("off")
    before, after = paths
    description = (
        f"总时间：{before['time_s'][-1]:.2f} → {after['time_s'][-1]:.2f} s\n"
        f"路径长度：{before['s_m'][-1]:.2f} → {after['s_m'][-1]:.2f} m\n"
        f"求解耗时：{info['solve_ms']:.1f} ms（单次样本）\n"
        f"最大运动学残差：{info['max_residual']:.2e}\n"
        f"惩罚外循环：{info['penalty_iterations']} 次\n\n"
        "橙色：原速度规划生成的初值\n蓝色：LIOM 优化输出\n灰虚线：配置物理上限\n\n"
        "曲率分段变化的合成路径，不是实际搜索结果。\n"
        "两条曲线使用各自时间轴，不强制拉伸对齐。\n"
        "转角变化率是相邻点差分，非内部节点 omega。\n"
        "尚无障碍约束；未接入控制，不表示可以上车。")
    axes[2, 1].text(0.02, 0.96, description, va="top", fontsize=10, linespacing=1.5)
    figure = directory / "liom_comparison.png"
    fig.savefig(figure, dpi=150)
    print(f"对比图：{figure}\n数据目录：{directory}")
    if not headless:
        plt.show()
    plt.close(fig)


if __name__ == "__main__":
    main()
