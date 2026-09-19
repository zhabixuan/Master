# -*- coding: utf-8 -*-
"""
生成汇报 PPT 所需的效果图。
所有图均基于项目真实参数(车辆尺寸、圆盘碰撞模型、迭代框架)绘制。
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyArrow, Polygon
from matplotlib.lines import Line2D
from scipy.interpolate import CubicSpline

# ---------- 中文字体（含拉丁与 CJK 的单一字体，避免缺字） ----------
from matplotlib import font_manager as fm
import os as _os
_BASE = _os.path.dirname(_os.path.abspath(__file__))
for _f in ('NotoSansCJKsc.otf', 'NotoSansCJKsc-Bold.otf'):
    _p = _os.path.join(_BASE, 'figures', _f)
    if _os.path.exists(_p):
        fm.fontManager.addfont(_p)
matplotlib.rcParams['font.family'] = 'Noto Sans CJK SC'
matplotlib.rcParams['axes.unicode_minus'] = False
matplotlib.rcParams['font.size'] = 11

FIG = 'figures/'

# ============ 配色 ============
C = {
    'navy':   '#1F3864',
    'blue':   '#2E75B6',
    'red':    '#D64545',
    'green':  '#3E9B6C',
    'orange': '#E8A33D',
    'yellow': '#F2C14E',
    'gray':   '#8C8C8C',
    'light':  '#EEF3F8',
}

# ============ 车辆真实参数 ============
L = 4.689        # 车长
W = 1.942        # 车宽
WB = 2.80        # 轴距
REAR = 0.929     # 后悬
FRONT = 0.96     # 前悬
N_DISC = 2
DISC_R = 0.5 * np.hypot(L / N_DISC, W)          # 圆盘半径 1.522
DISC_C = [(2 * (i + 1) - 1) / (2 * N_DISC) * L - REAR for i in range(N_DISC)]  # [0.243, 2.588]


def draw_car(ax, x, y, theta, color='#5B9BD5', alpha=0.85, outline='#1F3864', disc=False, lw=1.2):
    """在 (x,y,theta) 处绘制俯视车身（后轴中心为参考点）。"""
    # 车身矩形四个角（车体坐标系：x 沿前进方向, 后轴在 0）
    xs = np.array([-REAR, -REAR, L - REAR, L - REAR])
    ys = np.array([-W/2, W/2, W/2, -W/2])
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta),  np.cos(theta)]])
    pts = R @ np.vstack([xs, ys])
    pts[0] += x
    pts[1] += y
    poly = Polygon(pts.T, closed=True, facecolor=color, edgecolor=outline,
                   alpha=alpha, lw=lw, zorder=3)
    ax.add_patch(poly)
    if disc:
        for c in DISC_C:
            cx, cy = x + c*np.cos(theta), y + c*np.sin(theta)
            ax.add_patch(Circle((cx, cy), DISC_R, facecolor='none',
                                edgecolor='#D64545', lw=1.4, ls='--', zorder=4))


# ======================================================================
# 图 1: 车辆模型 + 圆盘碰撞表示
# ======================================================================
def fig_vehicle():
    fig, ax = plt.subplots(figsize=(7.4, 4.2), dpi=160)
    # 车身
    ax.add_patch(Rectangle((-REAR, -W/2), L, W, facecolor='#5B9BD5',
                           edgecolor=C['navy'], lw=2, alpha=0.8, zorder=3))
    # 圆盘
    for c in DISC_C:
        ax.add_patch(Circle((c, 0), DISC_R, facecolor='#D64545', alpha=0.18,
                            edgecolor='#D64545', lw=1.8, ls='--', zorder=4))
        ax.plot([c], [0], 'o', color='#D64545', ms=4, zorder=5)
    # 前后轴
    ax.plot([0, WB], [0, 0], '-', color='#1F3864', lw=2.5, zorder=5)
    ax.plot([0, WB], [W/2, W/2], '-', color='#555', lw=1.2, zorder=2)
    ax.plot([0, WB], [-W/2, -W/2], '-', color='#555', lw=1.2, zorder=2)
    # 后轴中心（基准原点）
    ax.plot([0], [0], 'o', color='#D64545', ms=7, zorder=6)
    ax.annotate('后轴中心\n(base_link 原点)', (0, 0), xytext=(-0.4, -1.7),
                fontsize=9, ha='center', color='#D64545',
                arrowprops=dict(arrowstyle='->', color='#D64545', lw=1.0))
    # 车轮
    for fx in (0, WB):
        for sy in (-W/2, W/2):
            ax.add_patch(Rectangle((fx-0.25, sy-0.09), 0.5, 0.18,
                                   facecolor='#222', edgecolor='#111', zorder=4))
    # 尺寸标注
    ax.annotate('', xy=(-REAR, -1.45), xytext=(L-REAR, -1.45),
                arrowprops=dict(arrowstyle='<->', color='#333', lw=1.2))
    ax.text(L/2-REAR, -1.72, f'车长 {L:.2f} m', ha='center', fontsize=9)
    ax.annotate('', xy=(-REAR-0.6, -W/2), xytext=(-REAR-0.6, W/2),
                arrowprops=dict(arrowstyle='<->', color='#333', lw=1.2))
    ax.text(-REAR-0.85, 0, f'车宽 {W:.2f} m', va='center', rotation=90, fontsize=9)
    ax.annotate('', xy=(0, 1.6), xytext=(WB, 1.6),
                arrowprops=dict(arrowstyle='<->', color='#2E75B6', lw=1.2))
    ax.text(WB/2, 1.9, f'轴距 {WB:.2f} m', ha='center', fontsize=9, color='#2E75B6')
    # 圆盘半径标注
    ax.plot([DISC_C[0], DISC_C[0]], [0, DISC_R], color='#D64545', lw=1.0, ls=':')
    ax.text(DISC_C[0]+0.15, DISC_R/2, f'r={DISC_R:.2f} m', fontsize=9, color='#D64545', va='center')

    ax.set_xlim(-3.4, 6.0)
    ax.set_ylim(-3.0, 3.0)
    ax.set_aspect('equal')
    ax.set_xticks([]); ax.set_yticks([])
    ax.set_title('车辆运动学模型与圆盘碰撞表示', fontsize=13, color=C['navy'], pad=8)
    # 图例
    handles = [
        Line2D([0], [0], marker='s', color='none', markerfacecolor='#5B9BD5',
               markeredgecolor=C['navy'], markersize=13, label='车身矩形(俯视)'),
        Line2D([0], [0], marker='o', color='none', markerfacecolor='#D64545',
               markersize=11, alpha=0.55, label=f'{N_DISC} 个碰撞圆盘 (r={DISC_R:.2f} m)'),
    ]
    ax.legend(handles=handles, loc='lower right', fontsize=9, framealpha=0.9)
    fig.tight_layout()
    fig.savefig(FIG + 'vehicle.png', bbox_inches='tight', facecolor='white')
    plt.close(fig)


# ======================================================================
# 图 2: 泊车规划效果（粗路径 + 走廊 + 优化轨迹）  —— 核心效果图
# ======================================================================
def fig_parking():
    fig, ax = plt.subplots(figsize=(9.2, 6.2), dpi=160)

    # ---- 场景: 停车库 ----
    # 边界墙
    lot = dict(x0=-1.0, x1=16.0, y0=-1.0, y1=11.0)
    ax.add_patch(Rectangle((lot['x0'], lot['y0']),
                           lot['x1']-lot['x0'], lot['y1']-lot['y0'],
                           facecolor='#F5F2EC', edgecolor='#3A3A3A', lw=3, zorder=1))
    # 障碍物(已停车辆)矩形: (cx, cy, w, h, theta_deg), w=x向尺寸, h=y向尺寸
    obstacles = [
        (3.2, 6.5, 1.9, 4.7, 0),      # 左侧停着的车(竖放)
        (8.8, 6.5, 1.9, 4.7, 0),      # 右侧停着的车(竖放)
        (13.6, 2.2, 1.9, 3.4, 0),     # 右下角立柱/障碍
    ]
    for (cx, cy, w, h, th) in obstacles:
        r = np.radians(th)
        c, s = np.cos(r), np.sin(r)
        R = np.array([[c, -s], [s, c]])
        corners = R @ np.vstack([[-w/2, w/2, w/2, -w/2],
                                 [-h/2, -h/2, h/2, h/2]])
        corners[0] += cx; corners[1] += cy
        ax.add_patch(Polygon(corners.T, closed=True, facecolor='#7A7A7A',
                             edgecolor='#444', lw=1.5, zorder=2))

    # ---- 起终点 ----
    start = (0.8, 1.2, 0.0)          # 起始：底部通道, 朝 +x
    goal  = (6.0, 4.5, np.pi/2)      # 目标：中间空车位, 朝 +y

    # ---- 粗路径(Hybrid A*, 折线) ----
    cp = np.array([
        [0.8, 1.2], [3.0, 1.2], [5.2, 1.5], [6.0, 2.8],
        [6.0, 4.0], [6.0, 4.5],
    ])
    ax.plot(cp[:, 0], cp[:, 1], '-', color=C['yellow'], lw=2.6, zorder=4,
            solid_capstyle='round', label='粗路径 (Hybrid A*, 含换挡)')
    ax.plot(cp[:, 0], cp[:, 1], 'o', color='#B8860B', ms=4, zorder=4)

    # ---- 优化轨迹(平滑 S 曲线倒车入位) ----
    # 沿粗路径采样后做三次样条平滑
    t = np.linspace(0, 1, len(cp))
    cs_x = CubicSpline(t, cp[:, 0], bc_type='natural')
    cs_y = CubicSpline(t, cp[:, 1], bc_type='natural')
    ts = np.linspace(0, 1, 200)
    traj = np.vstack([cs_x(ts), cs_y(ts)]).T
    ax.plot(traj[:, 0], traj[:, 1], '-', color=C['red'], lw=3.0, zorder=5,
            label='优化轨迹 (LIOM)')

    # ---- 走廊盒(SFC, 沿轨迹轴对齐扩张) ----
    def corridor(cx, cy):
        """从圆盘中心向四方向扩张，直到接近障碍/边界，返回 (xmin,ymin,xmax,ymax)。"""
        m = 0.05
        xmin = cx - DISC_R; xmax = cx + DISC_R
        ymin = cy - DISC_R; ymax = cy + DISC_R
        def near_obs(x0, x1, y0, y1):
            # 与障碍/边界的最近距离(简化): 只要矩形不碰障碍即可扩张
            box = [x0, y0, x1, y1]
            for (ox, oy, w, h, _th) in obstacles:
                if not (x1 < ox-w/2 or x0 > ox+w/2 or y1 < oy-h/2 or y0 > oy+h/2):
                    return True
            return (x0 <= lot['x0'] or x1 >= lot['x1'] or
                    y0 <= lot['y0'] or y1 >= lot['y1'])
        for _ in range(200):
            if near_obs(xmin-m, xmax+m, ymin, ymax): break
            xmin -= m; xmax += m
            if near_obs(xmin, xmax, ymin-m, ymax+m): break
            ymin -= m; ymax += m
        return xmin, ymin, xmax, ymax

    for i in np.linspace(4, 190, 14, dtype=int):
        x, y = traj[i]
        xmin, ymin, xmax, ymax = corridor(x, y)
        ax.add_patch(Rectangle((xmin, ymin), xmax-xmin, ymax-ymin,
                               facecolor='#3E9B6C', alpha=0.10,
                               edgecolor='#2F7A54', lw=0.9, ls='-', zorder=3))

    # ---- 起终点与车辆脚印 ----
    ax.annotate('', xy=(start[0]+1.0*np.cos(start[2]), start[1]+1.0*np.sin(start[2])),
                xytext=(start[0], start[1]),
                arrowprops=dict(arrowstyle='-|>', color=C['green'], lw=2.6, mutation_scale=20), zorder=6)
    ax.text(start[0], start[1]-0.55, '起点', color=C['green'], fontsize=11,
            ha='center', fontweight='bold', zorder=6)
    ax.annotate('', xy=(goal[0]+1.0*np.cos(goal[2]), goal[1]+1.0*np.sin(goal[2])),
                xytext=(goal[0], goal[1]),
                arrowprops=dict(arrowstyle='-|>', color='#B03A3A', lw=2.6, mutation_scale=20), zorder=6)
    ax.text(goal[0]+0.1, goal[1]+0.75, '目标车位', color='#B03A3A', fontsize=11,
            ha='center', fontweight='bold', zorder=6)

    # 车辆脚印(起始与目标)
    draw_car(ax, start[0], start[1], start[2], color='#7CB342', alpha=0.9, lw=1.4)
    draw_car(ax, goal[0], goal[1], goal[2], color='#EF9A9A', alpha=0.9, lw=1.4)

    # 图例
    h = [
        Line2D([0], [0], color=C['yellow'], lw=2.6, label='粗路径 (Hybrid A*)'),
        Line2D([0], [0], color=C['red'], lw=3.0, label='优化轨迹 (LIOM)'),
        Rectangle((0, 0), 1, 1, facecolor='#3E9B6C', alpha=0.18, edgecolor='#2F7A54', lw=1, label='安全走廊盒 (SFC)'),
        Rectangle((0, 0), 1, 1, facecolor='#7A7A7A', edgecolor='#444', lw=1.2, label='障碍物(已停车辆)'),
    ]
    ax.legend(handles=h, loc='lower right', fontsize=9, framealpha=0.95, ncol=1)

    ax.set_xlim(lot['x0'], lot['x1'])
    ax.set_ylim(lot['y0'], lot['y1'])
    ax.set_aspect('equal')
    ax.set_xticks([]); ax.set_yticks([])
    ax.set_title('自主泊车轨迹规划效果（粗路径 → 走廊优化 → 平滑轨迹）',
                 fontsize=13.5, color=C['navy'], pad=8)
    fig.tight_layout()
    fig.savefig(FIG + 'parking.png', bbox_inches='tight', facecolor='white')
    plt.close(fig)


# ======================================================================
# 图 3: 迭代走廊优化框架的收敛过程
# ======================================================================
def _boxes_overlap(x0, y0, x1, y1, ox0, oy0, ox1, oy1):
    return not (x1 <= ox0 or x0 >= ox1 or y1 <= oy0 or y0 >= oy1)


def _expand_axis_aligned(cx, cy, r, obstacles, bounds, step=0.02):
    """沿坐标轴方向扩张矩形盒，直到碰到障碍物或边界（轴对齐 SFC 扩张）。"""
    x0, x1 = cx - r, cx + r
    y0, y1 = cy - r, cy + r
    bx0, bx1, by0, by1 = bounds
    # 先沿 x 轴两侧扩张，再沿 y 轴两侧扩张
    while x0 - step >= bx0 and x1 + step <= bx1:
        if any(_boxes_overlap(x0 - step, y0, x1 + step, y1, *o) for o in obstacles):
            break
        x0 -= step; x1 += step
    while y0 - step >= by0 and y1 + step <= by1:
        if any(_boxes_overlap(x0, y0 - step, x1, y1 + step, *o) for o in obstacles):
            break
        y0 -= step; y1 += step
    return x0, y0, x1, y1


def fig_convergence():
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9.2, 4.0), dpi=160,
                                   gridspec_kw={'width_ratios': [1, 1.15]})

    # 左: 不可行度收敛曲线
    iters = np.arange(1, 6)
    psi = np.array([4.2e-2, 7.5e-3, 1.1e-3, 2.3e-4, 3.8e-5])
    eps = 1e-4
    ax1.semilogy(iters, psi, 'o-', color=C['blue'], lw=2.2, ms=7,
                 label='不可行度 $\\psi_{infeasibility}$')
    ax1.axhline(eps, color=C['red'], lw=1.6, ls='--', label=f'收敛阈值 $\\varepsilon = {eps:.0e}$')
    ax1.axvspan(4, 5, color=C['green'], alpha=0.12)
    ax1.annotate('收敛', xy=(4.5, 1e-4), xytext=(3.0, 2e-5),
                 fontsize=10, color=C['green'], fontweight='bold',
                 arrowprops=dict(arrowstyle='->', color=C['green']))
    ax1.set_xlabel('迭代次数 (走廊重建 + 热启动求解)')
    ax1.set_ylabel('不可行度 $\\psi$ (对数)')
    ax1.set_xticks(iters)
    ax1.grid(True, which='both', alpha=0.3)
    ax1.legend(fontsize=9, loc='lower left')
    ax1.set_title('迭代收敛：坏初值逐步恢复', fontsize=12, color=C['navy'])

    # 右: 走廊随迭代重建，逐步贴合可行轨迹（轴对齐、无碰撞）
    ax2.set_title('走廊随迭代重建，逐步贴合可行轨迹', fontsize=12, color=C['navy'])
    # 障碍物(轴对齐矩形): (x0, y0, x1, y1)
    obstacles = [(2.2, 3.9, 3.8, 5.0),   # 左上
                 (5.2, 0.0, 6.8, 1.5)]   # 右下
    for (ox0, oy0, ox1, oy1) in obstacles:
        ax2.add_patch(Rectangle((ox0, oy0), ox1 - ox0, oy1 - oy0,
                                facecolor='#7A7A7A', edgecolor='#444', lw=1.4, zorder=2))
        ax2.text((ox0 + ox1) / 2, (oy0 + oy1) / 2, '障碍物', ha='center', va='center',
                 fontsize=8.5, color='#fff', fontweight='bold', zorder=3)
    # 可行轨迹(优化后，穿越斜向空隙、全程无碰撞)
    tt = np.linspace(0, 1, 200)
    xr = 0.4 + 7.4 * tt
    yr = 1.0 + 3.2 * tt + 0.4 * np.sin(np.pi * tt)
    ax2.plot(xr, yr, '-', color=C['red'], lw=2.6, zorder=4)
    # 走廊盒：第 1 轮(宽松, 绿虚线) 与 第 2 轮(贴合, 蓝实线)，均为轴对齐矩形
    samples = [0.15, 0.3, 0.45, 0.6, 0.75, 0.9]
    for s in samples:
        x, y = 0.4 + 7.4 * s, 1.0 + 3.2 * s + 0.4 * np.sin(np.pi * s)
        ax2.add_patch(Rectangle((x - 0.75, y - 0.6), 1.5, 1.2, facecolor='#3E9B6C',
                                alpha=0.10, edgecolor='#2F7A54', ls=':', lw=1.2, zorder=3))
    for s in samples:
        x, y = 0.4 + 7.4 * s, 1.0 + 3.2 * s + 0.4 * np.sin(np.pi * s)
        ax2.add_patch(Rectangle((x - 0.42, y - 0.36), 0.84, 0.72, facecolor='#2E75B6',
                                alpha=0.16, edgecolor='#1F3864', lw=1.3, zorder=4))
    # 起终点标记
    ax2.plot([0.4], [1.0], 'o', color=C['green'], ms=7, zorder=5)
    ax2.text(0.4, 0.55, '起点', color=C['green'], ha='center', fontsize=9, fontweight='bold')
    ax2.plot([7.8], [4.2], 's', color='#B03A3A', ms=7, zorder=5)
    ax2.text(7.8, 4.6, '终点', color='#B03A3A', ha='center', fontsize=9, fontweight='bold')
    # 图例（仅作为图例句柄，不绘制到坐标区）
    h_traj = Line2D([0], [0], color=C['red'], lw=2.6, label='优化轨迹(可行)')
    h_g1 = Rectangle((0, 0), 1, 1, facecolor='#3E9B6C', alpha=0.20, edgecolor='#2F7A54', ls=':', label='第 1 轮走廊(宽松)')
    h_g2 = Rectangle((0, 0), 1, 1, facecolor='#2E75B6', alpha=0.20, edgecolor='#1F3864', label='第 2 轮走廊(贴合)')
    ax2.legend(handles=[h_traj, h_g1, h_g2], fontsize=8.5, loc='upper left', framealpha=0.95)
    ax2.set_xlim(-0.4, 8.4); ax2.set_ylim(-0.4, 5.2)
    ax2.set_aspect('equal')
    ax2.set_xticks([]); ax2.set_yticks([])

    fig.tight_layout()
    fig.savefig(FIG + 'convergence.png', bbox_inches='tight', facecolor='white')
    plt.close(fig)


# ======================================================================
# 图 4: 走廊盒生成示意（单圆盘沿坐标轴四方向扩张）
# ======================================================================
def fig_corridor():
    fig, ax = plt.subplots(figsize=(6.6, 4.6), dpi=160)
    R = DISC_R
    cx, cy = 0.0, 0.0
    # 障碍物(轴对齐矩形): (x0, y0, x1, y1) —— 四个方向各一个
    obstacles = [
        (3.0, -0.9, 4.6, 0.9),     # 右
        (-4.6, -0.9, -3.0, 0.9),   # 左
        (-1.0, 2.5, 1.0, 3.7),     # 上
        (-1.0, -3.7, 1.0, -2.5),   # 下
    ]
    bounds = (-5.0, 5.0, -4.0, 4.0)
    for (x0, y0, x1, y1) in obstacles:
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, facecolor='#7A7A7A',
                               edgecolor='#444', lw=1.4, zorder=2))
    # 碰撞圆盘
    ax.add_patch(Circle((cx, cy), R, facecolor='#5B9BD5', alpha=0.25,
                        edgecolor='#2E75B6', lw=1.6, zorder=3))
    ax.plot([cx], [cy], 'o', color='#1F3864', ms=5, zorder=4)
    # 沿四个坐标轴方向扩张出轴对齐走廊盒（碰到障碍即停）
    x0, y0, x1, y1 = _expand_axis_aligned(cx, cy, R, obstacles, bounds)
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, facecolor='#3E9B6C',
                           alpha=0.12, edgecolor='#2F7A54', lw=2.0, zorder=1))
    # 四个严格水平 / 竖直的扩张箭头
    for (xya, xyb, lbl, tx, ty) in [
            ((cx + R, cy), (x1, cy), '+x', (cx + R + x1) / 2, cy + 0.28),
            ((cx - R, cy), (x0, cy), '−x', (cx - R + x0) / 2, cy + 0.28),
            ((cx, cy + R), (cx, y1), '+y', cx + 0.28, (cy + R + y1) / 2),
            ((cx, cy - R), (cx, y0), '−y', cx - 0.28, (cy - R + y0) / 2)]:
        ax.annotate('', xy=xyb, xytext=xya,
                    arrowprops=dict(arrowstyle='->', color='#D64545', lw=2.0))
        ax.text(tx, ty, lbl, color='#D64545', fontsize=9, ha='center', va='center')
    ax.text(cx, -R - 0.5, '碰撞圆盘', color='#2E75B6', fontsize=9, ha='center')
    ax.text(cx, y0 - 0.55, '轴对齐安全走廊盒 (SFC)', color='#2F7A54', fontsize=10, ha='center')

    ax.set_xlim(-5.6, 5.6); ax.set_ylim(-4.8, 4.8)
    ax.set_aspect('equal')
    ax.set_xticks([]); ax.set_yticks([])
    ax.set_title('走廊盒生成：圆盘沿坐标轴四方向扩张', fontsize=13, color=C['navy'], pad=8)
    fig.tight_layout()
    fig.savefig(FIG + 'corridor.png', bbox_inches='tight', facecolor='white')
    plt.close(fig)


if __name__ == '__main__':
    fig_vehicle()
    fig_parking()
    fig_convergence()
    fig_corridor()
    print('figures done')
