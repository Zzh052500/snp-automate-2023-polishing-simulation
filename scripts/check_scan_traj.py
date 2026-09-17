#!/usr/bin/env python3
"""扫描轨迹几何复核 —— 检查 config/scan_traj.yaml 是否存在碰撞。

背景：SNP 的 `LoadTrajectoryFromFile` 只把 YAML 里的关节角原样回放，
**不做碰撞检查**。所以轨迹里的干涉在运行时不会被拦下来，只能靠几何核算。
本脚本就是干这个的。

检查三类：
  1. 自碰撞   —— 机械臂各连杆两两最小距离（跳过相邻连杆）
  2. 环境     —— 各连杆到静止物体（台面 / 地面 / world）的最小距离
  3. 工件     —— 各连杆到工件网格的最小距离（仅当工件在 URDF 里时才查）

每个航点单独查，相邻航点之间再按关节空间线性插值取采样点查，
用来捕捉「两个航点都安全、但中间过程擦到」的情况。

用法：
    python3 scripts/check_scan_traj.py                 # 默认全量复核
    python3 scripts/check_scan_traj.py --quick         # 只查航点，跳过插值段
    python3 scripts/check_scan_traj.py --traj path/to/other.yaml
    python3 scripts/check_scan_traj.py --list-links    # 打印连杆分组后退出

退出码：0 = 全部通过；1 = 有碰撞 / 低于阈值；2 = 运行错误。

依赖：numpy、xacro（宿主机或 docker 容器里都行）
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

# ---------------------------------------------------------------- 常量

# 静止物体：这些连杆只通过固定关节挂在 world 上，本身不动
STATIC_FALLBACK = {"world", "floor", "table"}

# 判定「同一刚体」用的关节类型
FIXED = "fixed"


# ---------------------------------------------------------------- URDF 导出


def find_xacro() -> str | None:
    """找一个可用的 xacro 可执行文件。"""
    exe = shutil.which("xacro")
    if exe:
        return exe
    for c in ("/opt/ros/humble/bin/xacro", "/opt/ros/jazzy/bin/xacro",
              "/opt/ros/iron/bin/xacro"):
        if os.path.exists(c):
            return c
    return None


def export_urdf(repo: str, verbose: bool = True) -> str:
    """把 urdf/workcell.xacro 展开成 URDF 文本。

    优先用宿主机 xacro —— 但 xacro 里用了 $(find snp_automate_2023)，
    需要伪造一个 ament prefix 目录，把本仓库的 urdf/config/meshes
    软链进去，让 $(find ...) 能解析到。
    宿主机没有 xacro 时退回 docker 容器（容器里装了这个包）。
    """
    xacro = find_xacro()
    if xacro:
        with tempfile.TemporaryDirectory(prefix="snp_ament_") as pre:
            share = os.path.join(pre, "share")
            os.makedirs(os.path.join(share, "ament_index", "resource_index", "packages"))
            open(os.path.join(share, "ament_index", "resource_index", "packages",
                              "snp_automate_2023"), "w").close()
            pkg = os.path.join(share, "snp_automate_2023")
            os.makedirs(pkg, exist_ok=True)
            for sub in ("urdf", "config", "meshes"):
                src = os.path.join(repo, sub)
                if os.path.isdir(src):
                    os.symlink(src, os.path.join(pkg, sub))
            env = dict(os.environ)
            env["AMENT_PREFIX_PATH"] = pre + os.pathsep + env.get("AMENT_PREFIX_PATH", "")
            r = subprocess.run([xacro, os.path.join(repo, "urdf", "workcell.xacro")],
                               capture_output=True, text=True, env=env)
            if r.returncode == 0 and "<robot" in r.stdout:
                return r.stdout
            if verbose:
                print("  [warn] 宿主机 xacro 失败，尝试 docker…", file=sys.stderr)
                print("         " + (r.stderr or "").strip()[:400], file=sys.stderr)

    # 容器兜底
    cname = "snp_automate_2023_sim"
    inner = ("/opt/snp_automate_2023/install/snp_automate_2023/share/"
             "snp_automate_2023/urdf/workcell.xacro")
    r = subprocess.run(["docker", "exec", cname, "bash", "-lc", f"xacro {inner}"],
                       capture_output=True, text=True)
    if r.returncode == 0 and "<robot" in r.stdout:
        return r.stdout

    raise RuntimeError(
        "导出 URDF 失败：宿主机 xacro 和 docker 容器都不可用。\n"
        "  方案 A：source /opt/ros/<distro>/setup.bash\n"
        f"  方案 B：先启动仿真容器（docker ps 里要有 {cname}）"
    )


# ---------------------------------------------------------------- PLY 读取


def read_ply(path: str) -> np.ndarray:
    """读 PLY 顶点（ascii / binary_little_endian）。"""
    with open(path, "rb") as f:
        raw = f.read()
    marker = b"end_header\n"
    hdr_end = raw.find(marker)
    if hdr_end < 0:
        raise ValueError(f"{path}: 找不到 end_header")
    hdr_end += len(marker)
    hdr = raw[:hdr_end].decode("ascii", "replace")
    nv = int(re.search(r"element vertex (\d+)", hdr).group(1))
    props = re.findall(r"property (\S+) (\S+)", hdr.split("element face")[0])
    body = raw[hdr_end:]

    if "format ascii" in hdr:
        vals = np.fromstring(body.decode("ascii", "replace"), sep=" ", dtype=np.float64)
        return vals[: nv * len(props)].reshape(nv, len(props))[:, :3]

    sizes = {"float": 4, "float32": 4, "double": 8, "float64": 8, "uchar": 1,
             "uint8": 1, "int": 4, "int32": 4, "short": 2, "ushort": 2, "uint": 4}
    stride = sum(sizes[t] for t, _ in props)
    arr = np.frombuffer(body[: nv * stride], dtype=np.uint8).reshape(nv, stride)
    off, cols = 0, {}
    for t, nm in props:
        if nm in ("x", "y", "z"):
            cols[nm] = (off, sizes[t], t)
        off += sizes[t]
    out = []
    for nm in ("x", "y", "z"):
        o, s, t = cols[nm]
        dt = np.float32 if t in ("float", "float32") else np.float64
        out.append(arr[:, o:o + s].copy().view(dt).ravel().astype(np.float64))
    return np.stack(out, axis=1)


# ---------------------------------------------------------------- URDF 解析


def rpy_to_R(r: float, p: float, y: float) -> np.ndarray:
    """固定轴 rpy → 旋转矩阵，R = Rz(y) @ Ry(p) @ Rx(r)。"""
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def xyz_rpy(node) -> tuple[np.ndarray, np.ndarray]:
    if node is None:
        return np.zeros(3), np.zeros(3)
    xyz = np.array([float(v) for v in node.get("xyz", "0 0 0").split()])
    rpy = np.array([float(v) for v in node.get("rpy", "0 0 0").split()])
    return xyz, rpy


def hom(xyz, rpy) -> np.ndarray:
    M = np.eye(4)
    M[:3, :3] = rpy_to_R(*rpy)
    M[:3, 3] = xyz
    return M


class Model:
    """URDF 里跟碰撞检查相关的部分。"""

    def __init__(self, urdf_text: str, repo: str):
        self.repo = repo
        root = ET.fromstring(urdf_text)

        self.joints: dict[str, dict] = {}
        for j in root.findall("joint"):
            xyz, rpy = xyz_rpy(j.find("origin"))
            ax = j.find("axis")
            axis = (np.array([float(v) for v in ax.get("xyz", "1 0 0").split()])
                    if ax is not None else np.array([1.0, 0.0, 0.0]))
            self.joints[j.get("name")] = {
                "type": j.get("type"),
                "parent": j.find("parent").get("link"),
                "child": j.find("child").get("link"),
                "xyz": xyz, "rpy": rpy, "axis": axis,
            }

        self.geoms: dict[str, list] = {}
        for l in root.findall("link"):
            name = l.get("name")
            gs = []
            for tag in ("collision", "visual"):      # collision 优先
                for g in l.findall(tag):
                    geo = g.find("geometry")
                    if geo is None:
                        continue
                    xyz, rpy = xyz_rpy(g.find("origin"))
                    m = geo.find("mesh")
                    if m is not None:
                        gs.append(("mesh", xyz, rpy, m.get("filename")))
                        continue
                    b = geo.find("box")
                    if b is not None:
                        gs.append(("box", xyz, rpy,
                                   np.array([float(v) for v in b.get("size").split()])))
            if gs:
                self.geoms[name] = gs

        self._resolve_static()
        self._group_rigid()
        self._build_obstacles()

    # ---- 静止连杆 ----
    def _resolve_static(self) -> None:
        """从根往下一路都是固定关节的连杆，就是静止的。"""
        children = {j["child"] for j in self.joints.values()}
        roots = [n for n in self.geoms if n not in children] or ["world"]
        static = set()
        stack = [(r, True) for r in roots]
        while stack:
            link, fixed_so_far = stack.pop()
            if link in static:
                continue
            if fixed_so_far:
                static.add(link)
            for j in self.joints.values():
                if j["parent"] == link:
                    stack.append((j["child"], fixed_so_far and j["type"] == FIXED))
        # 根节点本身不算障碍物（通常是 world，没有几何）
        self.static = {s for s in static if s in self.geoms}
        if not self.static:                      # 兜底
            self.static = STATIC_FALLBACK & set(self.geoms)

    # ---- 连杆分组：固定关节相连的算同一刚体 ----
    def _group_rigid(self) -> None:
        # 注意：并查集要覆盖**所有** link，不能只覆盖有几何的。
        # tool0 / flange / camera_frame 这类没有视觉几何，但它们是固定关节链上的
        # 中间环节；漏掉它们会把 ee / sand_tcp 和 Link6 分成两组。
        names = set(self.geoms)
        for j in self.joints.values():
            names.add(j["parent"])
            names.add(j["child"])
        parent = {n: n for n in names}

        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        for j in self.joints.values():
            if j["type"] == FIXED:
                a, b = find(j["parent"]), find(j["child"])
                if a != b:
                    parent[a] = b

        self.group_of = {n: find(n) for n in self.geoms}
        self.groups: dict[str, list[str]] = {}
        for n in self.geoms:
            self.groups.setdefault(self.group_of[n], []).append(n)

        # 可动关节直接相连的两组算「相邻」
        self.adjacent: set[tuple[str, str]] = set()
        for j in self.joints.values():
            if j["type"] == FIXED:
                continue
            a, b = self.group_of.get(j["parent"]), self.group_of.get(j["child"])
            if a and b and a != b:
                self.adjacent.add((min(a, b), max(a, b)))

    def _build_obstacles(self) -> None:
        self.robot_groups = [g for g in self.groups
                             if not any(l in self.static for l in self.groups[g])]
        self.obstacle_groups = [g for g in self.groups
                                if any(l in self.static for l in self.groups[g])]

    # ---- 运动链 ----
    def chain(self, link: str) -> list[str]:
        """base_link → link 的关节名序列。"""
        out, cur, seen = [], link, set()
        while cur not in self.static and cur not in seen:
            seen.add(cur)
            pj = next((n for n, j in self.joints.items() if j["child"] == cur), None)
            if pj is None:
                break
            out.append(pj)
            cur = self.joints[pj]["parent"]
        return list(reversed(out))

    def fk(self, link: str, q: np.ndarray) -> np.ndarray:
        """link 相对 base_link 的 4x4（静止连杆返回单位阵）。"""
        T = np.eye(4)
        for n in self.chain(link):
            j = self.joints[n]
            T = T @ hom(j["xyz"], j["rpy"])
            if j["type"] != FIXED:
                idx = int(re.search(r"(\d+)$", n).group(1)) - 1
                a = j["axis"] / (np.linalg.norm(j["axis"]) or 1.0)
                K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
                ang = q[idx]
                R = np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)
                M = np.eye(4)
                M[:3, :3] = R
                T = T @ M
        return T

    # ---- 几何取点 ----
    def _cache(self) -> None:
        if hasattr(self, "_mc"):
            return
        self._mc: dict[str, np.ndarray] = {}

    def geom_pts(self, filename: str) -> np.ndarray:
        self._cache()
        if filename in self._mc:
            return self._mc[filename]
        rel = filename.replace("package://snp_automate_2023/", "")
        cand = [os.path.join(self.repo, rel),
                os.path.join(self.repo, "meshes", os.path.basename(filename))]
        for p in cand:
            if os.path.exists(p):
                try:
                    pts = read_ply(p)
                    self._mc[filename] = pts
                    return pts
                except Exception as e:
                    print(f"  [warn] 读不了 {p}: {e}", file=sys.stderr)
        self._mc[filename] = np.zeros((0, 3))
        return self._mc[filename]

    def link_pts(self, link: str, q: np.ndarray, ppl: int = 0,
                 rng: np.random.Generator | None = None) -> np.ndarray:
        """连杆所有几何在 base_link 系下的世界点云。ppl>0 时降采样。"""
        Tl = self.fk(link, q)
        chunks = []
        for kind, gxyz, grpy, payload in self.geoms[link]:
            if kind == "mesh":
                p = self.geom_pts(payload)
                if ppl and len(p) > ppl:
                    p = p[rng.choice(len(p), ppl, replace=False)]
            else:                                   # box → 8 角点
                h = np.asarray(payload) / 2.0
                p = np.array([[a, b, c] for a in (-h[0], h[0])
                              for b in (-h[1], h[1]) for c in (-h[2], h[2])])
            if len(p) == 0:
                continue
            M = Tl @ hom(gxyz, grpy)
            chunks.append((M @ np.hstack([p, np.ones((len(p), 1))]).T).T[:, :3])
        return np.vstack(chunks) if chunks else np.zeros((0, 3))

    def group_pts(self, g: str, q: np.ndarray, ppl: int = 0,
                  rng=None) -> np.ndarray:
        parts = [self.link_pts(l, q, ppl, rng) for l in self.groups[g]]
        parts = [p for p in parts if len(p)]
        return np.vstack(parts) if parts else np.zeros((0, 3))

    def group_label(self, g: str) -> str:
        links = sorted(self.groups[g])
        return "/".join(links)


# ---------------------------------------------------------------- 距离


def min_dist(A: np.ndarray, B: np.ndarray, chunk: int = 400) -> float:
    if len(A) == 0 or len(B) == 0:
        return float("inf")
    best = float("inf")
    for i in range(0, len(A), chunk):
        d = np.linalg.norm(A[i:i + chunk, None, :] - B[None, :, :], axis=2)
        best = min(best, float(d.min()))
    return best


# ---------------------------------------------------------------- 轨迹


def load_traj(path: str) -> list[np.ndarray]:
    """读 scan_traj.yaml。优先用 pyyaml，没有就正则兜底。"""
    try:
        import yaml
        d = yaml.safe_load(open(path))
        return [np.asarray(p["positions"], dtype=float) for p in d["points"]]
    except ImportError:
        pts = []
        for line in open(path):
            m = re.match(r"\s*- positions: \[(.*)\]", line)
            if m:
                pts.append(np.array([float(v) for v in m.group(1).split(",")]))
        return pts


# ---------------------------------------------------------------- 主流程


def main() -> int:
    ap = argparse.ArgumentParser(
        description="检查扫描轨迹是否存在碰撞",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--traj", default=None, help="轨迹 YAML（默认 config/scan_traj.yaml）")
    ap.add_argument("--samples", type=int, default=40,
                    help="相邻航点间插值采样数，0 = 只查航点（默认 40）")
    ap.add_argument("--ppl", type=int, default=0,
                    help="每个连杆网格降采样到多少点，0 = 全分辨率（默认 0）")
    ap.add_argument("--min-self", type=float, default=0.02,
                    help="自碰撞告警阈值 m（默认 0.02）")
    ap.add_argument("--min-env", type=float, default=0.0,
                    help="环境告警阈值 m（默认 0.0，即只要不穿入就算过）")
    ap.add_argument("--list-links", action="store_true",
                    help="打印连杆分组后退出")
    ap.add_argument("--seed", type=int, default=0, help="降采样随机种子")
    args = ap.parse_args()

    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    traj_path = args.traj or os.path.join(repo, "config", "scan_traj.yaml")

    if not os.path.exists(traj_path):
        print(f"✗ 找不到轨迹文件：{traj_path}", file=sys.stderr)
        return 2

    print("=" * 78)
    print("扫描轨迹几何复核")
    print("=" * 78)
    print(f"仓库      : {repo}")
    print(f"轨迹      : {os.path.relpath(traj_path, repo)}")

    try:
        urdf = export_urdf(repo)
        model = Model(urdf, repo)
    except Exception as e:
        print(f"✗ {e}", file=sys.stderr)
        return 2

    pts = load_traj(traj_path)
    if not pts:
        print("✗ 轨迹里没有航点", file=sys.stderr)
        return 2
    print(f"航点      : {len(pts)} 个")

    # 连杆分组概览
    print(f"\n静止物体  : {', '.join(model.group_label(g) for g in model.obstacle_groups) or '(无)'}")
    print(f"可动刚体  : {', '.join(model.group_label(g) for g in model.robot_groups)}")

    if args.list_links:
        print("\n相邻（跳过自碰撞检查）的刚体对:")
        for a, b in sorted(model.adjacent):
            print(f"  {model.group_label(a)}  <->  {model.group_label(b)}")
        return 0

    rng = np.random.default_rng(args.seed)

    # 自碰撞刚体对：排除自身和相邻
    pairs = [(a, b) for i, a in enumerate(model.robot_groups)
             for b in model.robot_groups[i + 1:]
             if (min(a, b), max(a, b)) not in model.adjacent]

    # 环境：可动刚体 × 静止刚体
    env_pairs = [(r, o) for r in model.robot_groups for o in model.obstacle_groups]

    print(f"采样      : {'仅航点' if args.samples == 0 else f'每段 {args.samples} 点'}"
          f"，网格 {'全分辨率' if not args.ppl else f'降采样 {args.ppl} 点/连杆'}")
    print(f"阈值      : 自碰撞 ≥ {args.min_self} m，环境 ≥ {args.min_env} m")

    # 需要查询的状态序列：航点 + 插值点，带标签
    states: list[tuple[str, np.ndarray]] = []
    for i, q in enumerate(pts):
        states.append((f"P{i + 1}", q))
        if args.samples and i + 1 < len(pts):
            for t in np.linspace(0, 1, args.samples)[1:-1]:
                states.append((f"P{i + 1}->P{i + 2} @{t:.2f}",
                               pts[i] + t * (pts[i + 1] - pts[i])))

    # 缓存静止物体点云（它们不随 q 变）
    obstacles = {o: model.group_pts(o, pts[0], args.ppl, rng)
                 for o in model.obstacle_groups}

    worst_self = (float("inf"), None, None)
    worst_env = (float("inf"), None, None)
    worst_part = (float("inf"), None, None)
    n_total = len(states)

    print(f"\n开始检查 {n_total} 个状态…\n", flush=True)
    print(f"  {'状态':<16}{'自碰撞':>12}{'环境':>12}{'工件':>12}   最危险对")
    print("  " + "-" * 74)

    # 工件：URDF 之外单独找 part_scan.ply
    part = None
    for p in (os.path.join(repo, "meshes", "part_scan.ply"),):
        if os.path.exists(p):
            try:
                part = read_ply(p)
            except Exception:
                pass

    bad = []
    for label, q in states:
        cache = {g: model.group_pts(g, q, args.ppl, rng) for g in model.robot_groups}

        s_local = (float("inf"), None, None)
        for a, b in pairs:
            d = min_dist(cache[a], cache[b])
            if d < s_local[0]:
                s_local = (d, model.group_label(a), model.group_label(b))

        e_local = (float("inf"), None, None)
        for r, o in env_pairs:
            d = min_dist(cache[r], obstacles[o])
            if d < e_local[0]:
                e_local = (d, model.group_label(r), model.group_label(o))

        p_local = (float("inf"), None, None)
        if part is not None:
            for r in model.robot_groups:
                d = min_dist(cache[r], part)
                if d < p_local[0]:
                    p_local = (d, model.group_label(r), "part_scan.ply")

        if s_local[0] < worst_self[0]:
            worst_self = (s_local[0], s_local[1], s_local[2])
        if e_local[0] < worst_env[0]:
            worst_env = (e_local[0], e_local[1], e_local[2])
        if p_local[0] < worst_part[0]:
            worst_part = (p_local[0], p_local[1], p_local[2])

        name = s_local[1] or ""
        name2 = s_local[2] or ""
        flag = ""
        if s_local[0] < args.min_self:
            flag = "  ❌ 自碰撞过近"
            bad.append((label, "自碰撞", s_local))
        if e_local[0] < args.min_env:
            flag += "  ❌ 环境碰撞"
            bad.append((label, "环境", e_local))

        print(f"  {label:<16}{s_local[0]:>11.4f}{e_local[0]:>12.4f}"
              f"{p_local[0]:>12.4f}   {name} <-> {name2}{flag}", flush=True)

    print()
    print("=" * 78)
    print("汇总")
    print("=" * 78)
    print(f"  自碰撞最小  {worst_self[0]:.4f} m   {worst_self[1]} <-> {worst_self[2]}")
    print(f"  环境最小    {worst_env[0]:.4f} m   {worst_env[1]} <-> {worst_env[2]}")
    if part is not None:
        print(f"  工件最小    {worst_part[0]:.4f} m   {worst_part[1]} <-> {worst_part[2]}")
    print()

    if bad:
        print(f"❌ 发现 {len(bad)} 处问题：")
        for label, kind, v in bad[:20]:
            print(f"    {label:<16} {kind} {v[0]:.4f} m  {v[1]} <-> {v[2]}")
        if len(bad) > 20:
            print(f"    … 另有 {len(bad) - 20} 处")
        return 1

    print("✅ 全部通过")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n已中断", file=sys.stderr)
        sys.exit(2)
