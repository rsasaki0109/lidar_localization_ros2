#!/usr/bin/env python3
"""Render a global localization run (no initial pose) as a README GIF.

The GIF shows a recorded quickstart run in four phases: the live scan with no
pose, the map-wide search (every candidate of every query up to the accepted
one, from the global_localization_node log), the scan placed at the first
published pose, and the tracking that follows, with the estimated path and the
dashed ground truth.

Inputs are what a ``tools/benchmark/run_benchmark.py`` run leaves in its run
directory: ``est.tum`` (/pcl_pose), ``status.jsonl`` (startup states, sim time)
and ``quickstart.log`` (the ``query answered:`` JSON of each search). Scans come
from the replayed bag. Display choices: the view is fitted to the candidates
and the path (the map and the search extend beyond it), map points above
``--map-z-max`` (the ceiling) are left out, and the move of the scan from the
inset onto the map is a screen-space interpolation between its real start
(sensor frame, levelled with the odometry roll and pitch) and end (first
published pose). Poses between two published ones are interpolated linearly.

usage: render_global_localization.py --map MAP.pcd --bag BAG --run RUN_DIR
           --gt GT.tum --out OUT.gif [--caption TEXT] [--stills 0,60,240]
Needs numpy, Pillow, a sourced ROS 2 (rosbag2_py, yaml) and ffmpeg.
"""

from __future__ import annotations

import argparse
import functools
import json
import math
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

W, H = 720, 405  # output size
SS = 2  # supersampling of the point layers
TOP, BOTTOM = 34, 26  # title and caption bars
FPS = 12
BG = (6, 8, 12)
TITLE = (245, 170, 60)
TEXT = (210, 225, 235)
DIM = (130, 145, 160)
SCAN = np.array([255, 120, 40], dtype=np.float32)
EST = (90, 220, 255)
GT_COLOR = (255, 255, 255)
GOLD = (255, 214, 70)
CAND = (255, 95, 120)
# Height colour ramp (low to high), like the hero GIF's blue-green map.
RAMP = np.array(
    [[40, 40, 120], [40, 90, 170], [30, 150, 170], [60, 190, 130], [190, 220, 90]],
    dtype=np.float32,
)


@functools.lru_cache(maxsize=None)
def font(size: int, bold: bool = False):
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    for root in ("/usr/share/fonts/truetype/dejavu", "/usr/share/fonts/TTF"):
        path = Path(root) / name
        if path.is_file():
            return ImageFont.truetype(str(path), size)
    return ImageFont.load_default()


# ---------------------------------------------------------------- inputs


def read_pcd_xyz(path: Path) -> np.ndarray:
    """x, y, z of a binary PCD whose first three fields are float32 x y z."""
    data = path.read_bytes()
    header_end = data.index(b"DATA binary\n") + len(b"DATA binary\n")
    header = data[:header_end].decode().splitlines()
    fields = dict(line.split(" ", 1) for line in header if " " in line)
    if not fields["FIELDS"].startswith("x y z"):
        raise ValueError("expected a PCD starting with fields x y z")
    count = int(fields["POINTS"])
    step = sum(int(s) for s in fields["SIZE"].split())
    raw = np.frombuffer(data[header_end:], dtype=np.uint8, count=count * step)
    return raw.reshape(count, step)[:, :12].copy().view("<f4").reshape(count, 3)


def quat_matrix(q) -> np.ndarray:
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def load_run(run_dir: Path):
    """Published poses, startup states and the search answers of one run."""
    est = np.loadtxt(run_dir / "est.tum", ndmin=2)
    states = []
    for line in (run_dir / "status.jsonl").read_text().splitlines():
        row = json.loads(line)
        states.append((row["sim_time_sec"], json.loads(row["status"])))
    queries = []
    marker = "query answered: "
    for line in (run_dir / "quickstart.log").read_text(errors="replace").splitlines():
        if marker in line:
            queries.append(json.loads(line.split(marker, 1)[1]))
    return est, states, queries


def load_scans(bag: Path, stamps, topic: str = "/livox/lidar", keep: int = 6000):
    """The scan nearest to each requested stamp: {stamp: (scan_stamp, xyz)}."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2

    wanted = np.sort(np.asarray(stamps, dtype=float))
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    found = {}
    rng = np.random.default_rng(0)
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, PointCloud2)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        near = wanted[np.abs(wanted - stamp) < 0.051]
        if near.size == 0:
            continue
        offsets = {f.name: f.offset for f in msg.fields}
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        raw = raw.reshape(-1, msg.point_step)
        xyz = np.stack(
            [
                raw[:, offsets[a] : offsets[a] + 4].copy().view("<f4")[:, 0]
                for a in "xyz"
            ],
            axis=1,
        ).astype(float)
        rng_m = np.linalg.norm(xyz, axis=1)
        xyz = xyz[np.isfinite(rng_m) & (rng_m > 0.8) & (rng_m < 40.0)]
        if len(xyz) > keep:
            xyz = xyz[rng.choice(len(xyz), keep, replace=False)]
        for s in near:
            if s not in found or abs(found[s][0] - s) > abs(stamp - s):
                found[s] = (stamp, xyz)
    return found


def bag_start(bag: Path) -> float:
    """Start time of a rosbag2 directory, from its metadata.yaml."""
    import yaml

    info = yaml.safe_load((bag / "metadata.yaml").read_text())
    return (
        info["rosbag2_bagfile_information"]["starting_time"]["nanoseconds_since_epoch"]
        * 1e-9
    )


def pose_at(traj: np.ndarray, t: float) -> np.ndarray:
    """4x4 pose of a TUM trajectory at t (linear position, nearest rotation)."""
    i = int(np.clip(np.searchsorted(traj[:, 0], t), 1, len(traj) - 1))
    a, b = traj[i - 1], traj[i]
    w = float(np.clip((t - a[0]) / max(b[0] - a[0], 1e-9), 0.0, 1.0))
    pose = np.eye(4)
    pose[:3, :3] = quat_matrix((b if w > 0.5 else a)[4:8])
    pose[:3, 3] = a[1:4] + w * (b[1:4] - a[1:4])
    return pose


# ---------------------------------------------------------------- drawing


class Camera:
    """Orthographic 3/4 view, fitted to the given points."""

    def __init__(self, fit: np.ndarray, azimuth_deg: float, elevation_deg: float):
        self.az = math.radians(azimuth_deg)
        self.el = math.radians(elevation_deg)
        self.scale, self.offset = 1.0, np.zeros(2)
        uv, _ = self.project(fit)
        lo, hi = uv.min(axis=0), uv.max(axis=0)
        avail = np.array([W - 24, H - TOP - BOTTOM - 16], dtype=float)
        self.scale = float(np.min(avail / (hi - lo)))
        center = (lo + hi) / 2 * self.scale
        self.offset = np.array([W / 2, TOP + 8 + avail[1] / 2]) - center

    def project(self, xyz: np.ndarray):
        """Screen pixels (final resolution) and depth (larger is farther)."""
        c, s = math.cos(self.az), math.sin(self.az)
        xr = c * xyz[:, 0] + s * xyz[:, 1]
        yr = -s * xyz[:, 0] + c * xyz[:, 1]
        up = yr * math.sin(self.el) + xyz[:, 2] * math.cos(self.el)
        depth = yr * math.cos(self.el) - xyz[:, 2] * math.sin(self.el)
        uv = np.stack([xr * self.scale, -up * self.scale], axis=1) + self.offset
        return uv, depth


def splat(canvas: np.ndarray, uv: np.ndarray, rgb: np.ndarray, alpha, size: int):
    """Alpha-blend square points into an SS-resolution float canvas."""
    px = np.round(uv * SS).astype(int)
    alpha = np.broadcast_to(np.asarray(alpha, dtype=np.float32), (len(px),))
    rgb = np.broadcast_to(rgb, (len(px), 3))
    h, w = canvas.shape[:2]
    for dx in range(size):
        for dy in range(size):
            x, y = px[:, 0] + dx, px[:, 1] + dy
            ok = (x >= 0) & (x < w) & (y >= 0) & (y < h)
            a = alpha[ok, None]
            canvas[y[ok], x[ok]] = canvas[y[ok], x[ok]] * (1 - a) + rgb[ok] * a


def map_layer(cam: Camera, points: np.ndarray) -> np.ndarray:
    uv, depth = cam.project(points)
    order = np.argsort(-depth)  # far first
    uv, depth, z = uv[order], depth[order], points[order, 2]
    lo, hi = np.percentile(z, [2, 98])
    t = np.clip((z - lo) / max(hi - lo, 1e-6), 0, 1) * (len(RAMP) - 1)
    i = np.minimum(t.astype(int), len(RAMP) - 2)
    color = RAMP[i] + (RAMP[i + 1] - RAMP[i]) * (t - i)[:, None]
    near = 1 - (depth - depth.min()) / max(np.ptp(depth), 1e-6)
    color *= (0.7 + 0.5 * near)[:, None]
    canvas = np.zeros((H * SS, W * SS, 3), dtype=np.float32)
    canvas[:] = BG
    splat(canvas, uv, color, 0.85, 2)
    return canvas


def to_image(canvas: np.ndarray) -> Image.Image:
    img = Image.fromarray(np.clip(canvas, 0, 255).astype(np.uint8))
    return img.resize((W, H), Image.LANCZOS)


def ease(x: float) -> float:
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


def dashed(draw, pts, color, dash=6, gap=5, width=1):
    """Dashed polyline through screen points."""
    run, on = 0.0, True
    for a, b in zip(pts[:-1], pts[1:]):
        seg = np.hypot(*(b - a))
        pos = 0.0
        while pos < seg:
            step = min((dash if on else gap) - run, seg - pos)
            if on:
                p0 = a + (b - a) * (pos / seg)
                p1 = a + (b - a) * ((pos + step) / seg)
                draw.line([tuple(p0), tuple(p1)], fill=color, width=width)
            pos += step
            run += step
            if run >= (dash if on else gap):
                run, on = 0.0, not on


def bars(draw, title: str, right: str, caption: str):
    draw.rectangle([0, 0, W, TOP], fill=(0, 0, 0))
    draw.rectangle([0, H - BOTTOM, W, H], fill=(0, 0, 0))
    draw.text((10, 8), title, font=font(16, bold=True), fill=TITLE)
    draw.text((W - 10, 9), right, font=font(14), fill=TEXT, anchor="ra")
    draw.text((10, H - BOTTOM + 6), caption, font=font(12), fill=DIM)


def label(draw, xy, text, color=TEXT, size=14, bold=False, anchor="la"):
    f = font(size, bold)
    box = draw.textbbox(xy, text, font=f, anchor=anchor)
    draw.rectangle([box[0] - 4, box[1] - 3, box[2] + 4, box[3] + 3], fill=(0, 0, 0))
    draw.text(xy, text, font=f, fill=color, anchor=anchor)


# ---------------------------------------------------------------- the story


def build(args) -> list:
    est, states, queries = load_run(args.run)
    gt = np.loadtxt(args.gt, ndmin=2)
    t0 = bag_start(args.bag)
    accepted_at = next(
        t for t, s in states if s["reason"] == "global_candidate_accepted"
    )
    used = [q for q in queries if q["query_end_sec"] <= accepted_at + 1e-6]
    first_pose_t = est[0, 0]
    track_times = np.arange(first_pose_t + args.track_step, est[-1, 0], args.track_step)
    scans = load_scans(
        args.bag, [*(q["scan_stamp_sec"] for q in used), first_pose_t, *track_times]
    )

    # View: fitted to the candidates and the path plus a margin; the map
    # (and the search) extends beyond it.
    cands = np.array([[c["x"], c["y"], c["z"]] for q in used for c in q["candidates"]])
    fit = np.vstack([cands, est[:, 1:4]])
    lo, hi = fit.min(axis=0) - args.margin, fit.max(axis=0) + args.margin
    corners = np.array(
        [
            [x, y, z]
            for x in (lo[0], hi[0])
            for y in (lo[1], hi[1])
            for z in (lo[2], hi[2])
        ]
    )
    cam = Camera(corners, args.azimuth, args.elevation)
    map_pts = read_pcd_xyz(args.map)
    keep = np.random.default_rng(1).random(len(map_pts)) < args.map_fraction
    keep &= map_pts[:, 2] < args.map_z_max  # drop the ceiling
    background = map_layer(cam, map_pts[keep])

    # Inset: a query's scan, levelled with the odometry roll and pitch it used.
    box_c, box_r = np.array([W - 104.0, TOP + 106.0]), 86.0

    def in_inset(xyz, q):
        top = q["candidates"][0]
        roll, pitch = math.radians(top["roll_deg"]), math.radians(top["pitch_deg"])
        lev = xyz @ rpy_matrix(roll, pitch, 0.0).T
        return box_c + np.stack([lev[:, 0], -lev[:, 1]], 1) * (box_r / args.inset_range)

    def inset_scan(q):
        uv = in_inset(scans[q["scan_stamp_sec"]][1], q)
        return uv[np.all(np.abs(uv - box_c) < box_r, axis=1)]

    # The scan at the first published pose, in the inset and in the map.
    xyz1 = scans[first_pose_t][1]
    pose1 = pose_at(est, first_pose_t)
    snap_from = in_inset(xyz1, used[-1])
    snap_to = cam.project(xyz1 @ pose1[:3, :3].T + pose1[:3, 3])[0]

    est_uv = cam.project(est[:, 1:4])[0]
    gt_uv = cam.project(gt[:, 1:4])[0]
    gt_segments = np.split(np.arange(len(gt)), np.where(np.diff(gt[:, 0]) > 0.5)[0] + 1)

    def gt_at(t):
        g = int(np.searchsorted(gt[:, 0], t))
        if 0 < g < len(gt) and gt[g, 0] - gt[g - 1, 0] < 0.1:
            return gt[g]
        return None

    def screen(xyz):
        return cam.project(np.asarray(xyz, dtype=float).reshape(1, 3))[0][0]

    frames = []

    def frame(t, phase, sub, overlay=None, scan=None, alpha=1.0, inset=False):
        canvas = background.copy()
        if inset:
            (x0, y0), (x1, y1) = (box_c - box_r) * SS, (box_c + box_r) * SS
            canvas[int(y0) : int(y1), int(x0) : int(x1)] *= 0.2
        if scan is not None:
            splat(canvas, scan, SCAN, alpha, 2)
        img = to_image(canvas)
        draw = ImageDraw.Draw(img)
        if inset:
            (x0, y0), (x1, y1) = box_c - box_r, box_c + box_r
            draw.rectangle([x0, y0, x1, y1], outline=DIM, width=1)
            draw.text((x0 + 5, y0 + 4), "live scan, no pose", font=font(11), fill=DIM)
            c = box_c
            draw.polygon(
                [(c[0] + 8, c[1]), (c[0] - 5, c[1] - 5), (c[0] - 5, c[1] + 5)], fill=EST
            )
        if overlay:
            overlay(draw)
        bars(draw, args.title, f"t = {t - t0:5.1f} s", args.caption)
        label(draw, (12, TOP + 10), phase, color=GOLD, size=15, bold=True)
        if sub:
            label(draw, (12, TOP + 34), sub, size=12)
        frames.append(img)

    # 1: the map, and a scan with no pose.
    for k in range(args.intro_frames):
        frame(
            used[0]["scan_stamp_sec"],
            "1  No initial pose",
            "Only the prior map and the live scan. Where is the robot?",
            scan=inset_scan(used[0]),
            alpha=ease(2 * k / args.intro_frames),
            inset=True,
        )

    # 2: every candidate of every query, until an answer is accepted.
    def search_overlay(qi, shown, ring=None):
        def draw_it(draw):
            for qj, q in enumerate(used[: qi + 1]):
                total = len(q["candidates"])
                # Revealed from the lowest score up, so the best one comes last;
                # a new one starts with a larger ring.
                first = 0 if qj < qi else total - shown
                for rank in reversed(range(first, total)):
                    c = q["candidates"][rank]
                    u, v = screen([c["x"], c["y"], c["z"]])
                    r = 2.5 + 7.0 * max(0.0, (c["score"] - 0.8) / 0.2)
                    if qj == qi and rank - first < 2 and shown < total:
                        r *= 1.8 - 0.4 * (rank - first)
                    color = GOLD if rank == 0 else CAND
                    if qj < qi:
                        color = tuple(int(x * 0.4) for x in color)
                    yaw = math.radians(c["yaw_deg"]) - cam.az
                    tip = (
                        u + 1.8 * r * math.cos(yaw),
                        v - 1.8 * r * math.sin(yaw) * math.sin(cam.el),
                    )
                    draw.line([(u, v), tip], fill=color, width=2)
                    draw.ellipse([u - r, v - r, u + r, v + r], outline=color, width=2)
            q = used[qi]
            truth = gt_at(q["scan_stamp_sec"])
            if truth is not None:
                u, v = screen(truth[1:4])
                draw.line([(u - 5, v - 5), (u + 5, v + 5)], fill=GT_COLOR, width=2)
                draw.line([(u - 5, v + 5), (u + 5, v - 5)], fill=GT_COLOR, width=2)
            lines = [
                f"query {qi + 1}: {q['candidate_count']} candidates in {q['runtime_sec']:.1f} s"
            ]
            if shown == len(q["candidates"]):
                for rank, c in enumerate(q["candidates"][:3]):
                    fitness = c.get("registration_fitness", float("nan"))
                    lines.append(
                        f"#{rank + 1}  score {c['score']:.3f}   NDT fitness {fitness:.3f}"
                    )
            for i, text in enumerate(lines):
                label(
                    draw,
                    (12, TOP + 58 + 19 * i),
                    text,
                    color=GOLD if i == 1 else TEXT,
                    size=12,
                )
            if ring is not None:
                u, v = screen(ring[0])
                rr = 9 + 16 * ring[1]
                draw.ellipse([u - rr, v - rr, u + rr, v + rr], outline=GOLD, width=3)

        return draw_it

    search_sub = (
        "Candidates over the whole map, sized by score (x: motion-capture truth)."
    )
    for qi, q in enumerate(used):
        n = len(q["candidates"])
        for k in range(args.query_frames):
            shown = min(n, int(np.ceil(n * ease(k / (0.6 * args.query_frames)))))
            frame(
                q["scan_stamp_sec"],
                "2  Searching the whole map",
                search_sub,
                search_overlay(qi, shown),
                inset_scan(q),
                inset=True,
            )
    top = used[-1]["candidates"][0]
    for k in range(args.consensus_frames):
        frame(
            accepted_at,
            "2  Two queries agree",
            "The top answers match after the odometry motion: accepted.",
            search_overlay(
                len(used) - 1,
                len(used[-1]["candidates"]),
                ([top["x"], top["y"], top["z"]], 1 - (k % 7) / 7),
            ),
            inset_scan(used[-1]),
            inset=True,
        )

    # 3: the scan moves from the inset onto the map, at the first pose.
    truth = gt_at(first_pose_t)
    sub = f"First pose {first_pose_t - t0:.1f} s into the run"
    if truth is not None:
        sub += f", {np.linalg.norm(pose1[:2, 3] - truth[1:3]):.2f} m from the truth"
    marker = screen(pose1[:3, 3])

    def pose_dot(draw, at=marker):
        draw.ellipse(
            [at[0] - 5, at[1] - 5, at[0] + 5, at[1] + 5], fill=EST, outline=GT_COLOR
        )

    for k in range(args.snap_frames):
        w = ease(k / (args.snap_frames - 1))
        frame(
            first_pose_t,
            "3  Localized",
            sub,
            pose_dot if w == 1 else None,
            snap_from * (1 - w) + snap_to * w,
        )
    for _ in range(args.hold_frames):
        frame(first_pose_t, "3  Localized", sub, pose_dot, snap_to)

    # 4: tracking, with the estimated path and the dashed ground truth.
    for t in track_times:
        stamp, xyz = scans[t]
        pose = pose_at(est, stamp)
        upto = int(np.searchsorted(est[:, 0], stamp))
        truth = gt_at(stamp)
        if truth is None:
            sub = "outside the motion-capture area: no ground truth here"
        else:
            err = np.linalg.norm(pose[:2, 3] - truth[1:3])
            sub = f"position error to the motion-capture truth: {err:.2f} m"

        def overlay(draw, upto=upto, pose=pose, t=t):
            for seg in gt_segments:
                done = seg[(gt[seg, 0] >= first_pose_t) & (gt[seg, 0] <= t)]
                if len(done) > 1:
                    dashed(draw, gt_uv[done][::4], GT_COLOR, width=2)
            if upto > 1:
                draw.line([tuple(p) for p in est_uv[:upto]], fill=EST, width=2)
            pose_dot(draw, screen(pose[:3, 3]))

        scan_uv = cam.project(xyz @ pose[:3, :3].T + pose[:3, 3])[0]
        frame(
            t,
            "4  Tracking (cyan: estimate, dashed: truth)",
            sub,
            overlay,
            scan_uv,
            0.75,
        )
    errors = [
        np.linalg.norm(row[1:3] - truth[1:3])
        for row in est
        for truth in [gt_at(row[0])]
        if truth is not None
    ]
    summary = (
        f"Error where there is motion-capture truth: median {np.median(errors):.2f} m,"
        f" max {np.max(errors):.2f} m"
    )
    for _ in range(args.end_frames):
        frame(
            est[-1, 0],
            "Localized from no pose, then tracked",
            summary,
            overlay,
            scan_uv,
            0.75,
        )
    return frames


def encode(frames, out: Path, colors: int) -> None:
    tmp = Path(tempfile.mkdtemp())
    try:
        for i, img in enumerate(frames):
            img.save(tmp / f"f{i:04d}.png")
        palette = tmp / "palette.png"
        src = ["-framerate", str(FPS), "-i", str(tmp / "f%04d.png")]
        subprocess.run(
            [
                "ffmpeg",
                "-v",
                "error",
                "-y",
                *src,
                "-vf",
                f"palettegen=max_colors={colors}:stats_mode=full",
                str(palette),
            ],
            check=True,
        )
        subprocess.run(
            [
                "ffmpeg",
                "-v",
                "error",
                "-y",
                *src,
                "-i",
                str(palette),
                "-lavfi",
                "paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle",
                "-loop",
                "0",
                str(out),
            ],
            check=True,
        )
    finally:
        shutil.rmtree(tmp)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--map", type=Path, required=True)
    ap.add_argument("--bag", type=Path, required=True)
    ap.add_argument("--run", type=Path, required=True, help="run_benchmark run dir")
    ap.add_argument("--gt", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument(
        "--title", default="lidar_localization_ros2 — global localization on a Go2"
    )
    ap.add_argument("--caption", default="")
    ap.add_argument("--azimuth", type=float, default=-120.0)
    ap.add_argument("--elevation", type=float, default=38.0)
    ap.add_argument("--margin", type=float, default=3.0, help="view margin, m")
    ap.add_argument(
        "--inset-range", type=float, default=8.0, help="inset half width, m"
    )
    ap.add_argument("--map-fraction", type=float, default=0.8)
    ap.add_argument("--map-z-max", type=float, default=2.5, help="ceiling cut, m")
    ap.add_argument("--track-step", type=float, default=2.4, help="run s per frame")
    ap.add_argument("--intro-frames", type=int, default=24)
    ap.add_argument("--query-frames", type=int, default=22)
    ap.add_argument("--consensus-frames", type=int, default=18)
    ap.add_argument("--snap-frames", type=int, default=14)
    ap.add_argument("--hold-frames", type=int, default=14)
    ap.add_argument("--end-frames", type=int, default=24)
    ap.add_argument("--colors", type=int, default=128)
    ap.add_argument("--stills", default="", help="frame numbers to save as PNG")
    args = ap.parse_args(argv)

    frames = build(args)
    for index in filter(None, args.stills.split(",")):
        if int(index) >= len(frames):
            print(f"no frame {index} ({len(frames)} frames)", file=sys.stderr)
            continue
        frames[int(index)].save(args.out.with_name(f"{args.out.stem}_{index}.png"))
    encode(frames, args.out, args.colors)
    print(
        f"{args.out}: {len(frames)} frames, {len(frames) / FPS:.1f} s", file=sys.stderr
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
