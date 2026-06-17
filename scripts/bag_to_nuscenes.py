#!/usr/bin/env python3
"""
Convert a recorded rosbag2 (.db3) of /point_cloud + /ground_truth/agents into a
nuScenes-style dataset (subset of tables).

Reads the bag directly via sqlite3 and deserializes messages with rclpy, so it only
needs a sourced ROS 2 environment (Humble) — no extra pip packages beyond numpy.

Design decisions (see README "Dataset format"):
  * Every frame is a keyframe — labelling is free in sim, so each sample_data is a sample.
  * One episode  -> one scene.
  * One GT frame -> one sample + one ego_pose + one sample_data (.pcd.bin).
  * One agent/vehicle in a frame -> one sample_annotation.
  * Object identity (instance) keyed by (episode, kind, GetInstanceID).
  * Frames: annotations + ego_pose are GLOBAL (nuScenes convention). Point clouds are
    written in the SENSOR frame so the nuScenes chain sensor->ego->global reconstructs
    global exactly; calibrated_sensor is identity (LiDAR coincident with ego).
  * num_lidar_pts is computed in the global frame by point-in-box test.

Usage:
    source /opt/ros/humble/setup.bash
    python3 bag_to_nuscenes.py \
        --bag ~/Unity_Lidar_Sim/bags/sweep_s1_XXXX \
        --out ~/Unity_Lidar_Sim/nuscenes_out \
        --version v1.0-mini
"""

import argparse
import glob
import hashlib
import json
import math
import os
import sqlite3
import sys

import numpy as np


# ─────────────────────────────────────────────────────────────────────────────
# Bag reading
# ─────────────────────────────────────────────────────────────────────────────

def find_db3(bag_path):
    """Accept either a bag directory or a direct .db3 path."""
    if os.path.isfile(bag_path) and bag_path.endswith(".db3"):
        return bag_path
    matches = sorted(glob.glob(os.path.join(bag_path, "*.db3")))
    if not matches:
        sys.exit(f"[error] no .db3 found under {bag_path}")
    return matches[0]


def read_bag(db3_path, lidar_topic, gt_topic):
    """Return (lidar_msgs, gt_frames).

    lidar_msgs : list of (t_sec: float, points: np.ndarray[N,4])  # x,y,z,intensity
    gt_frames  : list of dict (parsed JSON, with 't_sec' added)
    """
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String

    conn = sqlite3.connect(db3_path)
    cur = conn.cursor()

    topics = {name: (tid, typ) for tid, name, typ in
              cur.execute("SELECT id, name, type FROM topics")}
    if lidar_topic not in topics:
        sys.exit(f"[error] lidar topic '{lidar_topic}' not in bag. Have: {list(topics)}")
    if gt_topic not in topics:
        sys.exit(f"[error] gt topic '{gt_topic}' not in bag. Have: {list(topics)}")

    lidar_id = topics[lidar_topic][0]
    gt_id = topics[gt_topic][0]

    lidar_msgs, gt_frames = [], []

    for topic_id, _bag_t, data in cur.execute(
            "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp"):
        if topic_id == lidar_id:
            msg = deserialize_message(bytes(data), PointCloud2)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            lidar_msgs.append((t, pointcloud2_to_xyzi(msg)))
        elif topic_id == gt_id:
            msg = deserialize_message(bytes(data), String)
            try:
                frame = json.loads(msg.data)
            except json.JSONDecodeError as e:
                lo = max(0, e.pos - 40)
                hi = min(len(msg.data), e.pos + 40)
                print("[error] malformed ground-truth JSON in bag:")
                print(f"        {e}")
                print(f"        ...{msg.data[lo:hi]!r}...")
                print(f"        {' ' * (3 + (e.pos - lo))}^ here")
                sys.exit("Re-record after fixing the publisher (see diagnosis below).")
            frame["t_sec"] = frame["stamp_sec"] + frame["stamp_nsec"] * 1e-9
            gt_frames.append(frame)

    conn.close()
    print(f"[read] {len(lidar_msgs)} point clouds, {len(gt_frames)} ground-truth frames")
    return lidar_msgs, gt_frames


def pointcloud2_to_xyzi(msg):
    """Extract an [N,4] float32 array (x, y, z, intensity) from a PointCloud2."""
    offsets = {f.name: f.offset for f in msg.fields}
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    raw = raw.reshape(-1, msg.point_step)

    def col(name, default=0.0):
        if name not in offsets:
            return np.full(raw.shape[0], default, dtype=np.float32)
        o = offsets[name]
        return raw[:, o:o + 4].copy().view(np.float32).reshape(-1)

    x = col("x"); y = col("y"); z = col("z"); i = col("intensity")
    return np.stack([x, y, z, i], axis=1).astype(np.float32)


# ─────────────────────────────────────────────────────────────────────────────
# Geometry
# ─────────────────────────────────────────────────────────────────────────────

def yaw_to_quat(yaw):
    """Quaternion [w, x, y, z] for a rotation about the global z (up) axis."""
    return [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]


def quat_to_matrix(w, x, y, z):
    """3x3 rotation matrix from quaternion [w,x,y,z]."""
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def ego_from_frame(frame):
    """Return (t_ego[3], R_ego[3,3]) from the frame's ego block, identity if missing."""
    ego = frame.get("ego")
    if not ego:
        return np.zeros(3), np.eye(3)
    t = np.array([ego["tx"], ego["ty"], ego["tz"]], dtype=np.float64)
    R = quat_to_matrix(ego["qw"], ego["qx"], ego["qy"], ego["qz"])
    return t, R


def count_points_in_box(pts_global, center, size_wlh, yaw):
    """Count global points inside an oriented box (yaw about z)."""
    if pts_global.shape[0] == 0:
        return 0
    w, l, h = size_wlh
    d = pts_global[:, :3] - np.asarray(center)
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx = c * d[:, 0] - s * d[:, 1]   # along length (forward / global x)
    ly = s * d[:, 0] + c * d[:, 1]   # along width  (left-right / global y)
    lz = d[:, 2]
    inside = (np.abs(lx) <= l / 2) & (np.abs(ly) <= w / 2) & (np.abs(lz) <= h / 2)
    return int(np.count_nonzero(inside))


# ─────────────────────────────────────────────────────────────────────────────
# Token helper
# ─────────────────────────────────────────────────────────────────────────────

def token(*parts):
    """Deterministic 32-char hex token from a stable key (reproducible across runs)."""
    return hashlib.md5(":".join(str(p) for p in parts).encode()).hexdigest()


# ─────────────────────────────────────────────────────────────────────────────
# Conversion
# ─────────────────────────────────────────────────────────────────────────────

# Map object state -> nuScenes attribute name.
STATE_TO_ATTR = {
    "Patrolling": "pedestrian.moving",
    "Wandering":  "pedestrian.moving",
    "Reacting":   "pedestrian.moving",
    "Paused":     "pedestrian.standing",
    "Crossing":   "pedestrian.moving",
    "Moving":     "vehicle.moving",
    "Unknown":    "pedestrian.standing",
}

# Map object type -> nuScenes category name.
TYPE_TO_CATEGORY = {
    "Pedestrian": "human.pedestrian.adult",
    "Animal":     "animal",
    "Vehicle":    "vehicle.emergency.ambulance",
}

VISIBILITY_BUCKETS = [  # (token level, min lidar pts)
    ("1", 0), ("2", 20), ("3", 80), ("4", 200),
]


def visibility_token(num_pts):
    level = "1"
    for lvl, lo in VISIBILITY_BUCKETS:
        if num_pts >= lo:
            level = lvl
    return level  # nuScenes visibility tokens are the level strings "1".."4"


def pair_lidar(gt_frames, lidar_msgs, max_dt=0.05):
    """For each GT frame, attach the nearest point cloud (by timestamp)."""
    lidar_t = np.array([t for t, _ in lidar_msgs])
    paired = []
    for frame in gt_frames:
        if len(lidar_t) == 0:
            break
        idx = int(np.argmin(np.abs(lidar_t - frame["t_sec"])))
        dt = abs(lidar_t[idx] - frame["t_sec"])
        if dt > max_dt:
            print(f"[warn] frame ep{frame['episode']} f{frame['frame']}: "
                  f"nearest cloud is {dt*1e3:.0f} ms away — skipping")
            continue
        paired.append((frame, lidar_msgs[idx][1]))
    print(f"[pair] {len(paired)}/{len(gt_frames)} frames paired with a cloud")
    return paired


def convert(paired, out_dir, version):
    tables = {name: [] for name in [
        "scene", "sample", "sample_data", "sample_annotation", "instance",
        "category", "attribute", "visibility", "ego_pose",
        "calibrated_sensor", "sensor", "log",
    ]}

    bin_dir = os.path.join(out_dir, "samples", "LIDAR_TOP")
    os.makedirs(bin_dir, exist_ok=True)

    # ── Static tables ────────────────────────────────────────────────────────
    sensor_tok = token("sensor", "LIDAR_TOP")
    tables["sensor"].append({
        "token": sensor_tok, "channel": "LIDAR_TOP", "modality": "lidar"})

    calib_tok = token("calib", "LIDAR_TOP")
    tables["calibrated_sensor"].append({
        "token": calib_tok, "sensor_token": sensor_tok,
        "translation": [0.0, 0.0, 0.0], "rotation": [1.0, 0.0, 0.0, 0.0],
        "camera_intrinsic": []})

    for name, idx in [(c, i) for i, c in enumerate(sorted(set(TYPE_TO_CATEGORY.values())))]:
        tables["category"].append({
            "token": token("category", name), "name": name,
            "description": name, "index": idx})

    for name in sorted(set(STATE_TO_ATTR.values())):
        tables["attribute"].append({
            "token": token("attribute", name), "name": name, "description": name})

    for lvl, lo in VISIBILITY_BUCKETS:
        tables["visibility"].append({
            "token": lvl, "level": f"v{lvl}",
            "description": f"approx >= {lo} lidar points"})

    logs_seen = {}

    # ── Group frames by episode (one scene each) ─────────────────────────────
    episodes = {}
    for frame, pts in paired:
        episodes.setdefault(frame["episode"], []).append((frame, pts))

    # Track linked-list state per instance: last annotation token.
    instance_anns = {}   # instance_token -> list of annotation tokens (in order)
    instance_cat = {}    # instance_token -> category_token

    for episode, frames in sorted(episodes.items()):
        frames.sort(key=lambda fp: fp[0]["frame"])
        first = frames[0][0]
        config = first.get("config", "unknown")

        # One log per config (vehicle = airplane sim).
        if config not in logs_seen:
            log_tok = token("log", config)
            logs_seen[config] = log_tok
            tables["log"].append({
                "token": log_tok, "logfile": "", "vehicle": "airplane_sim",
                "date_captured": "2026-01-01", "location": config})
        log_tok = logs_seen[config]

        scene_tok = token("scene", episode)
        sample_toks = []

        for frame, pts in frames:
            t_us = int(frame["stamp_sec"] * 1_000_000 + frame["stamp_nsec"] / 1000)
            sample_tok = token("sample", episode, frame["frame"])
            sample_toks.append(sample_tok)

            # ego_pose (global)
            t_ego, R_ego = ego_from_frame(frame)
            ego = frame.get("ego")
            ego_tok = token("ego", episode, frame["frame"])
            tables["ego_pose"].append({
                "token": ego_tok, "timestamp": t_us,
                "translation": [float(t_ego[0]), float(t_ego[1]), float(t_ego[2])],
                "rotation": ([ego["qw"], ego["qx"], ego["qy"], ego["qz"]]
                             if ego else [1.0, 0.0, 0.0, 0.0]),
            })

            # sample
            tables["sample"].append({
                "token": sample_tok, "timestamp": t_us,
                "scene_token": scene_tok, "next": "", "prev": ""})

            # Point cloud: global -> sensor (calib identity), write .pcd.bin
            xyz_global = pts[:, :3].astype(np.float64)
            xyz_sensor = (xyz_global - t_ego) @ R_ego   # R_ego^T applied on the right
            out_pts = np.hstack([xyz_sensor, pts[:, 3:4]]).astype(np.float32)

            fname = f"{episode:04d}_{frame['frame']:05d}.pcd.bin"
            out_pts.tofile(os.path.join(bin_dir, fname))

            sd_tok = token("sample_data", episode, frame["frame"])
            tables["sample_data"].append({
                "token": sd_tok, "sample_token": sample_tok,
                "ego_pose_token": ego_tok, "calibrated_sensor_token": calib_tok,
                "filename": f"samples/LIDAR_TOP/{fname}", "fileformat": "pcd",
                "is_key_frame": True, "height": 0, "width": 0,
                "timestamp": t_us, "next": "", "prev": ""})

            # Annotations (agents + vehicles)
            objects = ([("agent", o) for o in frame.get("agents", [])] +
                       [("vehicle", o) for o in frame.get("vehicles", [])])
            for kind, obj in objects:
                inst_tok = token("instance", episode, kind, obj["id"])
                cat_name = TYPE_TO_CATEGORY.get(obj["type"], "movable_object")
                instance_cat[inst_tok] = token("category", cat_name)

                bb = obj["bbox"]
                center = [bb["cx"], bb["cy"], bb["cz"]]
                size_wlh = [bb["sy"], bb["sx"], bb["sz"]]  # [width, length, height]
                npts = count_points_in_box(pts, center, size_wlh, obj["yaw"])

                attr = STATE_TO_ATTR.get(obj.get("state", "Unknown"), "pedestrian.standing")
                ann_tok = token("ann", episode, frame["frame"], kind, obj["id"])
                ann_row = {
                    "token": ann_tok, "sample_token": sample_tok,
                    "instance_token": inst_tok,
                    "visibility_token": visibility_token(npts),
                    "attribute_tokens": [token("attribute", attr)],
                    "translation": center, "size": size_wlh,
                    "rotation": yaw_to_quat(obj["yaw"]),
                    "num_lidar_pts": npts, "num_radar_pts": 0,
                    "next": "", "prev": ""}
                tables["sample_annotation"].append(ann_row)
                ann_index[ann_tok] = ann_row
                instance_anns.setdefault(inst_tok, []).append(ann_tok)

        # scene row
        tables["scene"].append({
            "token": scene_tok, "name": f"scene-{episode:04d}",
            "description": f"{config} seed={first.get('seed')}",
            "log_token": log_tok, "nbr_samples": len(sample_toks),
            "first_sample_token": sample_toks[0],
            "last_sample_token": sample_toks[-1]})

    # ── Link next/prev ────────────────────────────────────────────────────────
    link_linked_list(tables["sample"], key=lambda r: r["token"],
                     group=lambda r: r["scene_token"], order=lambda r: r["timestamp"])
    link_linked_list(tables["sample_data"], key=lambda r: r["token"],
                     group=lambda r: r["sample_token"], order=lambda r: r["timestamp"])

    # sample_annotation next/prev chain within each instance
    for inst_tok, ann_toks in instance_anns.items():
        for i, atok in enumerate(ann_toks):
            row = ann_index[atok]
            row["prev"] = ann_toks[i - 1] if i > 0 else ""
            row["next"] = ann_toks[i + 1] if i < len(ann_toks) - 1 else ""

    # instance rows
    for inst_tok, ann_toks in instance_anns.items():
        tables["instance"].append({
            "token": inst_tok, "category_token": instance_cat[inst_tok],
            "nbr_annotations": len(ann_toks),
            "first_annotation_token": ann_toks[0],
            "last_annotation_token": ann_toks[-1]})

    # ── Write tables ──────────────────────────────────────────────────────────
    meta_dir = os.path.join(out_dir, version)
    os.makedirs(meta_dir, exist_ok=True)
    for name, rows in tables.items():
        with open(os.path.join(meta_dir, f"{name}.json"), "w") as fh:
            json.dump(rows, fh, indent=2)
        print(f"[write] {name:20s} {len(rows):6d} rows")
    print(f"\n[done] dataset written to {out_dir}  (version {version})")


# Built lazily so the annotation-linking pass can index by token.
ann_index = {}


def link_linked_list(rows, key, group, order):
    """Fill next/prev within each group, ordered by `order`."""
    from collections import defaultdict
    groups = defaultdict(list)
    for r in rows:
        groups[group(r)].append(r)
    for grp in groups.values():
        grp.sort(key=order)
        for i, r in enumerate(grp):
            r["prev"] = key(grp[i - 1]) if i > 0 else ""
            r["next"] = key(grp[i + 1]) if i < len(grp) - 1 else ""


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True, help="bag directory or .db3 file")
    ap.add_argument("--out", required=True, help="output dataset root")
    ap.add_argument("--version", default="v1.0-mini", help="metadata folder name")
    ap.add_argument("--lidar-topic", default="/point_cloud")
    ap.add_argument("--gt-topic", default="/ground_truth/agents")
    ap.add_argument("--max-dt", type=float, default=0.05,
                    help="max seconds between a GT frame and its paired cloud")
    args = ap.parse_args()

    db3 = find_db3(args.bag)
    print(f"[bag] {db3}")
    lidar_msgs, gt_frames = read_bag(db3, args.lidar_topic, args.gt_topic)
    if not gt_frames:
        sys.exit("[error] no ground-truth frames found")

    paired = pair_lidar(gt_frames, lidar_msgs, max_dt=args.max_dt)
    if not paired:
        sys.exit("[error] no frames paired — check topics / timestamps")

    convert(paired, args.out, args.version)


if __name__ == "__main__":
    main()
