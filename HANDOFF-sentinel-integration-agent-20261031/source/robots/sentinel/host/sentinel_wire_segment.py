#!/usr/bin/env python3
"""按 wheel_odom.twist 自动切分 app-*.wire.jsonl，输出各动作段的 main 稳向指标。

只读：不连接设备、不写固件。输入是维护台自动保存的原始 wire 日志。
段划分依据运动量自动识别，不依赖操作员回忆或人工标记。

用法（在包根目录）：
    python source/robots/sentinel/host/sentinel_wire_segment.py \
        results/logs/app-20261108T121953/*.wire.jsonl \
        --output results/logs/app-20261108T121953/segments.json

分段的判据是轮式里程计 twist：
    - 平移活跃: |v_xy| >= --move-threshold (默认 0.05 m/s)
    - 自旋活跃: |wz| >= --spin-threshold (默认 0.5 rad/s)
连续状态少于 --min-seconds 的段并入相邻同类，避免抖动切碎。
"""
from __future__ import annotations

import argparse
import glob
import json
import math
from pathlib import Path
import statistics
import sys

HOST_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(HOST_DIR))
from board_monitor import Decoder  # noqa: E402
import sentinel_client as wire  # noqa: E402

STATUS_TYPE = 0xC6


def unwrap(values):
    """把角度序列解卷绕，避免 ±pi 跳变被当成大跨度。"""
    if not values:
        return []
    out = [values[0]]
    for value in values[1:]:
        delta = value - out[-1]
        while delta > math.pi:
            delta -= 2.0 * math.pi
        while delta < -math.pi:
            delta += 2.0 * math.pi
        out.append(out[-1] + delta)
    return out


def load(path):
    """解析单个 wire 文件，返回 page0 状态序列。非法帧直接跳过并计数。"""
    decoder = Decoder()
    rows, rejected = [], 0
    with open(path, encoding="utf-8") as file:
        for line in file:
            record = json.loads(line)
            if record.get("direction") != "rx":
                continue
            try:
                frames = decoder.feed(bytes.fromhex(record["wire_hex"]))
            except ValueError:
                continue
            for kind, _sequence, payload in frames:
                if kind != STATUS_TYPE:
                    continue
                try:
                    status = wire.decode_status(payload)
                except Exception:
                    rejected += 1
                    continue
                if status.get("page") != 0:
                    continue
                joints = status.get("joints") or {}
                platform = status.get("main_platform") or {}
                twist = (status.get("wheel_odom") or {}).get("twist") or (0.0, 0.0, 0.0)
                rows.append({
                    "t": record["monotonic_s"],
                    "vx": float(twist[0]), "vy": float(twist[1]), "wz": float(twist[2]),
                    "heading": platform.get("heading_rad"),
                    "platform_validity": platform.get("validity"),
                    "main_yaw": (joints.get("main_yaw") or {}).get("angle_rad"),
                    "main_rate": (joints.get("main_yaw") or {}).get("rate_rad_s"),
                    "fire_yaw": (joints.get("fire_yaw") or {}).get("angle_rad"),
                    "pitch": (joints.get("pitch") or {}).get("angle_rad"),
                })
    return rows, rejected


def classify(row, move_threshold, spin_threshold):
    translation = math.hypot(row["vx"], row["vy"])
    spinning = abs(row["wz"]) >= spin_threshold
    moving = translation >= move_threshold
    if spinning and moving:
        return "spin_translate"
    if spinning:
        return "spin"
    if moving:
        return "translate"
    return "static"


def segment(rows, move_threshold, spin_threshold, min_samples):
    """连续同类状态合并成段；过短的段并入前一段。"""
    if not rows:
        return []
    labels = [classify(row, move_threshold, spin_threshold) for row in rows]
    segments, start = [], 0
    for index in range(1, len(rows) + 1):
        if index == len(rows) or labels[index] != labels[start]:
            segments.append({"kind": labels[start], "start_index": start, "end_index": index - 1})
            start = index
    merged = []
    for item in segments:
        count = item["end_index"] - item["start_index"] + 1
        if merged and count < min_samples:
            merged[-1]["end_index"] = item["end_index"]
            continue
        merged.append(item)
    return merged


def summarise(rows, item):
    span = rows[item["start_index"]: item["end_index"] + 1]
    headings = [r["heading"] for r in span if r["heading"] is not None]
    main_angles = [r["main_yaw"] for r in span if r["main_yaw"] is not None]
    rates = [abs(r["main_rate"]) for r in span if r["main_rate"] is not None]
    unwrapped = unwrap(headings)
    valid = [r["platform_validity"] for r in span]
    return {
        "kind": item["kind"],
        "t_start_s": round(span[0]["t"] - rows[0]["t"], 3),
        "t_end_s": round(span[-1]["t"] - rows[0]["t"], 3),
        "samples": len(span),
        "mean_speed_mps": round(statistics.fmean(math.hypot(r["vx"], r["vy"]) for r in span), 4),
        "mean_spin_rad_s": round(statistics.fmean(r["wz"] for r in span), 4),
        "heading_span_deg": round(math.degrees(max(unwrapped) - min(unwrapped)), 3) if unwrapped else None,
        "heading_drift_deg": round(math.degrees(unwrapped[-1] - unwrapped[0]), 3) if unwrapped else None,
        "main_yaw_span_deg": round(math.degrees(max(main_angles) - min(main_angles)), 3) if main_angles else None,
        "main_rate_peak_rad_s": round(max(rates), 4) if rates else None,
        "platform_valid_ratio": round(sum(1 for v in valid if v in ("valid", 1, True)) / len(valid), 4) if valid else None,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("inputs", nargs="+", help="wire.jsonl 路径或 glob")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--move-threshold", type=float, default=0.05)
    parser.add_argument("--spin-threshold", type=float, default=0.5)
    parser.add_argument("--min-seconds", type=float, default=2.0)
    parser.add_argument("--sample-period-s", type=float, default=0.02,
        help="估算最短段样本数用；默认按 10Hz 状态取 0.02（实际采样率更高）")
    args = parser.parse_args()

    paths = []
    for pattern in args.inputs:
        paths.extend(sorted(glob.glob(pattern)) or [pattern])
    if args.output.exists():
        parser.error(f"拒绝覆盖已有输出：{args.output}")

    report = {"schema": "sentinel-wire-segments/1", "sources": [], "segments": []}
    for path in paths:
        rows, rejected = load(path)
        if not rows:
            raise SystemExit(f"{path}: 没有可解析的 page0 状态帧")
        duration = rows[-1]["t"] - rows[0]["t"]
        rate = (len(rows) - 1) / duration if duration > 0 else 0.0
        min_samples = max(5, int(args.min_seconds * rate))
        items = segment(rows, args.move_threshold, args.spin_threshold, min_samples)
        report["sources"].append({
            "file": Path(path).name, "page0_frames": len(rows),
            "rejected_frames": rejected, "duration_s": round(duration, 3),
            "status_rate_hz": round(rate, 2),
        })
        for item in items:
            summary = summarise(rows, item)
            summary["file"] = Path(path).name
            report["segments"].append(summary)
        print(f"{Path(path).name}: {len(rows)} 帧, {duration:.1f}s, 切出 {len(items)} 段")

    report["method"] = {
        "segmentation": "按 wheel_odom.twist 的运动量自动分段，不依赖人工标记",
        "move_threshold_mps": args.move_threshold,
        "spin_threshold_rad_s": args.spin_threshold,
        "min_segment_s": args.min_seconds,
        "heading_drift_note": "漂移是末值减首值，含解卷绕；不是绝对真值误差",
        "limitation": "姿态估计无独立光学真值；航向跨度包含估计器误差，不能单独判为机械摆动",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as file:
        json.dump(report, file, ensure_ascii=False, indent=2)
    print(f"写入 {args.output}")


if __name__ == "__main__":
    main()
