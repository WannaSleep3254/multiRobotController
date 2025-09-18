#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AddressMap.json conflict validator

Validates a Modbus address map JSON against these rules:
- No overlaps within each address space (coils, discrete_inputs, holding, input_registers).
- "reserved" ranges don't overlap each other and don't overlap actual assignments.
- Known *_BASE keys expand to multi-word ranges (e.g., POSE bases expand to 12 words by default).
- TARGET_QUEUE_BASE expands using TARGET_QUEUE_STRIDE and --max-queue (optional).
- Reports gaps, overlaps, and a concise summary per space.

Usage:
  python validate_address_map.py AddressMap.json [--max-queue N] [--strict]

Exit codes:
  0: OK (no conflicts)
  2: Errors found
"""

import argparse
import json
import sys
from typing import Dict, List, Tuple, Optional

Range = Tuple[int, int, str]  # (start, end, name)

def die(msg: str) -> None:
    print(f"[ERROR] {msg}", file=sys.stderr)
    sys.exit(2)

def warn(msg: str) -> None:
    print(f"[WARN] {msg}", file=sys.stderr)

def info(msg: str) -> None:
    print(f"[INFO] {msg}")

def expand_word_range(start: int, words: int) -> Tuple[int, int]:
    if words is None or words <= 0:
        return (start, start)
    return (start, start + words - 1)

def get_pose_words(meta: Dict) -> int:
    axes = meta.get("pose_axes", 6)
    words_per_axis = meta.get("float32_words_per_axis", 2)
    return axes * words_per_axis

def infer_words_for_key(space: str, key: str, meta: Dict, full_map: Dict) -> Optional[int]:
    """Return number of words for a key if it represents a range; None/1 means single word."""
    # Pose bases: 6 axes * 2 words (default 12)
    if key.endswith("_BASE") and "POSE" in key:
        return get_pose_words(meta)
    # Tick bases: u32 = 2 words
    if key.endswith("TICK_BASE"):
        return 2
    # TARGET_QUEUE has variable length; handled separately in add_entries_for_space
    if key == "TARGET_QUEUE_BASE":
        return None  # special handling
    # Default: single word
    return 1

def add_entries_for_space(space: str, mapping: Dict, meta: Dict, full_map: Dict,
                          max_queue: int, out: List[Range]) -> None:
    """Convert symbolic entries to concrete (start,end,name) ranges."""
    if space in ("coils", "discrete_inputs"):
        for k, v in mapping.items():
            if not isinstance(v, int):
                die(f"{space}.{k} must be an integer address")
            out.append((v, v, k))
        return

    # holding / input_registers
    for k, v in mapping.items():
        if not isinstance(v, int):
            # permit helpers to be non-ints (e.g., TARGET_QUEUE_STRIDE)
            if k in ("TARGET_QUEUE_STRIDE",):
                continue
            die(f"{space}.{k} must be an integer address (got {type(v).__name__})")

        if k == "TARGET_QUEUE_STRIDE":
            continue

        words = infer_words_for_key(space, k, meta, full_map)

        if k == "TARGET_QUEUE_BASE":
            stride = mapping.get("TARGET_QUEUE_STRIDE", get_pose_words(meta))
            if not isinstance(stride, int) or stride <= 0:
                die(f"{space}.TARGET_QUEUE_STRIDE must be positive integer (got {stride!r})")
            if max_queue > 0:
                start = v
                end = v + stride * max_queue - 1
                out.append((start, end, f"{k}[0..{max_queue-1}] (stride {stride})"))
            else:
                # Only register the base as a single address to still detect if BASE itself collides
                out.append((v, v, f"{k} (open-ended; use --max-queue to expand)"))
            continue

        if words is None or words == 1:
            out.append((v, v, k))
        else:
            start, end = expand_word_range(v, words)
            out.append((start, end, f"{k} ({words} words)"))

def normalize_reserved(reserved_list):
    out = []
    for item in reserved_list:
        start = item.get("start", None)
        end = item.get("end", None)
        purpose = item.get("purpose", "")
        if start is None or end is None:
            die(f"reserved item missing 'start' or 'end': {item}")
        if (not isinstance(start, int)) or (not isinstance(end, int)):
            die(f"reserved 'start'/'end' must be ints: {item}")
        if end < start:
            die(f"reserved range end<start: {item}")
        out.append((start, end, purpose))
    return out

def find_overlaps(ranges: List[Range]):
    """Return list of overlapping pairs."""
    if not ranges:
        return []
    rs = sorted(ranges, key=lambda r: (r[0], r[1]))
    overlaps = []
    prev = rs[0]
    for cur in rs[1:]:
        if cur[0] <= prev[1]:
            overlaps.append((prev, cur))
            if cur[1] > prev[1]:
                prev = (prev[0], cur[1], prev[2])
        else:
            prev = cur
    return overlaps

def summarize(ranges: List[Range]):
    if not ranges:
        return (None, None)
    s = min(r[0] for r in ranges)
    e = max(r[1] for r in ranges)
    return s, e

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("json_path", help="Path to AddressMap.json")
    parser.add_argument("--max-queue", type=int, default=0,
                        help="Expand TARGET_QUEUE_BASE for N targets (default: 0=do not expand)")
    parser.add_argument("--strict", action="store_true",
                        help="Treat warnings as errors")
    args = parser.parse_args()

    try:
        with open(args.json_path, "r", encoding="utf-8") as f:
            m = json.load(f)
    except Exception as e:
        die(f"Failed to load JSON: {e}")

    meta = m.get("meta", {})
    reserved = m.get("reserved", {})

    spaces = ("coils", "discrete_inputs", "holding", "input_registers")
    used = {s: [] for s in spaces}
    resv = {s: [] for s in spaces}

    # Build used ranges
    for s in spaces:
        mapping = m.get(s, {})
        if not isinstance(mapping, dict):
            die(f"'{s}' must be an object")
        add_entries_for_space(s, mapping, meta, m, args.max_queue, used[s])

    # Build reserved ranges
    for s in spaces:
        lst = reserved.get(s, [])
        if lst:
            if not isinstance(lst, list):
                die(f"reserved.{s} must be an array")
            resv[s] = normalize_reserved(lst)

    # Validate: overlaps within used
    errors = 0
    for s in spaces:
        overlaps = find_overlaps(used[s])
        if overlaps:
            errors += 1
            print(f"\n[CONFLICT] Overlaps in {s}:")
            for a,b in overlaps:
                print(f"  {s}: {a}  <-->  {b}")

    # Validate: overlaps within reserved
    for s in spaces:
        overlaps = find_overlaps([(a,b,f\"reserved:{p}\") for (a,b,p) in resv[s]])
        if overlaps:
            errors += 1
            print(f\"\n[CONFLICT] Reserved ranges overlap in {s}:")
            for a,b in overlaps:
                print(f\"  {s}: {a}  <-->  {b}\")

    # Validate: used ∩ reserved
    for s in spaces:
        if not resv[s]:
            continue
        for u in used[s]:
            for (a,b,p) in resv[s]:
                if not (u[1] < a or u[0] > b):
                    errors += 1
                    print(f\"\n[CONFLICT] {s} used range {u} overlaps reserved {a}-{b} ({p})")

    # Summary
    print("\n=== SUMMARY ===")
    for s in spaces:
        s_used = used[s]
        s_resv = resv[s]
        smin, smax = summarize(s_used) if s_used else (None, None)
        print(f"- {s}: {len(s_used)} used range(s){' from '+str(smin)+' to '+str(smax) if smin is not None else ''}"
              f" | {len(s_resv)} reserved range(s)")

    # Warning about TARGET_QUEUE without --max-queue
    hold = m.get("holding", {})
    if "TARGET_QUEUE_BASE" in hold and args.max_queue == 0:
        msg = "holding.TARGET_QUEUE_BASE is open-ended; pass --max-queue N to expand its range for conflict checking."
        if args.strict:
            die(msg)
        else:
            warn(msg)

    if errors:
        die(f"{errors} conflict group(s) found.")
    else:
        info("No conflicts found. AddressMap looks good.")

if __name__ == "__main__":
    main()
