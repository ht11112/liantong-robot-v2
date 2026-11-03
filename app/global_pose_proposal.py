"""Global saved-map pose proposal from three front180 scan snapshots.

No ROS, device, TF publication, seed publication or motion permission exists in
this module. NumPy is imported only when the pure calculation is called. The
CLI may select a separate NumPy directory without installing anything globally.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys


SAVED_MAP_ID = "9e34ebc9f5a36fbbf64e3a6f7bbef4d23683cd97de1963bba00fe9f60507c746"


def finite(value, name):
    if type(value) not in (int, float):
        raise ValueError(name + " must be a finite built-in number")
    try:
        number = float(value)
    except (OverflowError, ValueError):
        raise ValueError(name + " must be finite") from None
    if not math.isfinite(number):
        raise ValueError(name + " must be finite")
    return number


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def map_identity(info, cells):
    """The historical map_saver normalized identity, after strict validation."""
    if type(info) is not dict or type(cells) not in (list, tuple):
        raise ValueError("Map info/data required")
    width, height = info.get("width"), info.get("height")
    if (type(width) is not int or type(height) is not int or width <= 0 or height <= 0
            or width * height > 4000000 or len(cells) != width * height):
        raise ValueError("Invalid map dimensions/data length")
    resolution = finite(info.get("resolution"), "map resolution")
    origin = info.get("origin")
    if resolution <= 0 or type(origin) not in (list, tuple) or len(origin) != 3:
        raise ValueError("Positive map resolution and planar origin required")
    origin = [finite(v, "map origin") for v in origin]
    if any(type(v) is not int or not -1 <= v <= 100 for v in cells):
        raise ValueError("Map cells must be exact integers in -1..100")
    normalized = [0 if 0 <= v <= 25 else 100 if v >= 65 else -1 for v in cells]
    stable = dict(width=width, height=height, resolution=round(resolution, 6),
                  origin=[round(v, 6) for v in origin], cells=normalized)
    return hashlib.sha256(json.dumps(stable, sort_keys=True, separators=(",", ":"),
                                      allow_nan=False).encode()).hexdigest()


def pgm_cells(raw):
    """Parse this saved map's strict P5/255 pixels, preserving first whitespace pixel."""
    if type(raw) is not bytes:
        raise ValueError("PGM must be exact bytes")
    offset, tokens = 0, []
    while len(tokens) < 4:
        while offset < len(raw) and raw[offset:offset + 1] in b" \r\n\t":
            offset += 1
        if raw[offset:offset + 1] == b"#":
            end = raw.find(b"\n", offset)
            if end < 0:
                raise ValueError("Incomplete PGM comment")
            offset = end + 1
            continue
        start = offset
        while offset < len(raw) and raw[offset:offset + 1] not in b" \r\n\t":
            offset += 1
        if offset == start:
            raise ValueError("Incomplete PGM header")
        tokens.append(raw[start:offset])
    if tokens[0] != b"P5" or tokens[3] != b"255":
        raise ValueError("Expected an 8-bit P5/255 saved map")
    width, height = int(tokens[1]), int(tokens[2])
    if width <= 0 or height <= 0 or width * height > 4000000:
        raise ValueError("Invalid PGM dimensions")
    if raw[offset:offset + 2] == b"\r\n":
        offset += 2
    elif raw[offset:offset + 1] in (b" ", b"\n", b"\r", b"\t"):
        offset += 1
    else:
        raise ValueError("Missing PGM pixel separator")
    pixels = raw[offset:]
    if len(pixels) != width * height:
        raise ValueError("PGM pixel count differs")
    cells = []
    for y in range(height - 1, -1, -1):
        cells.extend(0 if p == 254 else 100 if p == 0 else -1
                     for p in pixels[y * width:(y + 1) * width])
    return width, height, cells


def verified_map(map_doc, pgm_bytes, expected_map_id):
    if type(map_doc) is not dict:
        raise ValueError("Map JSON object required")
    info, cells = map_doc.get("info"), map_doc.get("data")
    current_id = map_identity(info, cells)
    if (type(expected_map_id) is not str or len(expected_map_id) != 64
            or any(c not in "0123456789abcdef" for c in expected_map_id)
            or current_id != expected_map_id):
        raise ValueError("Map JSON identity does not equal the expected saved map")
    if "map_id" in map_doc and map_doc["map_id"] != current_id:
        raise ValueError("Map JSON declared identity differs")
    width, height, pgm_data = pgm_cells(pgm_bytes)
    if (width != info["width"] or height != info["height"]
            or map_identity(info, pgm_data) != current_id):
        raise ValueError("PGM and map JSON normalized identities differ")
    # Search the canonical saved grid, so intermediate occupancy values match
    # the PGM/map_server unknown interpretation rather than inventing known area.
    normalized = [0 if 0 <= v <= 25 else 100 if v >= 65 else -1 for v in cells]
    return info, normalized, dict(map_id=current_id, pgm_sha256=hashlib.sha256(pgm_bytes).hexdigest(),
                            pgm_and_json_grid_agree=True)


def scan_points(scan, extrinsics, max_points):
    """Retain source identity; crop rear geometrically before selecting endpoints."""
    import numpy as np
    if type(scan) is not dict or type(scan.get("ranges")) not in (list, tuple):
        raise ValueError("Scan ranges object required")
    header = scan.get("header") or {}
    if type(header) is not dict:
        raise ValueError("Scan header object required")
    seconds, nanos = header.get("stamp_secs"), header.get("stamp_nsecs")
    if seconds is not None or nanos is not None:
        if (type(seconds) is not int or seconds < 0 or type(nanos) is not int
                or not 0 <= nanos < 1000000000):
            raise ValueError("Invalid exact scan source stamp")
        stamp = seconds + nanos / 1e9
        stamp_key = (seconds, nanos)
    else:
        stamp = scan.get("stamp_s", scan.get("source_stamp", header.get("source_stamp")))
        stamp = finite(stamp, "scan source stamp")
        stamp_key = divmod(int(round(stamp * 1000000000)), 1000000000)
    if not math.isfinite(stamp) or stamp <= 0:
        raise ValueError("Positive scan source stamp required")
    for supplied in (scan.get("stamp_s"), scan.get("source_stamp"), header.get("source_stamp")):
        if supplied is not None and abs(finite(supplied, "scan source stamp") - stamp) > 1e-6:
            raise ValueError("Scan source stamp fields disagree")
    if scan.get("navigation_fov_deg", 180) != 180:
        raise ValueError("Front180 snapshot required")
    amin, inc = (finite(scan.get(k), k) for k in ("angle_min", "angle_increment"))
    rmin, rmax = (finite(scan.get(k), k) for k in ("range_min", "range_max"))
    if inc == 0 or not 0 <= rmin < rmax or not 20 <= len(scan["ranges"]) <= 10000:
        raise ValueError("Invalid scan geometry")
    x, y, yaw = extrinsics
    points = []
    for index, raw in enumerate(scan["ranges"]):
        if raw is None or type(raw) is str and raw.lower() in ("inf", "+inf", "-inf", "nan"):
            continue
        if type(raw) not in (int, float):
            raise ValueError("Invalid scan range value")
        try:
            distance = float(raw)
        except OverflowError:
            raise ValueError("Scan range outside numeric range") from None
        beam = angle(amin + index * inc + yaw)
        if (not math.isfinite(distance) or abs(beam) > math.pi / 2 + 1e-12
                or not max(.3, rmin) <= distance <= min(8., rmax)):
            continue
        points.append((x + distance * math.cos(beam), y + distance * math.sin(beam)))
    if len(points) < 60:
        raise ValueError("At least 60 valid front180 endpoints per scan required")
    if len(points) > max_points:
        indices = np.linspace(0, len(points) - 1, max_points).astype(int)
        points = [points[int(i)] for i in indices]
    return np.asarray(points, dtype=float), dict(source_stamp=stamp, exact_source_key=stamp_key,
        original_header=header, raw_header=scan.get("raw_header"),
        frame_id=header.get("frame_id", scan.get("frame")), used=len(points))


def distance_field(occupied, resolution, limit=.4):
    import numpy as np
    height, width = occupied.shape
    field = np.full(occupied.shape, limit, dtype=float)
    radius = int(math.ceil(limit / resolution))
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            distance = math.hypot(dx, dy) * resolution
            if distance > limit:
                continue
            sy = slice(max(0, -dy), min(height, height - dy))
            sx = slice(max(0, -dx), min(width, width - dx))
            ty = slice(max(0, dy), min(height, height + dy))
            tx = slice(max(0, dx), min(width, width + dx))
            target, mask = field[ty, tx], occupied[sy, sx]
            target[mask] = np.minimum(target[mask], distance)
    return field


def endpoint_agreement(info, cells, points, pose, tolerance_m=.10):
    """Independent endpoint check; an unknown endpoint cell is never a match."""
    x, y, yaw = (finite(v, "pose") for v in pose)
    ox, oy, oa = info["origin"]
    resolution, width, height = info["resolution"], info["width"], info["height"]
    ca, sa, c, s = math.cos(oa), math.sin(oa), math.cos(yaw), math.sin(yaw)
    radius = int(math.ceil(tolerance_m / resolution))
    matched = 0
    for bx, by in points:
        wx, wy = x + c * bx - s * by - ox, y + s * bx + c * by - oy
        gx, gy = int(math.floor((ca * wx + sa * wy) / resolution)), int(math.floor((-sa * wx + ca * wy) / resolution))
        if not (0 <= gx < width and 0 <= gy < height):
            continue
        cell = cells[gy * width + gx]
        if cell < 0 or 25 < cell < 65:
            continue
        if any(0 <= gx + dx < width and 0 <= gy + dy < height
               and cells[(gy + dy) * width + gx + dx] >= 65
               for dy in range(-radius, radius + 1) for dx in range(-radius, radius + 1)
               if math.hypot(dx, dy) * resolution <= tolerance_m + 1e-12):
            matched += 1
    return dict(used=len(points), matched=matched, near_wall_ratio=matched / len(points) if len(points) else 0.,
                tolerance_m=tolerance_m)


def global_pose_proposal(map_doc, pgm_bytes, scans, expected_map_id=SAVED_MAP_ID, *,
                         extrinsics=(.13, 0., 0.), coarse_step_m=.10,
                         coarse_yaw_deg=5., fine_step_m=.025, fine_yaw_deg=.5,
                         refine_radius_m=.15, refine_yaw_deg=6., mode_count=5,
                         batch_size=1024, max_points=180):
    """Search the full known free map and refine every selected distinct mode.

    Output is an offline seed proposal; snapshot clocks are never refreshed and
    current source/physical stationarity must be revalidated by the live caller.
    """
    import numpy as np
    info, cells, identity = verified_map(map_doc, pgm_bytes, expected_map_id)
    snapshot_context = scans if type(scans) is dict else {}
    if snapshot_context.get("map_id", expected_map_id) != expected_map_id:
        raise ValueError("Scan snapshot map identity differs")
    if type(scans) is dict:
        scans = scans.get("snapshots", scans.get("scans"))
    if type(scans) not in (list, tuple) or len(scans) != 3:
        raise ValueError("Exactly three scan snapshots required")
    if type(extrinsics) not in (list, tuple) or len(extrinsics) != 3:
        raise ValueError("Explicit laser-to-base x,y,yaw required")
    extrinsics = tuple(finite(v, "laser extrinsics") for v in extrinsics)
    if "mounting" in snapshot_context:
        mounting = snapshot_context["mounting"]
        if type(mounting) is not dict:
            raise ValueError("Snapshot mounting metadata must be an object")
        for key, expected in zip(("x_m", "y_m", "yaw_rad"), extrinsics):
            if abs(finite(mounting.get(key), "snapshot mounting " + key) - expected) > 1e-9:
                raise ValueError("Snapshot mounting differs from explicit proposal extrinsics")
    expected_laser_frame = None
    if "frames" in snapshot_context:
        frames = snapshot_context["frames"]
        if (type(frames) is not dict or type(frames.get("laser")) is not str
                or not frames["laser"].strip()):
            raise ValueError("Snapshot frames must specify a nonempty laser frame")
        expected_laser_frame = frames["laser"]
    values = dict(coarse_step_m=coarse_step_m, coarse_yaw_deg=coarse_yaw_deg,
                  fine_step_m=fine_step_m, fine_yaw_deg=fine_yaw_deg,
                  refine_radius_m=refine_radius_m, refine_yaw_deg=refine_yaw_deg)
    for name, value in values.items():
        values[name] = finite(value, name)
        if values[name] <= 0:
            raise ValueError(name + " must be positive")
    if (type(mode_count) is not int or not 2 <= mode_count <= 10
            or type(batch_size) is not int or not 1 <= batch_size <= 4096
            or type(max_points) is not int or not 60 <= max_points <= 720
            or values["coarse_yaw_deg"] > 20 or values["fine_yaw_deg"] > 5):
        raise ValueError("Invalid search limits")
    parsed = [scan_points(scan, extrinsics, max_points) for scan in scans]
    scan_frames = [item[1]["frame_id"] for item in parsed]
    if expected_laser_frame is not None and any(frame != expected_laser_frame for frame in scan_frames):
        raise ValueError("Scan header frame differs from snapshot laser frame")
    if any(frame is not None for frame in scan_frames):
        if (any(type(frame) is not str or not frame.strip() for frame in scan_frames)
                or len(set(scan_frames)) != 1):
            raise ValueError("Three scan header frames must be present and identical")
    stamps = [item[1]["exact_source_key"] for item in parsed]
    if not stamps[0] < stamps[1] < stamps[2]:
        raise ValueError("Three different advancing original scan stamps required")
    grid = np.asarray(cells, dtype=int).reshape(info["height"], info["width"])
    resolution = float(info["resolution"])
    ox, oy, oa = info["origin"]
    ca, sa = math.cos(oa), math.sin(oa)
    field = distance_field(grid >= 65, resolution)
    if not bool(np.any(grid >= 65)):
        raise ValueError("Saved map has no occupied landmarks")
    stride = max(1, int(round(values["coarse_step_m"] / resolution)))
    free_y, free_x = np.nonzero((grid >= 0) & (grid <= 25))
    chosen = (free_y % stride == 0) & (free_x % stride == 0)
    if not bool(np.any(chosen)):
        raise ValueError("No free-cell coarse candidate at the requested resolution")
    lx, ly = (free_x[chosen] + .5) * resolution, (free_y[chosen] + .5) * resolution
    xy = np.column_stack((ox + ca * lx - sa * ly, oy + sa * lx + ca * ly))
    if 360. / values["coarse_yaw_deg"] > 720:
        raise ValueError("Coarse angular work exceeds 720 headings")
    yaws = np.deg2rad(np.arange(-180., 180., values["coarse_yaw_deg"]))
    if len(xy) * len(yaws) > 32000000:
        raise ValueError("Coarse score matrix exceeds the bounded work limit")
    if (2 * values["refine_radius_m"] / values["fine_step_m"] + 2) ** 2 * (
            2 * values["refine_yaw_deg"] / values["fine_yaw_deg"] + 2) > 2000000:
        raise ValueError("Per-mode refinement exceeds the bounded work limit")

    def indices(wx, wy):
        dx, dy = wx - ox, wy - oy
        return (np.floor((ca * dx + sa * dy) / resolution).astype(int),
                np.floor((-sa * dx + ca * dy) / resolution).astype(int))

    def score_at(points, positions, headings, binary=False):
        output = np.full((len(headings), len(positions)), -np.inf)
        for yi, yaw in enumerate(headings):
            c, s = math.cos(float(yaw)), math.sin(float(yaw))
            px, py = c * points[:, 0] - s * points[:, 1], s * points[:, 0] + c * points[:, 1]
            for first in range(0, len(positions), batch_size):
                batch = positions[first:first + batch_size]
                cx, cy = indices(batch[:, 0], batch[:, 1])
                valid_center = (cx >= 0) & (cx < info["width"]) & (cy >= 0) & (cy < info["height"])
                free = np.zeros(len(batch), dtype=bool)
                free[valid_center] = ((grid[cy[valid_center], cx[valid_center]] >= 0)
                                      & (grid[cy[valid_center], cx[valid_center]] <= 25))
                gx, gy = indices(batch[:, 0, None] + px, batch[:, 1, None] + py)
                inside = (gx >= 0) & (gx < info["width"]) & (gy >= 0) & (gy < info["height"])
                known, distances = np.zeros(gx.shape, dtype=bool), np.full(gx.shape, .4)
                known[inside] = grid[gy[inside], gx[inside]] >= 0
                distances[known] = field[gy[known], gx[known]]
                matches = ((distances <= .10 + 1e-12).astype(float) if binary
                           else np.exp(-.5 * (distances / .10) ** 2))
                matches[~known] = 0.0  # Unknown and outside endpoints never match.
                scores = matches.mean(axis=1)
                scores[~free] = -np.inf
                output[yi, first:first + len(batch)] = scores
        return output

    scores = score_at(parsed[0][0], xy, yaws)
    score_bytes = int(scores.nbytes)
    coarse_modes = []
    for _ in range(mode_count):
        yi, xi = np.unravel_index(int(np.argmax(scores)), scores.shape)
        value = float(scores[yi, xi])
        if not math.isfinite(value):
            break
        pose = [float(xy[xi, 0]), float(xy[xi, 1]), angle(float(yaws[yi]))]
        coarse_modes.append(dict(score=value, pose=pose))
        near_xy = np.hypot(xy[:, 0] - pose[0], xy[:, 1] - pose[1]) <= .4
        near_yaw = np.abs(np.arctan2(np.sin(yaws - pose[2]), np.cos(yaws - pose[2]))) <= math.radians(20)
        scores[np.ix_(near_yaw, near_xy)] = -np.inf
    if not coarse_modes:
        raise ValueError("No free global pose candidate")

    refined = []
    offsets = np.arange(-values["refine_radius_m"], values["refine_radius_m"] + values["fine_step_m"] * .1,
                        values["fine_step_m"])
    yaw_offsets = np.deg2rad(np.arange(-values["refine_yaw_deg"], values["refine_yaw_deg"] + values["fine_yaw_deg"] * .1,
                                      values["fine_yaw_deg"]))
    for mode in coarse_modes:
        pose = mode["pose"]
        positions = np.asarray([(pose[0] + dx, pose[1] + dy) for dx in offsets for dy in offsets])
        headings = pose[2] + yaw_offsets
        local = score_at(parsed[0][0], positions, headings)
        yi, xi = np.unravel_index(int(np.argmax(local)), local.shape)
        if math.isfinite(float(local[yi, xi])):
            refined.append(dict(score=float(local[yi, xi]),
                                pose=[float(positions[xi, 0]), float(positions[xi, 1]), angle(float(headings[yi]))],
                                coarse_seed=list(pose)))
    modes = []
    for candidate in sorted(refined + coarse_modes, key=lambda item: item["score"], reverse=True):
        p = candidate["pose"]
        if all(math.hypot(p[0] - other["pose"][0], p[1] - other["pose"][1]) > .4
               or abs(angle(p[2] - other["pose"][2])) > math.radians(20) for other in modes):
            modes.append(candidate)
    best = modes[0]
    margin = best["score"] - modes[1]["score"] if len(modes) > 1 else None
    frame_agreement = []
    for points, source in parsed:
        frame_agreement.append(dict(endpoint_agreement(info, cells, points, best["pose"]),
                                    source_stamp=source["source_stamp"]))
    accepted = bool(margin is not None and margin >= .08
                    and all(row["used"] >= 60 and row["near_wall_ratio"] >= .65 for row in frame_agreement))
    return dict(schema="p2p-global-pose-proposal-v1", pose=best["pose"], score=best["score"],
        seed_candidate_accepted=accepted, localization_verified=False,
        device_permit=False, navigation_task_allowed=False, motion_requested=False,
        map_identity=identity, map_id=identity["map_id"],
        boot_id=snapshot_context.get("boot_id"), nonce=snapshot_context.get("nonce"),
        proposal_snapshot_stamps=[item[1]["source_stamp"] for item in parsed],
        proposal_snapshot_exact_stamps=[item[1]["exact_source_key"] for item in parsed],
        mode_score_margin=margin, competing_modes=modes[:mode_count],
        coarse_modes=coarse_modes, refined_modes=refined, refined_mode_count=len(refined),
        frame_agreement=frame_agreement, scan_sources=[p[1] for p in parsed],
        source_time_preserved=True, source_freshness_revalidated=False,
        stationarity_revalidated=False, navigation_fov_deg=180,
        extrinsics=dict(x_m=extrinsics[0], y_m=extrinsics[1], yaw_rad=extrinsics[2],
                        basis="explicit_model_must_equal_live_TF", calibrated=False),
        search_basis="Full saved-map free-cell candidates and all yaw; no historical robot pose used",
        search=dict(parameters=values, free_cells=int(len(free_x)), coarse_candidates=int(len(xy)),
                    coarse_yaws=int(len(yaws)), coarse_stride_cells=stride,
                    actual_coarse_step_m=stride * resolution, batch_size=batch_size,
                    score_matrix_bytes=score_bytes, max_points_per_scan=max_points,
                    every_selected_distinct_mode_refined=True),
        acceptance_thresholds=dict(mode_margin=.08, frame_near_wall_ratio=.65, points_per_frame=60),
        agreement_scope="fit_frame_and_two_held_out_frames; proposal_only")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("map-json", "map-pgm", "scan-snapshot", "output"):
        parser.add_argument("--" + name, required=True)
    parser.add_argument("--numpy-dir")
    parser.add_argument("--expected-map-id", default=SAVED_MAP_ID)
    parser.add_argument("--laser-x", type=float, default=.13)
    parser.add_argument("--laser-y", type=float, default=0.)
    parser.add_argument("--laser-yaw-rad", type=float, default=0.)
    parser.add_argument("--coarse-step-m", type=float, default=.10)
    parser.add_argument("--coarse-yaw-deg", type=float, default=5.)
    args = parser.parse_args(argv)
    if args.numpy_dir:
        sys.path.insert(0, str(Path(args.numpy_dir).resolve(strict=True)))
    result = global_pose_proposal(json.loads(Path(args.map_json).read_text(encoding="utf-8")),
        Path(args.map_pgm).read_bytes(), json.loads(Path(args.scan_snapshot).read_text(encoding="utf-8")),
        args.expected_map_id, extrinsics=(args.laser_x, args.laser_y, args.laser_yaw_rad),
        coarse_step_m=args.coarse_step_m, coarse_yaw_deg=args.coarse_yaw_deg)
    Path(args.output).write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps(dict(output=str(Path(args.output).resolve()),
                         seed_candidate_accepted=result["seed_candidate_accepted"], pose=result["pose"],
                         mode_score_margin=result["mode_score_margin"], localization_verified=False)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
