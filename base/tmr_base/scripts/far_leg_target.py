"""ROS-free, frame-explicit far-leg selection and front-edge alignment geometry."""

from collections import deque
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path

from table_leg_detection import cluster_points, DetectionError


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def in_reference(pose, origin):
    dx, dy = pose[0] - origin[0], pose[1] - origin[1]
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return c * dx + s * dy, -s * dx + c * dy, wrap(pose[2] - origin[2])


def to_odom(point, origin):
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return origin[0] + c * point[0] - s * point[1], origin[1] + s * point[
        0
    ] + c * point[1]


def interpolate_pose(history, stamp):
    """Interpolate only bracketed odometry; never substitute the latest pose."""
    for (ta, a), (tb, b) in zip(history, list(history)[1:]):
        if ta <= stamp <= tb and 0 < tb - ta <= 0.15:
            u = (stamp - ta) / (tb - ta)
            return (
                a[0] + u * (b[0] - a[0]),
                a[1] + u * (b[1] - a[1]),
                wrap(a[2] + u * wrap(b[2] - a[2])),
            )
    return None


def project_scan(scan, extrinsic, pose, origin, cfg):
    """Full mounting XY projection, scan-time odometry, then fixed green START."""
    robot = in_reference(pose, origin)
    c, s = math.cos(robot[2]), math.sin(robot[2])
    points = []
    for i in range(0, len(scan.ranges), 2):
        distance = float(scan.ranges[i])
        if (
            not math.isfinite(distance)
            or not scan.range_min <= distance <= scan.range_max
        ):
            continue
        angle = scan.angle_min + i * scan.angle_increment
        ux, uy = extrinsic.project_unit(angle)
        bx, by = extrinsic.tx + distance * ux, extrinsic.ty + distance * uy
        if abs(bx) <= 0.41 and abs(by) <= 0.30:
            continue
        x, y = robot[0] + c * bx - s * by, robot[1] + s * bx + c * by
        if (
            cfg["roi"]["x"][0] <= x <= cfg["roi"]["x"][1]
            and cfg["roi"]["y"][0] <= y <= cfg["roi"]["y"][1]
        ):
            points.append((x, y))
    return points


@dataclass
class ScanFrame:
    source: str
    stamp: float
    points: list


class EvidenceWindow:
    def __init__(self, cfg):
        self.cfg = cfg
        self.frames = deque(maxlen=160)
        self.last_stamp = {}

    def add(self, frame):
        if frame.stamp <= self.last_stamp.get(frame.source, 0):
            return False
        self.last_stamp[frame.source] = frame.stamp
        self.frames.append(frame)
        return True

    def detect(self, now):
        frames = [f for f in self.frames if 0 <= now - f.stamp <= self.cfg["window_s"]]
        if len(frames) < self.cfg["minimum_frames"]:
            raise DetectionError("too few independent scan frames")
        for source in self.cfg["scan_topics"]:
            source_frames = [f for f in frames if f.source == source]
            if (
                len(source_frames) < self.cfg["minimum_frames_per_source"]
                or now - max(f.stamp for f in source_frames) > self.cfg["fresh_s"]
            ):
                raise DetectionError(f"missing or stale scanner: {source}")
        clusters = cluster_points((p for f in frames for p in f.points), self.cfg)
        valid = []
        for leg in clusters:
            radius = leg.diameter * math.sqrt(2) / 2 + self.cfg["grid_resolution"]
            support = [
                f
                for f in frames
                if any(math.hypot(x - leg.x, y - leg.y) <= radius for x, y in f.points)
            ]
            if (
                len(support) >= self.cfg["minimum_leg_frames"]
                and now - max(f.stamp for f in support) <= self.cfg["fresh_s"]
            ):
                valid.append(leg)
        if len(valid) != 2:
            raise DetectionError(
                f"expected exactly two stable leg clusters in ROI, found {len(valid)}"
            )
        near, far = sorted(valid, key=lambda leg: leg.x)
        if far.x - near.x < self.cfg["minimum_x_separation_m"]:
            raise DetectionError(
                "two clusters do not have a distinct near/far x ordering"
            )
        return {
            "near": asdict(near),
            "far": asdict(far),
            "frames": len(frames),
            "newest_stamp": max(f.stamp for f in frames),
        }


class StableLegTarget:
    def __init__(self, cfg):
        self.cfg = cfg
        self.history = deque(maxlen=cfg["stable_observations"])
        self.anchor = None
        self.last_stamp = 0.0

    def missing(self):
        self.history.clear()

    def update(self, detection):
        if detection["newest_stamp"] <= self.last_stamp:
            return None
        self.last_stamp = detection["newest_stamp"]
        for key in ("near", "far"):
            point = detection[key]
            if (
                self.anchor is not None
                and math.hypot(
                    point["x"] - self.anchor[key]["x"],
                    point["y"] - self.anchor[key]["y"],
                )
                > self.cfg["locked_target_tolerance_m"]
            ):
                self.missing()
                raise DetectionError(
                    "locked table-leg identity changed; cannot switch to another leg"
                )
            if any(
                math.hypot(point["x"] - old[key]["x"], point["y"] - old[key]["y"])
                > self.cfg["stability_m"]
                for old in self.history
            ):
                self.missing()
        self.history.append(detection)
        if len(self.history) < self.history.maxlen:
            return None
        if self.anchor is None:
            self.anchor = detection
        return detection


def alignment_error(pose_roi, leg, cfg):
    yaw = wrap(pose_roi[2])
    if abs(yaw) > math.radians(cfg["maximum_heading_error_deg"]):
        raise DetectionError(
            "forward heading is not aligned with green START +x; ROI origin/heading needs checking"
        )
    # Most forward corner of the front edge, expressed along ROI +x.
    front_x = (
        pose_roi[0]
        + cfg["front_offset_m"] * math.cos(yaw)
        + cfg["half_width_m"] * abs(math.sin(yaw))
    )
    return leg["x"] - front_x, front_x


def forward_speed(error, cfg):
    tolerance = cfg["front_alignment_tolerance_m"]
    if error < -tolerance:
        raise DetectionError("base front has already passed the target leg")
    if error <= tolerance:
        return 0.0
    return min(cfg["maximum_speed_mps"], 0.65 * error)


def driver_session(proc_root=Path("/proc")):
    matches = []
    for item in proc_root.iterdir():
        if not item.name.isdigit():
            continue
        try:
            command = (item / "cmdline").read_bytes().split(b"\0")[0]
            if not command.endswith(b"/ros2_control_node"):
                continue
            fields = (item / "stat").read_text().rsplit(")", 1)[1].split()
            matches.append((item.name, fields[19]))
        except (OSError, IndexError):
            continue
    if len(matches) != 1:
        raise RuntimeError(
            "need one running base ros2_control_node to bind odometry session"
        )
    boot = (proc_root / "sys/kernel/random/boot_id").read_text().strip()
    return hashlib.sha256(repr((boot, matches)).encode()).hexdigest()


def load_roi_origin(path, expected_session, expected_frame):
    value = json.loads(Path(path).read_text())
    if (
        value.get("reference") != "green_start"
        or value.get("stationary_confirmed") is not True
    ):
        raise ValueError("ROI origin must be captured at the marked green START")
    if value.get("driver_session") != expected_session or (
        expected_frame and value.get("odom_frame") != expected_frame.lstrip("/")
    ):
        raise ValueError(
            "ROI origin belongs to another odometry/driver session; recapture at green START"
        )
    pose = value["pose_odom"]
    if len(pose) != 3 or not all(math.isfinite(float(x)) for x in pose):
        raise ValueError("invalid ROI origin pose")
    return value


def read_config(path):
    cfg = json.loads(Path(path).read_text())
    # This route has a fixed user-approved ROI and measured chassis geometry.
    if (
        cfg["roi"] != {"x": [1.65, 3.65], "y": [-0.1, 0.6]}
        or cfg["roi_reference"] != "current_pose"
        or cfg["front_offset_m"] != 0.40
        or cfg["half_width_m"] != 0.29
    ):
        raise ValueError("unexpected ROI reference or chassis geometry")
    for key in (
        "grid_resolution",
        "cluster_connect_distance",
        "min_cell_hits",
        "min_cluster_hits",
        "max_leg_diameter",
        "minimum_x_separation_m",
        "minimum_frames",
        "minimum_frames_per_source",
        "minimum_leg_frames",
        "window_s",
        "fresh_s",
        "stable_observations",
        "stability_m",
        "locked_target_tolerance_m",
        "front_alignment_tolerance_m",
        "maximum_heading_error_deg",
        "maximum_forward_m",
        "maximum_speed_mps",
        "acquire_timeout_s",
        "lost_target_timeout_s",
        "approach_timeout_s",
    ):
        if not math.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError(f"invalid positive configuration: {key}")
    if cfg["maximum_speed_mps"] > 0.05 or cfg["maximum_forward_m"] > 3.0:
        raise ValueError("approach speed/distance exceeds route bounds")
    return cfg
