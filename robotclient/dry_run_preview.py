"""Read the portable numeric artifact emitted by ``23_dry_run_preview.py``."""

from dataclasses import dataclass
import json
from pathlib import Path
import zipfile

import numpy as np


MAX_PREVIEW_SAMPLES = 100_000
# The GUI and the offline preview command share one transient handoff artifact.
# Reusing this path keeps normal preview runs from accumulating files.
ACTIVE_PREVIEW_PATH = Path('/tmp/rcb-runtime/active_preview.npz')


def _npy_shape(archive, member):
    """Read one NPY member's shape without decompressing its data array."""
    with archive.open(member) as stream:
        version = np.lib.format.read_magic(stream)
        if version == (1, 0):
            shape, _, _ = np.lib.format.read_array_header_1_0(stream)
        elif version == (2, 0):
            shape, _, _ = np.lib.format.read_array_header_2_0(stream)
        else:
            raise ValueError(f"unsupported NPY header version {version}")
    return tuple(shape)


def _sample_count_before_load(path, segments):
    """Count samples from NPY headers before loading trajectory arrays."""
    sample_count = 0
    segment_counts = []
    try:
        with zipfile.ZipFile(path) as archive:
            for segment in segments:
                prefix = segment.get("prefix")
                if not isinstance(prefix, str) or not prefix:
                    raise ValueError("not a dry-run preview: segment prefix is invalid")
                shape = _npy_shape(archive, f"{prefix}_time_s.npy")
                if len(shape) != 1:
                    raise ValueError("not a dry-run preview: time array must be one-dimensional")
                count = int(shape[0])
                segment_counts.append(count)
                sample_count += count
                if sample_count > MAX_PREVIEW_SAMPLES:
                    raise ValueError(
                        f"preview has more than {MAX_PREVIEW_SAMPLES:,} samples; "
                        "split the program into smaller previews")
    except ValueError:
        raise
    except (KeyError, OSError, zipfile.BadZipFile) as exc:
        raise ValueError("not a dry-run preview: cannot inspect sample counts") from exc
    return sample_count, tuple(segment_counts)



@dataclass(frozen=True)
class DryRunPreviewData:
    """Timestamped, concatenated samples from a dry-run preview artifact."""

    time_s: np.ndarray
    joint_position_rad: np.ndarray
    joint_velocity_rad_s: np.ndarray
    joint_acceleration_rad_s2: np.ndarray
    joint_jerk_rad_s3: np.ndarray
    tcp_pose: np.ndarray
    tcp_velocity: np.ndarray
    tcp_acceleration: np.ndarray
    segments: tuple[dict, ...]
    events: tuple[dict, ...]
    duration_s: float
    segment_sample_counts: tuple[int, ...]


def load_dry_run_preview(path):
    """Load a numeric preview artifact without allowing pickle deserialization."""
    path = Path(path)
    with np.load(path, allow_pickle=False) as payload:
        if "metadata_json" not in payload:
            raise ValueError("not a dry-run preview: missing metadata_json")
        try:
            metadata = json.loads(str(payload["metadata_json"].item()))
        except (json.JSONDecodeError, ValueError) as exc:
            raise ValueError("not a dry-run preview: metadata_json is invalid") from exc

        segments = metadata.get("segments")
        if not isinstance(segments, list) or not segments:
            raise ValueError("not a dry-run preview: no planned segments")
        sample_count, segment_sample_counts = _sample_count_before_load(path, segments)
        declared_count = metadata.get("sample_count")
        if declared_count is not None and int(declared_count) != sample_count:
            raise ValueError("not a dry-run preview: sample count metadata is inconsistent")

        arrays = {name: [] for name in (
            "time_s", "joint_position_rad", "joint_velocity_rad_s",
            "joint_acceleration_rad_s2", "tcp_pose", "tcp_velocity",
            "tcp_acceleration",
        )}
        jerk_arrays = []
        jerk_present = []
        for segment in segments:
            prefix = segment.get("prefix")
            if not isinstance(prefix, str) or not prefix:
                raise ValueError("not a dry-run preview: segment prefix is invalid")
            for name in arrays:
                key = f"{prefix}_{name}"
                if key not in payload:
                    raise ValueError(f"not a dry-run preview: missing {key}")
                arrays[name].append(np.asarray(payload[key], dtype=np.float64))
            jerk_key = f"{prefix}_joint_jerk_rad_s3"
            jerk_present.append(jerk_key in payload)
            if jerk_key in payload:
                jerk_arrays.append(np.asarray(payload[jerk_key], dtype=np.float64))

    time_s = np.concatenate(arrays["time_s"])
    matrix_names = tuple(name for name in arrays if name != "time_s")
    matrices = {name: np.concatenate(arrays[name]) for name in matrix_names}
    if time_s.ndim != 1 or len(time_s) < 2 or not np.isfinite(time_s).all():
        raise ValueError("not a dry-run preview: time samples are invalid")
    if np.any(np.diff(time_s) < 0.0):
        raise ValueError("not a dry-run preview: time samples are not ordered")
    if any(jerk_present):
        if not all(jerk_present):
            raise ValueError("not a dry-run preview: joint jerk arrays are incomplete")
        matrices["joint_jerk_rad_s3"] = np.concatenate(jerk_arrays)
    else:
        positive_steps = np.diff(time_s)
        positive_steps = positive_steps[positive_steps > 0.0]
        if not len(positive_steps):
            raise ValueError("not a dry-run preview: cannot derive jerk without time spacing")
        step_s = float(np.median(positive_steps))
        matrices["joint_jerk_rad_s3"] = np.gradient(
            matrices["joint_acceleration_rad_s2"], step_s, axis=0)
    for name, values in matrices.items():
        if values.shape != (len(time_s), 6) or not np.isfinite(values).all():
            raise ValueError(f"not a dry-run preview: {name} must be finite (N, 6)")

    return DryRunPreviewData(
        time_s=time_s,
        joint_position_rad=matrices["joint_position_rad"],
        joint_velocity_rad_s=matrices["joint_velocity_rad_s"],
        joint_acceleration_rad_s2=matrices["joint_acceleration_rad_s2"],
        joint_jerk_rad_s3=matrices["joint_jerk_rad_s3"],
        tcp_pose=matrices["tcp_pose"],
        tcp_velocity=matrices["tcp_velocity"],
        tcp_acceleration=matrices["tcp_acceleration"],
        segment_sample_counts=segment_sample_counts,
        segments=tuple(segments),
        events=tuple(metadata.get("events", [])),
        duration_s=float(metadata.get("duration_s", time_s[-1])),
    )
