"""Capture named 2D navigation poses from TF for ros_vention navigation."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import yaml

try:
    import rospy
    import tf2_ros

    ROSPY_IMPORTED = True
except ModuleNotFoundError:
    ROSPY_IMPORTED = False


def _default_locations_file() -> Path:
    return Path(__file__).resolve().parents[3] / "config" / "nav_named_locations.yaml"


def _load_yaml(filepath: Path) -> dict[str, Any]:
    if not filepath.exists():
        return {"frame_id": "map", "locations": {}}
    with open(filepath, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    data.setdefault("frame_id", "map")
    data.setdefault("locations", {})
    return data


def _save_yaml(filepath: Path, data: dict[str, Any]) -> None:
    filepath.parent.mkdir(parents=True, exist_ok=True)
    with open(filepath, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)


def _lookup_pose(
    tf_buffer: tf2_ros.Buffer,
    map_frame: str,
    base_frame: str,
    timeout_s: float,
) -> dict[str, Any]:
    transform = tf_buffer.lookup_transform(
        target_frame=map_frame,
        source_frame=base_frame,
        time=rospy.Time(0),
        timeout=rospy.Duration(timeout_s),
    )

    t = transform.transform.translation
    q = transform.transform.rotation
    return {
        "frame_id": map_frame,
        "position": {
            "x": round(float(t.x), 6),
            "y": round(float(t.y), 6),
            "z": round(float(t.z), 6),
        },
        "orientation": {
            "x": round(float(q.x), 6),
            "y": round(float(q.y), 6),
            "z": round(float(q.z), 6),
            "w": round(float(q.w), 6),
        },
    }


def _confirm(prompt: str) -> bool:
    while True:
        resp = input(prompt).strip().lower()
        if resp in {"y", "yes"}:
            return True
        if resp in {"n", "no"}:
            return False
        print("Please answer y/yes or n/no.")


def _print_tf_debug_info(tf_buffer: tf2_ros.Buffer) -> None:
    """Print a short TF debug dump to help diagnose missing frames."""
    try:
        frames_txt = tf_buffer.all_frames_as_string()
    except Exception:  # pragma: no cover - debug-only path
        print("Could not query TF frame list from buffer.")
        return
    print("Current TF frame graph:\n" + frames_txt)


def _main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--locations-file", type=Path, default=_default_locations_file())
    parser.add_argument("--map-frame", type=str, default="map")
    parser.add_argument("--base-frame", type=str, default="vention_base_link")
    parser.add_argument(
        "--locations",
        type=str,
        default="fridge,microwave,table,sink",
        help="Comma-separated ordered location names.",
    )
    parser.add_argument("--lookup-timeout-s", type=float, default=2.0)
    args = parser.parse_args()

    if not ROSPY_IMPORTED:
        raise RuntimeError("ROS not imported. Run this script in a ROS environment.")

    rospy.init_node("capture_named_locations", anonymous=True)
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(20.0))
    _tf_listener = tf2_ros.TransformListener(tf_buffer)
    del _tf_listener

    config = _load_yaml(args.locations_file)
    config["frame_id"] = args.map_frame

    ordered_locations = [x.strip() for x in args.locations.split(",") if x.strip()]
    if not ordered_locations:
        raise ValueError("No locations provided.")

    print("TF capture ready.")
    print(f"Using map frame: {args.map_frame}")
    print(f"Using base frame: {args.base_frame}")
    print(f"Saving to: {args.locations_file}")

    for location_name in ordered_locations:
        existing = config["locations"].get(location_name)
        if existing is not None:
            overwrite = _confirm(
                f"Location '{location_name}' already exists. Overwrite? [y/n]: "
            )
            if not overwrite:
                print(f"Skipping {location_name}.")
                continue

        while not rospy.is_shutdown():

            input(
                f"Move the robot in front of '{location_name}', then press Enter to capture pose..."
            )

            try:
                pose = _lookup_pose(
                    tf_buffer=tf_buffer,
                    map_frame=args.map_frame,
                    base_frame=args.base_frame,
                    timeout_s=args.lookup_timeout_s,
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
                tf2_ros.TimeoutException,
            ) as exc:
                print(f"TF lookup failed: {exc}")
                print(
                    "Hint: ensure a node is publishing map->odom and odom->"
                    f"{args.base_frame}. In this stack, run Cartographer (SLAM or localization) "
                    "before capturing named locations."
                )
                _print_tf_debug_info(tf_buffer)
                retry = _confirm("Retry capture for this location? [y/n]: ")
                if retry:
                    continue
                break

            p = pose["position"]
            q = pose["orientation"]
            print(
                f"Captured {location_name}: "
                f"pos=({p['x']:.3f}, {p['y']:.3f}, {p['z']:.3f}), "
                f"quat=({q['x']:.3f}, {q['y']:.3f}, {q['z']:.3f}, {q['w']:.3f})"
            )

            save = _confirm(f"Save this pose for '{location_name}'? [y/n]: ")
            if save:
                config["locations"][location_name] = pose
                _save_yaml(args.locations_file, config)
                print(f"Saved {location_name}.")
                break

            retry = _confirm("Retake this location now? [y/n]: ")
            if not retry:
                print(f"Skipped {location_name}.")
                break

    _save_yaml(args.locations_file, config)
    print("Done. Final location file written.")


if __name__ == "__main__":
    _main()
