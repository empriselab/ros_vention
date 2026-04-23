"""Interactive helper for building and saving a 2D map for ros_vention navigation."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import importlib
from pathlib import Path
import subprocess
import time
from typing import Any

try:
    import rospy
    from nav_msgs.msg import OccupancyGrid

    ROSPY_IMPORTED = True
except ModuleNotFoundError:
    ROSPY_IMPORTED = False

@dataclass
class MapStats:
    """Summary statistics for the current map."""

    width: int
    height: int
    total_cells: int
    known_cells: int
    unknown_cells: int
    free_cells: int
    occupied_cells: int

    @property
    def known_ratio(self) -> float:
        if self.total_cells == 0:
            return 0.0
        return self.known_cells / self.total_cells


class InteractiveMapBuilder:
    """Wait for a sufficiently built map and save Cartographer state."""

    def __init__(
        self,
        map_topic: str,
        pbstream_file: Path,
        save_occupancy_snapshot: bool,
        occupancy_map_prefix: Path,
        finish_trajectory_before_save: bool,
        finish_trajectory_service: str,
        write_state_service: str,
        trajectory_id: int,
        sample_period_s: float,
        min_known_cells: int,
        saturation_window: int,
        max_known_growth_per_sample: int,
    ) -> None:
        self._map_topic = map_topic
        self._pbstream_file = pbstream_file
        self._save_occupancy_snapshot = save_occupancy_snapshot
        self._occupancy_map_prefix = occupancy_map_prefix
        self._finish_trajectory_before_save = finish_trajectory_before_save
        self._finish_trajectory_service = finish_trajectory_service
        self._write_state_service = write_state_service
        self._trajectory_id = trajectory_id
        self._sample_period_s = sample_period_s
        self._min_known_cells = min_known_cells
        self._saturation_window = saturation_window
        self._max_known_growth_per_sample = max_known_growth_per_sample
        self._latest_map: OccupancyGrid | None = None

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg

    def _stats_from_msg(self, msg: OccupancyGrid) -> MapStats:
        data = msg.data
        total = len(data)
        unknown = sum(1 for v in data if v < 0)
        occupied = sum(1 for v in data if v >= 65)
        free = sum(1 for v in data if 0 <= v < 65)
        known = total - unknown
        return MapStats(
            width=msg.info.width,
            height=msg.info.height,
            total_cells=total,
            known_cells=known,
            unknown_cells=unknown,
            free_cells=free,
            occupied_cells=occupied,
        )

    def _is_sufficient(self, known_history: deque[int], current_known: int) -> bool:
        if current_known < self._min_known_cells:
            return False
        if len(known_history) < self._saturation_window:
            return False

        growth = [
            known_history[i] - known_history[i - 1]
            for i in range(1, len(known_history))
        ]
        avg_growth = sum(abs(g) for g in growth) / len(growth)
        return avg_growth <= self._max_known_growth_per_sample

    def run(self) -> None:
        if not ROSPY_IMPORTED:
            raise RuntimeError("ROS not imported. Run this script in a ROS environment.")

        print("Starting interactive map builder node...")
        rospy.init_node("build_map_interactive", anonymous=True)
        print(f"Subscribing to map topic: {self._map_topic}")
        rospy.Subscriber(self._map_topic, OccupancyGrid, self._map_cb, queue_size=1)

        print(f"Waiting for map topic: {self._map_topic}")
        rospy.wait_for_message(self._map_topic, OccupancyGrid, timeout=30.0)
        print("Map topic is active.")

        input(
            "Press Enter to start mapping. Teleoperate the base to explore the space. "
            "The script will notify you when the map appears sufficiently built..."
        )

        known_history: deque[int] = deque(maxlen=self._saturation_window)
        while not rospy.is_shutdown():
            msg = self._latest_map
            if msg is None:
                time.sleep(self._sample_period_s)
                continue

            stats = self._stats_from_msg(msg)
            known_history.append(stats.known_cells)
            print(
                "Map progress | "
                f"size={stats.width}x{stats.height}, "
                f"known={stats.known_cells}/{stats.total_cells} "
                f"({100.0 * stats.known_ratio:.1f}%), "
                f"free={stats.free_cells}, occupied={stats.occupied_cells}"
            )

            if self._is_sufficient(known_history, stats.known_cells):
                print("Map appears sufficiently built (coverage has saturated).")
                break

            time.sleep(self._sample_period_s)

        input("Press Enter to save Cartographer state now...")
        self._save_cartographer_state()
        if self._save_occupancy_snapshot:
            self._save_occupancy_map_snapshot()

    def _save_cartographer_state(self) -> None:
        try:
            cartographer_srv_module = importlib.import_module("cartographer_ros_msgs.srv")
            FinishTrajectory = getattr(cartographer_srv_module, "FinishTrajectory")
            WriteState = getattr(cartographer_srv_module, "WriteState")
        except (ModuleNotFoundError, AttributeError) as exc:
            raise RuntimeError(
                "cartographer_ros_msgs is not available in this environment. "
                "Please source a workspace with cartographer_ros installed."
            ) from exc

        self._pbstream_file.parent.mkdir(parents=True, exist_ok=True)

        print(f"Waiting for service: {self._write_state_service}")
        rospy.wait_for_service(self._write_state_service, timeout=20.0)

        write_state_srv: Any = rospy.ServiceProxy(self._write_state_service, WriteState)

        if self._finish_trajectory_before_save:
            print(f"Waiting for service: {self._finish_trajectory_service}")
            rospy.wait_for_service(self._finish_trajectory_service, timeout=20.0)
            finish_srv: Any = rospy.ServiceProxy(self._finish_trajectory_service, FinishTrajectory)
            try:
                finish_resp = finish_srv(self._trajectory_id)
                returned_id = getattr(finish_resp, "trajectory_id", None)
                if returned_id is None:
                    returned_id = getattr(finish_resp, "finished_trajectory_id", None)
                print(
                    "Finished trajectory. "
                    f"Requested id={self._trajectory_id}, returned id={returned_id}, "
                    f"raw_response={finish_resp}."
                )
            except rospy.ServiceException as exc:
                print(
                    "Warning: finish_trajectory call failed. "
                    "Continuing with write_state using unfinished submaps. "
                    f"Details: {exc}"
                )
        else:
            print(
                "Skipping finish_trajectory before write_state. "
                "This keeps Cartographer trajectory active so map->odom TF can continue publishing."
            )

        write_state_srv(
            filename=str(self._pbstream_file),
            include_unfinished_submaps=True,
        )
        print(f"Saved Cartographer state: {self._pbstream_file}")

    def _save_occupancy_map_snapshot(self) -> None:
        self._occupancy_map_prefix.parent.mkdir(parents=True, exist_ok=True)
        cmd = ["rosrun", "map_server", "map_saver", "-f", str(self._occupancy_map_prefix)]
        print("Running map save command:", " ".join(cmd))
        proc = subprocess.run(cmd, check=False, capture_output=True, text=True)
        if proc.returncode != 0:
            print(proc.stdout)
            print(proc.stderr)
            raise RuntimeError("map_saver failed. Verify map_server is installed and /map is valid.")
        print(proc.stdout.strip())
        print(
            "Saved occupancy snapshot files:\n"
            f"  {self._occupancy_map_prefix}.yaml\n"
            f"  {self._occupancy_map_prefix}.pgm"
        )


def _default_pbstream_file() -> Path:
    return Path(__file__).resolve().parents[3] / "config" / "maps" / "vention_map.pbstream"


def _default_occupancy_map_prefix() -> Path:
    return Path(__file__).resolve().parents[3] / "config" / "maps" / "vention_map"


def _main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-topic", type=str, default="/map")
    parser.add_argument("--pbstream-file", type=Path, default=_default_pbstream_file())
    parser.add_argument(
        "--save-occupancy-snapshot",
        action="store_true",
        help="Also export map_server-compatible YAML/PGM snapshot.",
    )
    parser.add_argument(
        "--occupancy-map-prefix",
        type=Path,
        default=_default_occupancy_map_prefix(),
    )
    parser.add_argument(
        "--finish-trajectory-before-save",
        action="store_true",
        help=(
            "Call /finish_trajectory before /write_state. Disabled by default to keep "
            "map->odom TF publishing for follow-up tools like capture_named_locations.py."
        ),
    )
    parser.add_argument("--finish-trajectory-service", type=str, default="/finish_trajectory")
    parser.add_argument("--write-state-service", type=str, default="/write_state")
    parser.add_argument("--trajectory-id", type=int, default=0)
    parser.add_argument("--sample-period-s", type=float, default=2.0)
    parser.add_argument("--min-known-cells", type=int, default=10000)
    parser.add_argument("--saturation-window", type=int, default=8)
    parser.add_argument("--max-known-growth-per-sample", type=int, default=250)
    args = parser.parse_args()

    print("Interactive Map Builder")
    builder = InteractiveMapBuilder(
        map_topic=args.map_topic,
        pbstream_file=args.pbstream_file,
        save_occupancy_snapshot=args.save_occupancy_snapshot,
        occupancy_map_prefix=args.occupancy_map_prefix,
        finish_trajectory_before_save=args.finish_trajectory_before_save,
        finish_trajectory_service=args.finish_trajectory_service,
        write_state_service=args.write_state_service,
        trajectory_id=args.trajectory_id,
        sample_period_s=args.sample_period_s,
        min_known_cells=args.min_known_cells,
        saturation_window=args.saturation_window,
        max_known_growth_per_sample=args.max_known_growth_per_sample,
    )
    print(
        f"Map topic: {args.map_topic}\n"
        f"Cartographer pbstream file: {args.pbstream_file}\n"
        f"Save occupancy snapshot: {args.save_occupancy_snapshot}\n"
        f"Occupancy map prefix: {args.occupancy_map_prefix}\n"
        f"Finish trajectory before save: {args.finish_trajectory_before_save}\n"
        f"finish_trajectory service: {args.finish_trajectory_service}\n"
        f"write_state service: {args.write_state_service}\n"
        f"Trajectory ID: {args.trajectory_id}\n"
        f"Sample period (s): {args.sample_period_s}\n"
        f"Minimum known cells: {args.min_known_cells}\n"
        f"Saturation window (samples): {args.saturation_window}\n"
        f"Max known growth per sample: {args.max_known_growth_per_sample}"
    )
    builder.run()


if __name__ == "__main__":
    _main()
