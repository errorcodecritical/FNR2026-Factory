from pathlib import Path

from factory_autonomy.scheduler import (
    ZONE_INCOMING,
    ZONE_MACHINE_A,
    ZONE_MACHINE_B,
    ZONE_OUTGOING,
    waypoint_name,
)
from factory_autonomy.waypoints import load_waypoints_file


def test_shared_waypoints_cover_all_scheduler_slots():
    repo_root = Path(__file__).resolve().parents[4]
    waypoint_path = repo_root / 'shared' / 'factory_waypoints.yaml'

    store = load_waypoints_file(str(waypoint_path))

    for zone in (ZONE_INCOMING, ZONE_OUTGOING, ZONE_MACHINE_A, ZONE_MACHINE_B):
        for slot in range(1, 5):
            canonical_name = waypoint_name(zone, slot)
            alias = f'{zone}:{slot}'

            waypoint = store.resolve(canonical_name)
            assert waypoint is not None
            assert waypoint.name == canonical_name
            assert store.resolve(alias) == waypoint
