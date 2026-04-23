"""Factory movement planner.

The planner does not receive jobs. It looks at the current warehouse/machine
snapshot and chooses the next useful transport action:

    R -> Machine A input -> Machine A output becomes G
    G -> Machine B input -> Machine B output becomes B
    B -> Outgoing warehouse
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

from .crate_protocol import (
    FactorySnapshot,
    PART_EMPTY,
    PART_FINAL,
    PART_INTERMEDIATE,
    PART_RAW,
    target_zone_for_part,
)


ZONE_INCOMING = 'incoming'
ZONE_OUTGOING = 'outgoing'
ZONE_MACHINE_A = 'machine_a'
ZONE_MACHINE_B = 'machine_b'

MACHINE_INPUT_SLOTS = (1, 3)
MACHINE_OUTPUT_SLOTS = (2, 4)
_INPUT_TO_OUTPUT = {1: 2, 3: 4}


@dataclass(frozen=True)
class FactoryLocation:
    """One physical slot in the incoming area, outgoing area, or a machine."""

    zone: str
    slot: int

    @property
    def waypoint_name(self) -> str:
        """Canonical waypoint id used by the navigation layer."""
        return waypoint_name(self.zone, self.slot)

    def label(self) -> str:
        """Human-readable label used in logs and status output."""
        return f'{self.zone}:{self.slot}'


@dataclass(frozen=True)
class TransportTask:
    """A single move: take one part from source and place it at target."""

    part_type: str
    source: FactoryLocation
    target: FactoryLocation

    @property
    def pickup_waypoint(self) -> str:
        """Waypoint name for the source slot."""
        return self.source.waypoint_name

    @property
    def dropoff_waypoint(self) -> str:
        """Waypoint name for the target slot."""
        return self.target.waypoint_name

    def to_dict(self) -> dict:
        """Serialize the task into status/log-friendly fields."""
        return {
            'part_type': self.part_type,
            'source': self.source.label(),
            'target': self.target.label(),
            'pickup_waypoint': self.pickup_waypoint,
            'dropoff_waypoint': self.dropoff_waypoint,
        }


class FactoryPlanner:
    """Select the next movement from the current factory state."""

    def choose_next(self, snapshot: FactorySnapshot) -> Optional[TransportTask]:
        """Choose the highest-priority feasible transport.

        The priority order is tuned to keep the processing pipeline flowing:
        raw parts first, then intermediate parts, then unloading machine
        outputs, and finally shipping already-final incoming parts.
        """
        occupied_outputs = list(_machine_output_sources(snapshot))
        incoming_sources = list(_incoming_sources(snapshot))

        # Priority 1 and 2: load parts that still need processing.
        for desired_type in (PART_RAW, PART_INTERMEDIATE):
            for source in incoming_sources:
                if _slot_value(snapshot, source) == desired_type:
                    task = self._task_for_source(snapshot, source)
                    if task is not None:
                        return task

        # Priority 3: unload processed machine outputs.
        for source in occupied_outputs:
            task = self._task_for_source(snapshot, source)
            if task is not None:
                return task

        # Priority 4: move final incoming parts to the outgoing warehouse.
        for source in incoming_sources:
            if _slot_value(snapshot, source) == PART_FINAL:
                task = self._task_for_source(snapshot, source)
                if task is not None:
                    return task

        return None

    def _task_for_source(
        self,
        snapshot: FactorySnapshot,
        source: FactoryLocation,
    ) -> Optional[TransportTask]:
        """Build a task for one source slot if a valid destination exists."""
        part_type = _slot_value(snapshot, source)
        target_zone = target_zone_for_part(part_type)
        if target_zone is None:
            return None

        target = _first_available_target(snapshot, target_zone)
        if target is None:
            return None

        return TransportTask(part_type=part_type, source=source, target=target)


def waypoint_name(zone: str, slot: int) -> str:
    """Convert a zone/slot pair into the canonical waypoint key."""
    prefixes = {
        ZONE_INCOMING: 'iwp_slot',
        ZONE_OUTGOING: 'owp_slot',
        ZONE_MACHINE_A: 'machine_a_slot',
        ZONE_MACHINE_B: 'machine_b_slot',
    }
    try:
        prefix = prefixes[zone]
    except KeyError as exc:
        raise ValueError(f'unknown zone {zone!r}') from exc
    return f'{prefix}_{slot}'


def _incoming_sources(snapshot: FactorySnapshot) -> Iterable[FactoryLocation]:
    """Yield all non-empty incoming slots in slot order."""
    for slot, part_type in enumerate(snapshot.incoming, start=1):
        if part_type != PART_EMPTY:
            yield FactoryLocation(ZONE_INCOMING, slot)


def _machine_output_sources(snapshot: FactorySnapshot) -> Iterable[FactoryLocation]:
    """Yield non-empty machine output slots from both machines."""
    for zone, slots in (
        (ZONE_MACHINE_A, snapshot.machine_a),
        (ZONE_MACHINE_B, snapshot.machine_b),
    ):
        for slot in MACHINE_OUTPUT_SLOTS:
            if slots[slot - 1] != PART_EMPTY:
                yield FactoryLocation(zone, slot)


def _first_available_target(
    snapshot: FactorySnapshot,
    target_zone: str,
) -> Optional[FactoryLocation]:
    """Find the first valid destination for a part type.

    Outgoing accepts any empty slot. Machine targets are stricter: an input
    slot is considered available only when both the input and its paired output
    slot are empty, which prevents placing a new part into a machine lane that
    is still occupied by unfinished or uncollected work.
    """
    if target_zone == ZONE_OUTGOING:
        for slot, part_type in enumerate(snapshot.outgoing, start=1):
            if part_type == PART_EMPTY:
                return FactoryLocation(ZONE_OUTGOING, slot)
        return None

    if target_zone in (ZONE_MACHINE_A, ZONE_MACHINE_B):
        slots = _zone_slots(snapshot, target_zone)
        for input_slot in MACHINE_INPUT_SLOTS:
            output_slot = _INPUT_TO_OUTPUT[input_slot]
            input_empty = slots[input_slot - 1] == PART_EMPTY
            output_empty = slots[output_slot - 1] == PART_EMPTY
            if input_empty and output_empty:
                return FactoryLocation(target_zone, input_slot)
        return None

    raise ValueError(f'unknown target zone {target_zone!r}')


def _slot_value(snapshot: FactorySnapshot, location: FactoryLocation) -> str:
    """Read one slot value from the snapshot."""
    return _zone_slots(snapshot, location.zone)[location.slot - 1]


def _zone_slots(snapshot: FactorySnapshot, zone: str) -> Tuple[str, str, str, str]:
    """Return the 4-slot tuple for one zone."""
    if zone == ZONE_INCOMING:
        return snapshot.incoming
    if zone == ZONE_OUTGOING:
        return snapshot.outgoing
    if zone == ZONE_MACHINE_A:
        return snapshot.machine_a
    if zone == ZONE_MACHINE_B:
        return snapshot.machine_b
    raise ValueError(f'unknown zone {zone!r}')
