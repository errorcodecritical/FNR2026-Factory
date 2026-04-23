"""RobotAtFactory competition server protocol helpers.

Chapter 6 of the 2026 rules describes a small UDP request/response server:

    IWP  -> incoming warehouse parts, e.g. "BGGR" or "STOP"
    OWP  -> outgoing warehouse parts, e.g. "BXXX" or "STOP"
    MAP  -> Machine type A slots, e.g. "XGXX" or "STOP"
    MBP  -> Machine type B slots, e.g. "XXBX" or "STOP"
    CTL  -> competition time left, e.g. "T532"
    PING -> "PONG"

Part letters:

    B: final part, should go to the outgoing warehouse
    G: intermediate part, should go to machine type B
    R: raw part, should go to machine type A
    X: empty slot
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Optional, Tuple


REQUEST_INCOMING = 'IWP'
REQUEST_OUTGOING = 'OWP'
REQUEST_MACHINE_A = 'MAP'
REQUEST_MACHINE_B = 'MBP'
REQUEST_TIME_LEFT = 'CTL'
REQUEST_PING = 'PING'

RESPONSE_STOP = 'STOP'
RESPONSE_PONG = 'PONG'

PART_FINAL = 'B'
PART_INTERMEDIATE = 'G'
PART_RAW = 'R'
PART_EMPTY = 'X'

PART_TYPES = {PART_FINAL, PART_INTERMEDIATE, PART_RAW}
SLOT_VALUES = PART_TYPES | {PART_EMPTY}
PART_RESPONSE_REQUESTS = {
    REQUEST_INCOMING,
    REQUEST_OUTGOING,
    REQUEST_MACHINE_A,
    REQUEST_MACHINE_B,
}


@dataclass(frozen=True)
class FactorySnapshot:
    """Immutable view of the four server-managed slot groups."""

    incoming: Tuple[str, str, str, str] = (PART_EMPTY,) * 4
    outgoing: Tuple[str, str, str, str] = (PART_EMPTY,) * 4
    machine_a: Tuple[str, str, str, str] = (PART_EMPTY,) * 4
    machine_b: Tuple[str, str, str, str] = (PART_EMPTY,) * 4

    def update(self, request_code: str, slots: Tuple[str, str, str, str]) -> 'FactorySnapshot':
        """Return a new snapshot with one zone replaced from a server response."""
        if request_code == REQUEST_INCOMING:
            return replace(self, incoming=slots)
        if request_code == REQUEST_OUTGOING:
            return replace(self, outgoing=slots)
        if request_code == REQUEST_MACHINE_A:
            return replace(self, machine_a=slots)
        if request_code == REQUEST_MACHINE_B:
            return replace(self, machine_b=slots)
        raise ValueError(f'cannot update snapshot from request {request_code}')

    def with_slot(self, zone: str, slot: int, value: str) -> 'FactorySnapshot':
        """Return a new snapshot with a single slot changed locally."""
        if slot < 1 or slot > 4:
            raise ValueError(f'slot must be 1..4, got {slot}')
        if value not in SLOT_VALUES:
            raise ValueError(f'unsupported slot value {value!r}')

        field_name = zone_to_snapshot_field(zone)
        current = list(getattr(self, field_name))
        current[slot - 1] = value
        return replace(self, **{field_name: tuple(current)})

    def to_dict(self) -> dict:
        """Serialize tuples back into compact 4-character strings."""
        return {
            'incoming': ''.join(self.incoming),
            'outgoing': ''.join(self.outgoing),
            'machine_a': ''.join(self.machine_a),
            'machine_b': ''.join(self.machine_b),
        }


def parse_server_response(request_code: str, payload: bytes | str):
    """Parse a UDP response for one request.

    Returns one of:
      - "STOP" for a pre-run STOP response
      - tuple[str, str, str, str] for part-slot responses
      - int seconds for CTL
      - "PONG" for PING
    """
    text = decode_text(payload)

    if text == RESPONSE_STOP:
        return RESPONSE_STOP

    if request_code in PART_RESPONSE_REQUESTS:
        return parse_part_slots(text)

    if request_code == REQUEST_TIME_LEFT:
        return parse_time_left(text)

    if request_code == REQUEST_PING:
        if text != RESPONSE_PONG:
            raise ValueError(f'expected PONG, got {text!r}')
        return RESPONSE_PONG

    raise ValueError(f'unknown request code {request_code!r}')


def decode_text(payload: bytes | str) -> str:
    """Decode bytes, trim whitespace, and normalize to upper-case ASCII-ish text."""
    if isinstance(payload, bytes):
        text = payload.decode('utf-8', errors='replace')
    else:
        text = payload
    return text.strip().upper()


def parse_part_slots(text: str) -> Tuple[str, str, str, str]:
    """Validate a 4-slot response such as 'BGXR'."""
    text = text.strip().upper()
    if len(text) != 4:
        raise ValueError(f'part response must have exactly 4 letters, got {text!r}')

    invalid = [letter for letter in text if letter not in SLOT_VALUES]
    if invalid:
        raise ValueError(f'invalid part letters in response {text!r}')

    return tuple(text)  # type: ignore[return-value]


def parse_time_left(text: str) -> int:
    """Parse a time-left response like 'T532' into seconds."""
    text = text.strip().upper()
    if not text.startswith('T'):
        raise ValueError(f'time-left response must start with T, got {text!r}')
    try:
        return int(text[1:])
    except ValueError as exc:
        raise ValueError(f'invalid time-left response {text!r}') from exc


def target_zone_for_part(part_type: str) -> Optional[str]:
    """Map a part type to the next destination zone in the factory."""
    part_type = part_type.upper()
    if part_type == PART_FINAL:
        return 'outgoing'
    if part_type == PART_INTERMEDIATE:
        return 'machine_b'
    if part_type == PART_RAW:
        return 'machine_a'
    if part_type == PART_EMPTY:
        return None
    raise ValueError(f'unsupported part type {part_type!r}')


def zone_to_snapshot_field(zone: str) -> str:
    """Translate public zone names into FactorySnapshot attribute names."""
    zones = {
        'incoming': 'incoming',
        'outgoing': 'outgoing',
        'machine_a': 'machine_a',
        'machine_b': 'machine_b',
    }
    try:
        return zones[zone]
    except KeyError as exc:
        raise ValueError(f'unknown factory zone {zone!r}') from exc
