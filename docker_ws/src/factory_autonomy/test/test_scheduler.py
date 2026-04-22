from factory_autonomy.crate_protocol import FactorySnapshot
from factory_autonomy.scheduler import FactoryPlanner


def test_final_part_goes_to_outgoing():
    planner = FactoryPlanner()
    snapshot = FactorySnapshot(incoming=tuple('BXXX'), outgoing=tuple('XXXX'))

    task = planner.choose_next(snapshot)

    assert task.part_type == 'B'
    assert task.source.label() == 'incoming:1'
    assert task.target.label() == 'outgoing:1'


def test_green_part_goes_to_machine_b_input():
    planner = FactoryPlanner()
    snapshot = FactorySnapshot(incoming=tuple('GXXX'), machine_b=tuple('XXXX'))

    task = planner.choose_next(snapshot)

    assert task.part_type == 'G'
    assert task.source.label() == 'incoming:1'
    assert task.target.label() == 'machine_b:1'


def test_red_part_goes_to_machine_a_input():
    planner = FactoryPlanner()
    snapshot = FactorySnapshot(incoming=tuple('XXRX'), machine_a=tuple('XXXX'))

    task = planner.choose_next(snapshot)

    assert task.part_type == 'R'
    assert task.source.label() == 'incoming:3'
    assert task.target.label() == 'machine_a:1'


def test_raw_part_has_priority_over_green_part():
    planner = FactoryPlanner()
    snapshot = FactorySnapshot(
        incoming=tuple('GXRX'),
        machine_a=tuple('XXXX'),
        machine_b=tuple('XXXX'),
    )

    task = planner.choose_next(snapshot)

    assert task.part_type == 'R'
    assert task.source.label() == 'incoming:3'
    assert task.target.label() == 'machine_a:1'


def test_green_part_has_priority_over_machine_output():
    planner = FactoryPlanner()
    snapshot = FactorySnapshot(
        incoming=tuple('GXXX'),
        machine_b=tuple('XBXX'),
    )

    task = planner.choose_next(snapshot)

    assert task.part_type == 'G'
    assert task.source.label() == 'incoming:1'
    assert task.target.label() == 'machine_b:3'


def test_machine_output_is_unloaded_before_incoming_final_part():
    planner = FactoryPlanner()
    snapshot = FactorySnapshot(
        incoming=tuple('BXXX'),
        outgoing=tuple('XXXX'),
        machine_b=tuple('XBXX'),
    )

    task = planner.choose_next(snapshot)

    assert task.part_type == 'B'
    assert task.source.label() == 'machine_b:2'
    assert task.target.label() == 'outgoing:1'


def test_machine_input_is_available_only_when_pair_output_is_empty():
    planner = FactoryPlanner()
    snapshot = FactorySnapshot(
        incoming=tuple('RXXX'),
        machine_a=tuple('RXXX'),
    )

    task = planner.choose_next(snapshot)

    assert task.part_type == 'R'
    assert task.target.label() == 'machine_a:3'
