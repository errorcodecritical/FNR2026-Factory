from factory_autonomy.crate_protocol import (
    REQUEST_INCOMING,
    REQUEST_PING,
    REQUEST_TIME_LEFT,
    RESPONSE_PONG,
    RESPONSE_STOP,
    parse_server_response,
    target_zone_for_part,
)


def test_parse_part_response():
    response = parse_server_response(REQUEST_INCOMING, b'BGGR')

    assert response == ('B', 'G', 'G', 'R')


def test_parse_stop_response():
    response = parse_server_response(REQUEST_INCOMING, b'STOP')

    assert response == RESPONSE_STOP


def test_parse_time_left_response():
    response = parse_server_response(REQUEST_TIME_LEFT, b'T532')

    assert response == 532


def test_parse_ping_response():
    response = parse_server_response(REQUEST_PING, b'PONG')

    assert response == RESPONSE_PONG


def test_part_target_zones():
    assert target_zone_for_part('B') == 'outgoing'
    assert target_zone_for_part('G') == 'machine_b'
    assert target_zone_for_part('R') == 'machine_a'
