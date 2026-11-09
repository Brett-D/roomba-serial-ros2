import struct

import pytest

from roomba_driver import oi


def test_group_sizes_match_spec():
    sizes = {g: oi.packet_size(g) for g in (0, 1, 2, 3, 4, 5, 6, 100, 101, 106, 107)}
    assert sizes == {0: 26, 1: 10, 2: 6, 3: 10, 4: 14, 5: 12, 6: 52, 100: 80, 101: 28, 106: 12, 107: 9}


def test_decode_battery_packet_captured_from_robot():
    v = oi.decode(3, bytes.fromhex('043967018e12089a0a88'))
    assert v == {'charging_state': 4, 'voltage_mv': 14695, 'current_ma': 398, 'temperature_c': 18,
                 'charge_mah': 2202, 'capacity_mah': 2696}


def test_decode_bumps_and_cliffs_captured_while_lifted():
    v = oi.decode(1, bytes.fromhex('0d000101010100800000'))
    assert v['wheel_drop_left'] and v['wheel_drop_right'] and v['bump_right']
    assert not v['bump_left'] and not v['wall'] and not v['virtual_wall']
    assert v['cliff_left'] and v['cliff_front_left'] and v['cliff_front_right'] and v['cliff_right']
    assert not any(v[k] for k in ('overcurrent_left_wheel', 'overcurrent_right_wheel',
                                  'overcurrent_main_brush', 'overcurrent_side_brush'))


def test_decode_group_100_has_every_sensor_key():
    v = oi.decode(100, bytes(80))
    assert {'left_encoder_counts', 'oi_mode', 'stasis', 'light_bump_right_signal',
            'button_clock', 'home_base_available', 'side_brush_current_ma'} <= set(v)
    assert len(v) == 68


def test_decode_rejects_wrong_length():
    with pytest.raises(ValueError):
        oi.decode(3, bytes(9))


def test_stasis_uses_bit_zero_only():
    assert oi.decode(58, b'\x01') == {'stasis': True}
    assert oi.decode(58, b'\x02') == {'stasis': False}


def test_stream_parser_resyncs_and_splits():
    frame = bytes.fromhex('130807012bf9fe2c022568')
    parser = oi.StreamParser([7, 43, 44])
    assert parser.feed(b'\x00\x13\xff' + frame[:5]) == []
    frames = parser.feed(frame[5:] + frame)
    assert len(frames) == 2
    assert frames[0]['left_encoder_counts'] == 63998
    assert frames[0]['right_encoder_counts'] == 549
    assert frames[0]['bump_right'] and not frames[0]['bump_left']


def test_stream_parser_rejects_bad_checksum():
    frame = bytearray.fromhex('130807012bf9fe2c022568')
    frame[-1] ^= 0x01
    parser = oi.StreamParser([7, 43, 44])
    assert parser.feed(bytes(frame)) == []
    assert parser.bad_frames == 1


def _frame(payload):
    body = bytes([oi.STREAM_HEADER, len(payload)]) + bytes(payload)
    return body + bytes([-sum(body) & 0xFF])


def test_stream_parser_accepts_alternating_layouts():
    a = bytes.fromhex('130807012bf9fe2c022568')          # [7, 43, 44]
    b = _frame([35, 2, 58, 1])                            # [35, 58]
    parser = oi.StreamParser([7, 43, 44], [35, 58])
    frames = parser.feed_frames(a + b + a)
    assert [n for n, _ in frames] == [8, 4, 8]
    assert frames[1][1] == {'oi_mode': 2, 'stasis': True}


def test_driver_stream_layouts_cover_every_sensor_and_fit():
    covered = set()
    for layout in (oi.STREAM_PRIMARY, oi.STREAM_SECONDARY):
        assert oi.stream_payload_length(layout) + 3 <= 75
        for pid in layout:
            covered.update(oi.packet_ids(pid))
        ids = {i for pid in layout for i in oi.packet_ids(pid)}
        assert {7, 9, 10, 11, 12, 35, 43, 44} <= ids   # safety + odometry in every frame
    assert covered == set(oi.GROUPS[100])


def test_drive_direct_sends_right_wheel_first_and_clamps():
    assert oi.drive_direct(-100, 200) == bytes([145]) + struct.pack('>hh', 200, -100)
    assert oi.drive_direct(900, -900) == bytes([145]) + struct.pack('>hh', -500, 500)


def test_drive_special_radii_and_clamping():
    assert oi.drive(100, oi.RADIUS_STRAIGHT) == bytes([137]) + struct.pack('>hh', 100, 32767)
    assert oi.drive(100, 32768) == bytes([137]) + struct.pack('>hh', 100, 32767)
    assert oi.drive(-900, -5000) == bytes([137]) + struct.pack('>hh', -500, -2000)
    assert oi.drive(50, oi.RADIUS_TURN_CW) == bytes([137]) + struct.pack('>hh', 50, -1)


def test_drive_pwm_and_cleaning_motors():
    assert oi.drive_pwm(-300, 100) == bytes([146]) + struct.pack('>hh', 100, -255)
    assert oi.motors(True, True, True, True, True) == bytes([138, 0b11111])
    assert oi.motors(vacuum=True) == bytes([138, 0b00010])
    assert oi.pwm_motors(-200, 50, 300) == bytes([144]) + struct.pack('>bbB', -127, 50, 127)


def test_leds_and_scheduling_leds():
    assert oi.leds(debris=True, check_robot=True, power_color=255, power_intensity=128) == \
        bytes([139, 0b1001, 255, 128])
    assert oi.scheduling_leds(weekdays=0b1000001, schedule=True, colon=True) == bytes([162, 0b1000001, 0b10001])


def test_display():
    assert oi.digit_leds_ascii('HELO') == bytes([164]) + b'HELO'
    assert oi.digit_leds_ascii('Hi') == bytes([164]) + b'Hi  '
    assert oi.digit_leds_ascii('\x01ABCDE') == bytes([164]) + b' ABC'
    assert oi.digit_leds_raw([0x7F, 0, 1, 0xFF]) == bytes([163, 0x7F, 0, 1, 0x7F])
    with pytest.raises(ValueError):
        oi.digit_leds_raw([1, 2, 3])


def test_song_and_play_validation():
    assert oi.song(2, [(60, 16), (64, 32)]) == bytes([140, 2, 2, 60, 16, 64, 32])
    assert oi.play(4) == bytes([141, 4])
    for bad in (lambda: oi.song(5, [(60, 16)]), lambda: oi.song(0, []),
                lambda: oi.song(0, [(60, 1)] * 17), lambda: oi.play(5)):
        with pytest.raises(ValueError):
            bad()


def test_schedule_clock_buttons_baud():
    assert oi.schedule(0b0111110, [(9, 30)] * 7) == bytes([167, 0b0111110] + [9, 30] * 7)
    assert oi.set_day_time(3, 14, 5) == bytes([168, 3, 14, 5])
    assert oi.buttons(0b101) == bytes([165, 5])
    assert oi.baud(115200) == bytes([129, 11])
    for bad in (lambda: oi.schedule(1, [(24, 0)] * 7), lambda: oi.set_day_time(7, 0, 0),
                lambda: oi.buttons(256), lambda: oi.baud(1000)):
        with pytest.raises(ValueError):
            bad()


def test_stream_and_query_requests():
    assert oi.stream([100]) == bytes([148, 1, 100])
    assert oi.query_list([43, 44]) == bytes([149, 2, 43, 44])
    assert oi.pause_resume_stream(False) == bytes([150, 0])
    assert oi.stream_payload_length([100]) == 81
    with pytest.raises(ValueError):
        oi.stream([99])
