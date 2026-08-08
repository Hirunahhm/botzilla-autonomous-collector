"""Pure unit tests for KobukiDriver's gyro byte decoding — no hardware, no ROS.

Regression test for a real bug: gyro_velocity_data() used to scale the low and high
bytes of each 16-bit gyro sample independently instead of combining them into one
signed value. Near-zero rotation this looked like ordinary sensor noise; during a real
turn the high byte dominates and the reported rate diverged sharply from the true one,
corrupting heading estimation. See gyro_bytes_to_signed_int16() in KobukiDriver.py.
"""

from botzilla_control.KobukiDriver import gyro_bytes_to_signed_int16


def test_zero():
    assert gyro_bytes_to_signed_int16([0, 0]) == 0


def test_low_byte_only_positive():
    assert gyro_bytes_to_signed_int16([1, 0]) == 1
    assert gyro_bytes_to_signed_int16([255, 0]) == 255


def test_high_byte_contributes_full_weight():
    # High byte must weight 256x the low byte, not be read as an independent value.
    assert gyro_bytes_to_signed_int16([0, 1]) == 256
    assert gyro_bytes_to_signed_int16([1, 1]) == 257


def test_max_positive_int16():
    assert gyro_bytes_to_signed_int16([0xFF, 0x7F]) == 32767


def test_min_negative_int16():
    assert gyro_bytes_to_signed_int16([0x00, 0x80]) == -32768


def test_negative_small_magnitude():
    # Two's complement: -1 and -2 as 16-bit values.
    assert gyro_bytes_to_signed_int16([0xFF, 0xFF]) == -1
    assert gyro_bytes_to_signed_int16([0xFE, 0xFF]) == -2


def test_negative_large_magnitude_needs_correct_high_byte():
    # -1000 as int16 = 0xFC18 little-endian -> [0x18, 0xFC]. The old bug would have
    # scaled 0x18 (24) and 0xFC (252) as two separate small positive-ish "readings"
    # instead of the true large negative rate.
    raw = (-1000) & 0xFFFF
    lo, hi = raw & 0xFF, (raw >> 8) & 0xFF
    assert gyro_bytes_to_signed_int16([lo, hi]) == -1000
