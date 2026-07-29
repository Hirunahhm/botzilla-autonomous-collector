"""
rplidar_driver.py.

Minimal RPLIDAR C1 serial driver (raw legacy SCAN protocol, no ROS imports) plus a pure
function to assemble parsed scan points into a fixed-size, angle-binned array suitable
for publishing as sensor_msgs/LaserScan. See rplidar_node.py for the ROS2 wrapper.

Why this exists instead of the upstream ros-jazzy-rplidar-ros package: on this exact
RPLIDAR C1 unit + Jetson Orin Nano combination, rplidar_composition (SDK 1.12.0)
reliably fails to start scanning (repeatable '80008002' timeout on the majority of
attempts, with the motor confirmed not spinning) while this driver, talking the legacy
SCAN command directly over pyserial, has completed multiple 30+ second soak tests with
zero malformed packets on the same hardware/port. dmesg showed no USB disconnect,
reset, or over-current events during the failing rplidar_ros attempts, and the same
unit was separately verified fault-free on a laptop, ruling out both a bad device and a
Jetson USB power problem — the fault is isolated to that package's scan-start sequence.
"""
from collections import namedtuple
import math
import time

import serial

DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUD = 460800

# Longest gap between complete scan packets tolerated before iter_scans() gives up and
# raises, so the caller can reconnect (which resyncs via CMD_STOP + buffer flush).
# The C1 streams ~3300 packets/s when healthy, so seconds of silence is never normal.
DATA_TIMEOUT_S = 5.0

CMD_STOP = bytes([0xA5, 0x25])
CMD_SCAN = bytes([0xA5, 0x20])
CMD_GET_INFO = bytes([0xA5, 0x50])
CMD_GET_HEALTH = bytes([0xA5, 0x52])

RESP_DESCRIPTOR_SYNC = bytes([0xA5, 0x5A])

HEALTH_STATUS = {0: 'Good', 1: 'Warning', 2: 'Error'}

ScanPoint = namedtuple(
    'ScanPoint', ['valid', 'new_revolution', 'quality', 'angle_deg', 'distance_mm']
)


def parse_scan_point(packet):
    """Decode one 5-byte RPLIDAR scan packet into a ScanPoint."""
    if len(packet) != 5:
        raise ValueError(f'scan packet must be 5 bytes, got {len(packet)}')

    b0, b1, b2, b3, b4 = packet
    start_flag = b0 & 0x1
    inv_start_flag = (b0 >> 1) & 0x1
    quality = b0 >> 2
    check_bit = b1 & 0x1
    angle_q6 = (b1 >> 1) | (b2 << 7)
    distance_q2 = b3 | (b4 << 8)

    valid = (start_flag != inv_start_flag) and (check_bit == 1)
    angle_deg = angle_q6 / 64.0
    distance_mm = distance_q2 / 4.0

    return ScanPoint(
        valid=valid, new_revolution=bool(start_flag), quality=quality,
        angle_deg=angle_deg, distance_mm=distance_mm,
    )


def parse_info_payload(data):
    """Decode the 20-byte GET_INFO payload into a dict."""
    if len(data) != 20:
        raise ValueError(f'info payload must be 20 bytes, got {len(data)}')
    return {
        'model': data[0],
        'firmware_major': data[2],
        'firmware_minor': data[1],
        'hardware': data[3],
        'serial_number': data[4:20][::-1].hex(),
    }


def parse_health_payload(data):
    """Decode the 3-byte GET_HEALTH payload into a dict."""
    if len(data) != 3:
        raise ValueError(f'health payload must be 3 bytes, got {len(data)}')
    status_code = data[0]
    error_code = data[1] | (data[2] << 8)
    return {
        'status_code': status_code,
        'status_text': HEALTH_STATUS.get(status_code, 'Unknown'),
        'error_code': error_code,
    }


def bin_scan_points(points, num_samples, range_min_m, range_max_m):
    """
    Bin ScanPoints spanning ~one revolution into a fixed-size (ranges, intensities) pair.

    Bin i corresponds to bearing +i * 360/num_samples degrees COUNTER-CLOCKWISE, which
    is what a LaserScan published with angle_min=0 and a positive angle_increment means
    under REP-103. A bin with no valid in-range point is left as float('inf')
    (REP-117: no obstacle detected), matching what Nav2/RTAB-Map expect from
    sensor_msgs/LaserScan. When multiple points land in the same bin, the closest wins.

    The RPLIDAR reports angle_deg increasing CLOCKWISE, so it is negated here. Binning
    it directly (the original behaviour) published a mirror image of the room: harmless
    while stationary, but during a turn the mirrored pattern rotates the SAME way as the
    robot instead of opposite to it, so after the TF rotation is applied the walls sweep
    at twice the turn rate. Measured on this hardware before the fix: transforming two
    scans 20.2 deg apart into the odom frame left a -42.0 deg residual, i.e. -2x the
    rotation, which is the signature of a sign-flipped scan angle. That is what drew the
    rotating "fan" of duplicated walls in the occupancy grid.
    """
    ranges = [float('inf')] * num_samples
    intensities = [0.0] * num_samples
    bin_width_deg = 360.0 / num_samples

    for p in points:
        if not p.valid or p.distance_mm <= 0.0:
            continue
        distance_m = p.distance_mm / 1000.0
        if distance_m < range_min_m or distance_m > range_max_m:
            continue
        # floor(), not int(): int() truncates toward zero, which would fold negative
        # angles onto the wrong bin either side of 0.
        i = int(math.floor(-p.angle_deg / bin_width_deg)) % num_samples
        if distance_m < ranges[i]:
            ranges[i] = distance_m
            intensities[i] = float(p.quality)

    return ranges, intensities


class RPLidarError(RuntimeError):
    """Raised for RPLIDAR protocol errors (bad response descriptor, malformed payload)."""


class RPLidar:
    """Blocking pyserial driver for the RPLIDAR C1, legacy SCAN protocol."""

    def __init__(self, port=DEFAULT_PORT, baudrate=DEFAULT_BAUD, timeout=2):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._ser = None
        self._scanning = False

    def connect(self):
        self._ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        self._ser.write(CMD_STOP)
        time.sleep(0.05)
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()

    def disconnect(self):
        if self._ser is None:
            return
        try:
            if self._scanning:
                self.stop_scan()
        finally:
            self._ser.close()
            self._ser = None

    def _read_descriptor(self):
        descriptor = self._ser.read(7)
        if len(descriptor) < 7 or descriptor[0:2] != RESP_DESCRIPTOR_SYNC:
            raise RPLidarError(f'invalid response descriptor: {descriptor.hex()}')
        return descriptor

    def get_info(self):
        self._ser.reset_input_buffer()
        self._ser.write(CMD_GET_INFO)
        self._ser.flush()
        self._read_descriptor()
        return parse_info_payload(self._ser.read(20))

    def get_health(self):
        self._ser.reset_input_buffer()
        self._ser.write(CMD_GET_HEALTH)
        self._ser.flush()
        self._read_descriptor()
        return parse_health_payload(self._ser.read(3))

    def start_motor(self, spin_up_delay=2.0):
        self._ser.dtr = False
        if spin_up_delay:
            time.sleep(spin_up_delay)

    def stop_motor(self):
        self._ser.dtr = True

    def iter_scans(self):
        """Start scanning; yield ScanPoints until stop_scan() is called.

        Raises RPLidarError if no complete packet arrives for DATA_TIMEOUT_S. Without
        that check a device which accepted CMD_SCAN but then stopped streaming (motor
        not spinning, or the link left mid-stream by an unclean shutdown) makes this
        loop spin on read timeouts forever: no data, no exception, so the caller's
        reconnect path never runs and the node hangs silently with /scan simply absent.
        """
        self._ser.reset_input_buffer()
        self._ser.write(CMD_SCAN)
        self._ser.flush()
        self._read_descriptor()
        self._scanning = True

        last_packet_at = time.monotonic()
        while self._scanning:
            packet = self._ser.read(5)
            if len(packet) < 5:
                stalled_for = time.monotonic() - last_packet_at
                if stalled_for > DATA_TIMEOUT_S:
                    raise RPLidarError(
                        f'no scan data for {stalled_for:.1f}s after CMD_SCAN — '
                        f'device stalled or stream desynchronised'
                    )
                continue
            last_packet_at = time.monotonic()
            yield parse_scan_point(packet)

    def stop_scan(self):
        self._scanning = False
        if self._ser is not None:
            self._ser.write(CMD_STOP)
            time.sleep(0.05)

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
