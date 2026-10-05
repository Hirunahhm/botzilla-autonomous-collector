from botzilla_fleet.remote_detection import DepthBuffer, RelayRate
from botzilla_perception.cube_depth import closest_cube, depth_at
import numpy as np


def test_nearest_within_tolerance():
    b = DepthBuffer()
    for i in range(10):
        b.add(i * 0.05, i, 'mono8')
    img, enc, off = b.nearest(0.21)
    assert img == 4 and abs(off - 0.01) < 1e-9
    assert b.nearest(5.0) is None


def test_buffer_drops_old_frames():
    b = DepthBuffer(keep_s=1.0)
    for i in range(40):
        b.add(i * 0.1, i, 'mono8')
    assert len(b) <= 11


def test_relay_rate():
    r = RelayRate(8.0)
    passed = [t for t in np.arange(0, 1.0, 0.033) if r.due(float(t))]
    assert 7 <= len(passed) <= 9


def test_depth_metres_32fc1_and_blind_spot_choice():
    d = np.full((480, 640), 0.8, np.float32)
    d[200:260, 500:560] = 0.4                       # second cube inside the blind spot
    assert abs(depth_at(d, 320, 240, '32FC1') - 0.8) < 1e-6
    # A blind-spot cube counts as nearest (it is the one being captured): z = 0.
    x, z = closest_cube([(320, 240), (530, 230)], d, '32FC1', 640)
    assert z == 0.0 and x > 0.6
    x, z = closest_cube([(320, 240)], d, '32FC1', 640)
    assert abs(z - 0.8) < 1e-6 and abs(x) < 1e-9
    assert closest_cube([], d, '32FC1', 640) is None


def test_mono8_kinect_conversion_matches_formula():
    raw = 100                                        # mono8 value, ~1.16 m
    d = np.full((480, 640), raw, np.uint8)
    expected = 1.0 / ((raw / 255.0) * 2047.0 * -0.0030711016 + 3.3309495161)
    assert abs(depth_at(d, 100, 100, 'mono8') - expected) < 1e-6
    d[:] = 255                                       # Kinect no-data
    assert depth_at(d, 100, 100, 'mono8') is None
