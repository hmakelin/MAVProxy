'''Vehicle-mounted camera orientation and switching back to the map.'''

import math
from unittest.mock import Mock

import numpy as np
import pytest

vtk = pytest.importorskip('vtk')

from MAVProxy.modules.mavproxy_map3d.camera import TerrainCamera

HOME = (-35.363262, 149.165238, 584.0)


@pytest.mark.parametrize('attitude,direction,up', [
    ((0, 0, 0), (0, 0, -1), (0, 1, 0)),
    ((0, 0, 90), (0, 0, -1), (1, 0, 0)),
    ((90, 0, 0), (-1, 0, 0), (0, 1, 0)),
    ((-90, 0, 0), (1, 0, 0), (0, 1, 0)),
    ((0, 90, 0), (0, 1, 0), (0, 0, 1)),
    ((0, -90, 0), (0, -1, 0), (0, 0, -1)),
    ((180, 0, 0), (0, 0, 1), (0, 1, 0)),
])
def test_down_view_body_axes(attitude, direction, up):
    camera = vtk.vtkCamera()
    tc = TerrainCamera(camera, (0, 0, 0), 100)
    position = (10, 20, 120)
    tc.set_down(position, *map(math.radians, attitude))
    assert camera.GetPosition() == position
    assert camera.GetDirectionOfProjection() == pytest.approx(direction)
    assert camera.GetViewUp() == pytest.approx(up)
    assert tc.pos == position
    assert tc.focal == pytest.approx(camera.GetFocalPoint())
    assert camera.GetClippingRange() == pytest.approx((0.5, 500000))


@pytest.mark.parametrize('attitude', [
    (25, 40, 135), (-70, -35, 250), (120, 89.99, -45),
    (32, 90, 75), (-115, -90, 20), (170, 130, -120),
])
def test_down_view_combined_attitude(attitude):
    roll, pitch, yaw = map(math.radians, attitude)
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    rx = np.array(((1, 0, 0), (0, cr, -sr), (0, sr, cr)))
    ry = np.array(((cp, 0, sp), (0, 1, 0), (-sp, 0, cp)))
    rz = np.array(((cy, -sy, 0), (sy, cy, 0), (0, 0, 1)))
    ned_to_enu = np.array(((0, 1, 0), (1, 0, 0), (0, 0, -1)))
    rotation = ned_to_enu @ rz @ ry @ rx
    camera = vtk.vtkCamera()
    tc = TerrainCamera(camera, (0, 0, 0), 100)
    tc.set_down((10, 20, 120), roll, pitch, yaw)
    direction = np.array(camera.GetDirectionOfProjection())
    up = np.array(camera.GetViewUp())
    assert direction == pytest.approx(rotation @ (0, 0, 1))
    assert up == pytest.approx(rotation @ (1, 0, 0))
    assert np.cross(direction, up) == pytest.approx(rotation @ (0, 1, 0))
    assert np.linalg.norm(direction) == pytest.approx(1)
    assert np.linalg.norm(up) == pytest.approx(1)
    assert np.dot(direction, up) == pytest.approx(0, abs=1e-12)


def scene(map3d_frame):
    frame = map3d_frame()
    frame.init_scene(*HOME)
    frame.handle(('vehicle', *HOME, 0, 0, 0))
    frame.follow_button.Enable = Mock()
    return frame


def test_switching_views_restores_original_map(map3d_frame):
    frame = scene(map3d_frame)
    camera = frame.ren.GetActiveCamera()
    original = (camera.GetPosition(), camera.GetFocalPoint(),
                camera.GetViewUp(), camera.GetViewAngle())
    frame.fpv_button.SetValue(True)
    frame.on_fpv_toggle(None)
    saved = frame.saved_map_view
    assert frame.fpv_button.GetValue()
    assert not frame.down_button.GetValue()
    assert camera.GetDirectionOfProjection() == pytest.approx((0, 1, 0))
    assert camera.GetViewUp() == pytest.approx((0, 0, 1))

    frame.down_button.SetValue(True)
    frame.on_down_toggle(None)
    assert not frame.fpv_button.GetValue()
    assert frame.down_button.GetValue()
    assert frame.saved_map_view == saved
    assert not frame.style.interaction_enabled
    assert not frame.elements.vehicle_visible
    frame.follow_button.Enable.assert_called_with(False)
    assert camera.GetDirectionOfProjection() == pytest.approx((0, 0, -1))

    frame.fpv_button.SetValue(True)
    frame.on_fpv_toggle(None)
    assert frame.fpv_button.GetValue()
    assert not frame.down_button.GetValue()
    assert frame.saved_map_view == saved
    assert camera.GetDirectionOfProjection() == pytest.approx((0, 1, 0))

    frame.down_button.SetValue(True)
    frame.on_down_toggle(None)
    frame.down_button.SetValue(False)
    frame.on_down_toggle(None)
    assert not frame.fpv_enabled
    assert not frame.down_enabled
    assert frame.style.interaction_enabled
    assert frame.elements.vehicle_visible
    frame.follow_button.Enable.assert_called_with(True)
    assert frame.saved_map_view is None
    for actual, expected in zip(
            (camera.GetPosition(), camera.GetFocalPoint(),
             camera.GetViewUp(), camera.GetViewAngle()), original):
        assert actual == pytest.approx(expected)


@pytest.mark.parametrize('before_scene', [False, True])
def test_down_view_tracks_telemetry_and_fov(map3d_frame, before_scene):
    frame = map3d_frame()
    if not before_scene:
        frame.init_scene(*HOME)
    frame.set_fpv_enabled(True, down=True)
    if before_scene:
        frame.init_scene(*HOME)
    assert not frame.style.interaction_enabled
    assert not frame.elements.vehicle_visible
    for attitude in ((0, 0, 0), (45, -30, 120), (-90, 90, 170)):
        angles = tuple(map(math.radians, attitude))
        frame.handle(('vehicle', HOME[0], HOME[1], HOME[2] + 100, *angles))
        expected = vtk.vtkCamera()
        TerrainCamera(expected, (0, 0, 0), 100).set_down(
            frame.elements.refresh_vehicle(), *angles)
        camera = frame.ren.GetActiveCamera()
        assert camera.GetPosition() == pytest.approx(expected.GetPosition())
        assert camera.GetDirectionOfProjection() == pytest.approx(
            expected.GetDirectionOfProjection())
        assert camera.GetViewUp() == pytest.approx(expected.GetViewUp())
    frame.handle(('fpvfov', 65))
    assert camera.GetViewAngle() == 65
    position, focal = camera.GetPosition(), camera.GetFocalPoint()
    frame.handle(('lookat', *HOME, 800))
    frame.handle(('center',))
    frame.set_follow_enabled(True)
    assert camera.GetPosition() == position
    assert camera.GetFocalPoint() == focal
    frame.set_follow_enabled(False)
    frame.set_fpv_enabled(False)
    assert camera.GetViewUp() == pytest.approx((0, 0, 1))
    assert camera.GetViewAngle() == 30
