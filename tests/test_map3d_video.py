'''Mounted-camera rendering, output framing and virtual-device lifecycle.'''

import math
import os
import queue
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

vtk = pytest.importorskip('vtk')

from MAVProxy.modules.mavproxy_map3d import video  # noqa: E402
from MAVProxy.modules.mavproxy_map3d.camera import TerrainCamera  # noqa: E402
from MAVProxy.modules.mavproxy_map3d.map3d import Map3D  # noqa: E402

HOME = (-35.363262, 149.165238, 584.0)
CONFIG = ('/dev/video10', 64, 48, 20.0, 'down')


@pytest.fixture
def backend(monkeypatch):
    camera = Mock()
    camera.send.side_effect = lambda frame: camera.frames.append(frame.copy())
    camera.frames = []
    factory = Mock(return_value=camera)
    monkeypatch.setitem(sys.modules, 'pyvirtualcam', SimpleNamespace(
        Camera=factory, PixelFormat=SimpleNamespace(RGB='RGB')))
    terrain = Mock(lat0=HOME[0], lon0=HOME[1])
    monkeypatch.setattr(video, 'TerrainManager', Mock(return_value=terrain))
    return factory, camera, terrain


@pytest.mark.parametrize('view', ['down', 'fpv'])
def test_real_render_rgb_orientation_and_cadence(backend, monkeypatch, view):
    if sys.platform.startswith('linux') and not os.environ.get('DISPLAY'):
        pytest.skip('VTK offscreen test needs an X display on older VTK builds')
    factory, camera, terrain = backend
    clock = Mock(return_value=10.0)
    monkeypatch.setattr(video.time, 'monotonic', clock)
    output = video.VideoOutput()
    try:
        output.open(CONFIG[:-1] + (view,), None, *HOME[:2], 1.0)
        factory.assert_called_once_with(
            width=64, height=48, fps=20.0, device='/dev/video10',
            fmt='RGB', backend='v4l2loopback')
        # A known bottom-red/top-blue background detects upside-down frames
        # and RGB/BGR swaps through the actual VTK readback path.
        output.renderer.SetBackground(1, 0, 0)
        output.renderer.SetBackground2(0, 0, 1)
        output.renderer.GradientBackgroundOn()
        pose = (*HOME, 0.4, -0.3, 1.2)
        output.send(pose, 65, (1.25, True, False))
        frame = camera.frames[0]
        assert frame.shape == (48, 64, 3)
        assert frame.dtype == np.uint8
        assert frame.flags.c_contiguous
        assert frame[0, 32, 2] > 240 and frame[0, 32, 0] < 15
        assert frame[-1, 32, 0] > 240 and frame[-1, 32, 2] < 15
        assert output.renderer.GetViewProps().GetNumberOfItems() == 0
        expected = vtk.vtkCamera()
        tc = TerrainCamera(expected, (0, 0, 0), 100)
        if view == 'down':
            tc.set_down((0, 0, HOME[2]), *pose[3:])
        else:
            tc.set_fpv((0, 0, HOME[2]), pose[5], pose[4], roll=pose[3])
        assert output.tc.cam.GetViewUp() == pytest.approx(expected.GetViewUp())
        assert output.tc.cam.GetDirectionOfProjection() == pytest.approx(
            expected.GetDirectionOfProjection())
        assert output.tc.cam.GetViewAngle() == 65
        clock.return_value = 10.01
        output.send(pose, 65, (1.25, True, False))
        assert len(camera.frames) == 1
        clock.return_value = 10.051
        # A second render must not return the filter's cached first image.
        output.renderer.SetBackground2(0, 1, 0)
        output.send(pose, 65, (1.25, True, False))
        assert len(camera.frames) == 2
        assert camera.frames[1][0, 32, 1] > 240
        terrain.set_render_settings.assert_called_once()
    finally:
        output.close()
    camera.close.assert_called_once()
    terrain.shutdown.assert_called_once()
    output.close()
    camera.close.assert_called_once()


@pytest.mark.parametrize('attitude', [
    (45, -30, 120), (32, 90, 75), (-115, -90, 20), (170, 130, -120),
])
def test_forward_camera_full_attitude(attitude):
    from pymavlink.rotmat import Matrix3, Vector3
    roll, pitch, yaw = map(math.radians, attitude)
    rotation = Matrix3()
    rotation.from_euler(roll, pitch, yaw)
    forward = rotation * Vector3(1, 0, 0)
    up = rotation * Vector3(0, 0, -1)
    camera = vtk.vtkCamera()
    TerrainCamera(camera, (0, 0, 0), 100).set_fpv(
        (10, 20, 120), yaw, pitch, roll=roll)
    assert camera.GetDirectionOfProjection() == pytest.approx(
        (forward.y, forward.x, -forward.z))
    assert camera.GetViewUp() == pytest.approx((up.y, up.x, -up.z))


def test_video_waits_for_scene_and_closes_on_stop(map3d_frame, monkeypatch):
    frame = map3d_frame()
    output = Mock()
    monkeypatch.setattr(video, 'VideoOutput', Mock(return_value=output))
    frame.handle(('video', CONFIG))
    assert frame.video is None
    frame.init_scene(*HOME)
    output.open.assert_called_once_with(CONFIG, None, *HOME[:2], 1.0)
    frame.handle(('video', CONFIG))
    output.open.assert_called_once()
    frame.last_vehicle_pose = (*HOME, 0.1, 0.2, 0.3)
    frame.send_video()
    output.send.assert_called_once_with(
        frame.last_vehicle_pose, 90.0, (1.25, True, False), (0.0, 0.0, 0.0))
    frame.handle(('video', None))
    output.close.assert_called_once()
    assert frame.video is None
    assert frame.video_config is None
    frame.timer.Start.assert_called_with(100)


@pytest.mark.parametrize('failure', ['open', 'send'])
def test_device_failure_reported_without_closing_map(
        map3d_frame, monkeypatch, failure):
    events = []
    frame = map3d_frame(events=events)
    frame.init_scene(*HOME)
    output = Mock()
    getattr(output, failure).side_effect = RuntimeError('device unavailable')
    monkeypatch.setattr(video, 'VideoOutput', Mock(return_value=output))
    frame.handle(('video', CONFIG))
    if failure == 'send':
        frame.last_vehicle_pose = (*HOME, 0, 0, 0)
        frame.send_video()
    assert ('video_error', 'device unavailable') in events
    assert frame.video is None
    assert frame.video_config is None
    assert frame.terrain is not None
    output.close.assert_called_once()


def test_missing_optional_dependency_is_actionable(monkeypatch):
    monkeypatch.setitem(sys.modules, 'pyvirtualcam', None)
    with pytest.raises(RuntimeError, match='MAVProxy\\[map3d-video\\]'):
        video.VideoOutput().open(CONFIG, None, *HOME[:2], 1.0)


def test_video_commands_and_view_reach_child(map3d_frame, map3d_module):
    viewer = Map3D.__new__(Map3D)
    viewer.child = SimpleNamespace(is_alive=lambda: True)
    viewer.object_queue = queue.Queue()
    module = map3d_module(viewer)
    module.cmd_map3d(['set', 'videowidth', '1280'])
    module.cmd_map3d(['set', 'videoheight', '720'])
    module.cmd_map3d(['set', 'videofps', '30'])
    module.cmd_map3d(['set', 'videoview', 'fpv'])
    while not viewer.object_queue.empty():
        viewer.object_queue.get_nowait()
    module.cmd_map3d(['video', 'start', '/dev/video42'])
    assert viewer.object_queue.get_nowait() == (
        'video', ('/dev/video42', 1280, 720, 30.0, 'fpv'))
    module.cmd_map3d(['video', 'stop'])
    assert viewer.object_queue.get_nowait() == ('video', None)
    frame = map3d_frame()
    frame.init_scene(*HOME)
    for view in ('fpv', 'down', 'map'):
        module.cmd_map3d(['view', view])
        message = viewer.object_queue.get_nowait()
        assert message == ('view', view)
        frame.handle(message)
        assert frame.fpv_enabled == (view != 'map')
        assert frame.down_enabled == (view == 'down')


def test_mount_setting_reaches_view_and_video(map3d_frame, map3d_module, capsys):
    viewer = Map3D.__new__(Map3D)
    viewer.child = SimpleNamespace(is_alive=lambda: True)
    viewer.object_queue = queue.Queue()
    module = map3d_module(viewer)
    frame = map3d_frame()
    module.cmd_map3d(['set', 'mountyaw', '90'])
    messages = []
    while not viewer.object_queue.empty():
        messages.append(viewer.object_queue.get_nowait())
    mount_message = next(m for m in messages if m[0] == 'camera_mount')
    assert mount_message == ('camera_mount', 0.0, 0.0, 90.0)
    frame.handle(mount_message)
    frame.video = Mock()
    frame.last_vehicle_pose = (*HOME, 0, 0, 0)
    frame.send_video()
    frame.video.send.assert_called_once_with(
        frame.last_vehicle_pose, 90.0, (1.25, True, False),
        pytest.approx((0, 0, math.pi / 2)))
    for bad in ('nan', 'inf', '181', 'not-a-number'):
        module.cmd_map3d(['set', 'mountyaw', bad])
        assert module.map3d_settings.mountyaw == 90
    assert capsys.readouterr().out


@pytest.mark.parametrize('width,height,hfov', [
    (640, 480, 90), (1920, 1080, 120), (480, 640, 60),
])
def test_horizontal_video_fov_uses_active_video_aspect(
        map3d_frame, width, height, hfov):
    frame = map3d_frame()
    frame.video = Mock()
    frame.video_config = ('/dev/video10', width, height, 20, 'down')
    frame.last_vehicle_pose = (*HOME, 0, 0, 0)
    frame.handle(('video_fov', hfov, 'horizontal'))
    frame.send_video()
    vfov = frame.video.send.call_args.args[1]
    assert math.tan(math.radians(vfov) / 2) * width / height == pytest.approx(
        math.tan(math.radians(hfov) / 2))
    assert height / (2 * math.tan(math.radians(vfov) / 2)) == pytest.approx(
        width / (2 * math.tan(math.radians(hfov) / 2)))


def test_video_fov_settings_live_and_default(map3d_frame, map3d_module, capsys):
    viewer = Map3D.__new__(Map3D)
    viewer.child = SimpleNamespace(is_alive=lambda: True)
    viewer.object_queue = queue.Queue()
    module = map3d_module(viewer)
    frame = map3d_frame()
    frame.video = Mock()
    frame.video_config = CONFIG
    frame.last_vehicle_pose = (*HOME, 0, 0, 0)

    def apply_commands(*commands):
        for command in commands:
            module.cmd_map3d(command)
        while not viewer.object_queue.empty():
            msg = viewer.object_queue.get_nowait()
            if msg[0] == 'video_fov':
                frame.handle(msg)

    frame.send_video()
    assert frame.video.send.call_args.args[1] == 90
    apply_commands(['set', 'videofovaxis', 'horizontal'],
                   ['set', 'videofov', '90'])
    frame.send_video()
    assert frame.video.send.call_args.args[1] == pytest.approx(73.7397952917)
    assert frame.fpv_fov == 90
    apply_commands(['set', 'videofovaxis', 'vertical'],
                   ['set', 'videofov', '75'])
    frame.send_video()
    assert frame.video.send.call_args.args[1] == 75
    for bad in ('nan', 'inf', '0.5', '180', 'not-a-number'):
        module.cmd_map3d(['set', 'videofov', bad])
        assert module.map3d_settings.videofov == 75
    assert capsys.readouterr().out
    apply_commands(['set', 'videofov', '0'])
    frame.fpv_fov = 65
    frame.send_video()
    assert frame.video.send.call_args.args[1] == 65


def test_mount_and_fov_retained_when_map_restarted(map3d_module, monkeypatch):
    from MAVProxy.modules import mavproxy_map3d
    module = map3d_module()
    module.send_fence = Mock()
    module.send_rally = Mock()
    module.send_cached_state = Mock()
    module.send_kml = Mock()
    module.cmd_map3d(['set', 'mountyaw', '90'])
    module.cmd_map3d(['set', 'videofovaxis', 'horizontal'])
    module.cmd_map3d(['set', 'videofov', '90'])
    viewer = Mock()
    viewer.is_alive.return_value = True
    monkeypatch.setattr(mavproxy_map3d, 'missing_packages', lambda: [])
    monkeypatch.setattr(mavproxy_map3d, 'resolve_tile_service',
                        lambda service: (service, None))
    monkeypatch.setattr(mavproxy_map3d, 'Map3D', Mock(return_value=viewer))
    module.start_map()
    viewer.set_camera_mount.assert_called_once_with(0, 0, 90)
    viewer.set_video_fov.assert_called_once_with(90, 'horizontal')
    viewer.set_camera_mount.reset_mock()
    viewer.set_video_fov.reset_mock()
    module.cmd_map3d(['set', 'showlabels', 'true'])
    viewer.set_camera_mount.assert_not_called()
    viewer.set_video_fov.assert_not_called()


def test_live_horizontal_video_fov_reaches_vtk(backend, monkeypatch, map3d_frame):
    if sys.platform.startswith('linux') and not os.environ.get('DISPLAY'):
        pytest.skip('VTK offscreen test needs an X display on older VTK builds')
    monkeypatch.setattr(video.time, 'monotonic', lambda: 10.0)
    output = video.VideoOutput()
    try:
        output.open(CONFIG, None, *HOME[:2], 1.0)
        frame = map3d_frame()
        frame.video = output
        frame.video_config = CONFIG
        frame.last_vehicle_pose = (*HOME, 0, 0, 0)
        frame.handle(('video_fov', 90, 'horizontal'))
        frame.send_video()
        assert output.tc.cam.GetViewAngle() == pytest.approx(73.7397952917)
        assert output.camera.frames[0].shape == (48, 64, 3)
    finally:
        output.close()


def test_unsupported_derived_vertical_fov_reports_error(map3d_frame):
    events = []
    frame = map3d_frame(events=events)
    frame.video = Mock()
    frame.video_config = ('/dev/video10', 16, 2160, 20, 'down')
    frame.last_vehicle_pose = (*HOME, 0, 0, 0)
    frame.handle(('video_fov', 179, 'horizontal'))
    frame.send_video()
    assert frame.video is None
    assert any(event[0] == 'video_error' and '179 degree limit' in event[1]
               for event in events)


@pytest.mark.parametrize('view', ['down', 'fpv'])
def test_video_uses_mount_rotation(backend, monkeypatch, view):
    if sys.platform.startswith('linux') and not os.environ.get('DISPLAY'):
        pytest.skip('VTK offscreen test needs an X display on older VTK builds')
    monkeypatch.setattr(video.time, 'monotonic', lambda: 10.0)
    output = video.VideoOutput()
    try:
        output.open(CONFIG[:-1] + (view,), None, *HOME[:2], 1.0)
        pose = (*HOME, 0.4, -0.3, 1.2)
        mount = (0.2, -0.4, math.pi / 2)
        output.send(pose, 65, (1.25, True, False), mount)
        expected = vtk.vtkCamera()
        tc = TerrainCamera(expected, (0, 0, 0), 100)
        if view == 'down':
            tc.set_down((0, 0, HOME[2]), *pose[3:], mount=mount)
        else:
            tc.set_fpv((0, 0, HOME[2]), pose[5], pose[4], roll=pose[3],
                       mount=mount)
        assert output.tc.cam.GetViewUp() == pytest.approx(expected.GetViewUp())
        assert output.tc.cam.GetDirectionOfProjection() == pytest.approx(
            expected.GetDirectionOfProjection())
    finally:
        output.close()


def test_failed_device_open_can_be_cleaned_up(backend):
    factory, camera, terrain = backend
    factory.side_effect = RuntimeError('not a loopback device')
    output = video.VideoOutput()
    with pytest.raises(RuntimeError, match='not a loopback'):
        output.open(CONFIG, None, *HOME[:2], 1.0)
    output.close()
    camera.close.assert_not_called()
    terrain.shutdown.assert_not_called()


def test_partial_startup_releases_virtual_camera(backend, monkeypatch):
    _, camera, _ = backend
    monkeypatch.setattr(
        video, 'TerrainManager', Mock(side_effect=RuntimeError('terrain failed')))
    output = video.VideoOutput()
    with pytest.raises(RuntimeError, match='terrain failed'):
        output.open(CONFIG, None, *HOME[:2], 1.0)
    output.close()
    camera.close.assert_called_once()
    assert output.window is None


def test_viewer_close_releases_video(map3d_frame, monkeypatch):
    frame = map3d_frame()
    frame.init_scene(*HOME)
    frame.terrain.shutdown = Mock()
    frame.Destroy = Mock()
    output = Mock()
    monkeypatch.setattr(video, 'VideoOutput', Mock(return_value=output))
    frame.set_video(CONFIG)
    frame.on_close(None)
    output.close.assert_called_once()
    frame.timer.Stop.assert_called_once()
    frame.terrain.shutdown.assert_called_once()
    frame.Destroy.assert_called_once()


def test_failed_output_can_be_restarted(map3d_frame, monkeypatch):
    frame = map3d_frame()
    frame.init_scene(*HOME)
    failed, working = Mock(), Mock()
    failed.open.side_effect = RuntimeError('busy')
    monkeypatch.setattr(video, 'VideoOutput',
                        Mock(side_effect=[failed, working]))
    frame.set_video(CONFIG)
    assert frame.video is None
    frame.set_video(CONFIG)
    assert frame.video is working
    working.open.assert_called_once()
    frame.stop_video()


@pytest.mark.parametrize('args', [
    ['video'], ['video', 'oops'], ['video', 'stop', 'extra'],
    ['video', 'start', '/dev/video10', 'extra'], ['view', 'oops'],
])
def test_invalid_commands_report_usage(map3d_module, capsys, args):
    viewer = Mock()
    module = map3d_module(viewer)
    module.cmd_map3d(args)
    assert 'usage: map3d' in capsys.readouterr().out
    viewer.set_video.assert_not_called()
    viewer.set_view.assert_not_called()
