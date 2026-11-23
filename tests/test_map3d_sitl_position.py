from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from MAVProxy.modules import mavproxy_map3d
from pymavlink import mavutil


def packet(kind, **fields):
    return SimpleNamespace(
        get_type=lambda: kind, get_srcSystem=lambda: 1,
        get_srcComponent=lambda: 1, **fields)


@pytest.fixture
def module():
    module = mavproxy_map3d.Map3DModule.__new__(mavproxy_map3d.Map3DModule)
    module.map3d_settings = mavproxy_map3d.make_settings()
    module.map = Mock()
    module.map.is_alive.return_value = True
    module.mpstate = SimpleNamespace(
        settings=SimpleNamespace(target_system=1, target_component=1),
        master=lambda: None)
    module.send_vehicle_position = Mock()
    module.last_global_position = 0
    module.last_sim_state = 0
    module.last_sim_request = 0
    module.last_sim_warning = 0
    return module


def test_sitl_source_uses_truth_not_estimate(module, monkeypatch):
    monkeypatch.setattr(mavproxy_map3d.time, 'monotonic', lambda: 10.0)
    module.map3d_settings.set('positionsource', 'sitl')
    estimate = packet('GLOBAL_POSITION_INT', lat=601000000, lon=240000000,
                      alt=100000, relative_alt=99000)
    truth = packet('SIM_STATE', lat_int=601695006, lon_int=248304476,
                   alt=123.5, roll=0.1, pitch=-0.2, yaw=1.5)

    module.mavlink_packet(estimate)
    module.send_vehicle_position.assert_not_called()
    module.mavlink_packet(truth)
    module.send_vehicle_position.assert_called_once_with(
        pytest.approx(60.1695006), pytest.approx(24.8304476), 123.5,
        attitude=(0.1, -0.2, 1.5), force=False)
    assert module.last_sim_state == 10.0


def test_estimate_remains_default_and_ignores_truth(module):
    truth = packet('SIM_STATE', lat_int=601695006, lon_int=248304476,
                   alt=123.5, roll=0.1, pitch=-0.2, yaw=1.5)
    estimate = packet('GLOBAL_POSITION_INT', lat=601695006, lon=248304476,
                      alt=123500, relative_alt=122500)
    module.mavlink_packet(truth)
    module.send_vehicle_position.assert_not_called()
    module.mavlink_packet(estimate)
    module.send_vehicle_position.assert_called_once_with(
        pytest.approx(60.1695006), pytest.approx(24.8304476), 123.5, 1.0,
        force=False)


@pytest.mark.parametrize('fields', [
    dict(lat_int=0, lon_int=0, alt=123.5),
    dict(lat_int=910000000, lon_int=100, alt=123.5),
    dict(lat_int=601695006, lon_int=248304476, alt=float('nan')),
])
def test_invalid_truth_does_not_replace_position(module, fields):
    module.map3d_settings.set('positionsource', 'sitl')
    module.mavlink_packet(packet('SIM_STATE', roll=0, pitch=0, yaw=0, **fields))
    module.send_vehicle_position.assert_not_called()
    assert module.last_sim_state == 0


def test_truth_from_another_vehicle_is_ignored(module):
    module.map3d_settings.set('positionsource', 'sitl')
    truth = packet('SIM_STATE', lat_int=601695006, lon_int=248304476,
                   alt=123.5, roll=0, pitch=0, yaw=0)
    truth.get_srcSystem = lambda: 2
    module.mavlink_packet(truth)
    module.send_vehicle_position.assert_not_called()


def test_request_truth_stream_and_retry_if_stale(module, monkeypatch, capsys):
    now = [100.0]
    monkeypatch.setattr(mavproxy_map3d.time, 'monotonic', lambda: now[0])
    master = SimpleNamespace(mav=SimpleNamespace(command_long_send=Mock()))
    module.mpstate.master = lambda: master
    module.request_sim_state()
    master.mav.command_long_send.assert_called_once_with(
        1, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_SIM_STATE, 100000, 0, 0, 0, 0, 0)
    assert 'waiting for SITL SIM_STATE' in capsys.readouterr().out
    now[0] += 1
    module.request_sim_state()
    assert master.mav.command_long_send.call_count == 1
    module.last_sim_state = now[0]
    now[0] += 4
    module.request_sim_state()
    assert master.mav.command_long_send.call_count == 1
    now[0] += 1
    module.request_sim_state()
    assert master.mav.command_long_send.call_count == 2


def test_cached_estimate_is_not_used_for_sitl(module):
    master = SimpleNamespace(messages={
        'GLOBAL_POSITION_INT': packet('GLOBAL_POSITION_INT', lat=1, lon=2,
                                      alt=1000, relative_alt=0)})
    module.mpstate.master = lambda: master
    module.icon_type = None
    module.map3d_settings.set('positionsource', 'sitl')
    module.send_cached_state()
    module.send_vehicle_position.assert_not_called()


def test_switching_sources_requests_fresh_truth_and_restores_estimate(module):
    module.send_cached_state = Mock()
    module.send_mission = Mock()
    module.send_render_settings = Mock()
    module.last_sim_state = 20
    module.last_sim_request = 19
    module.last_sim_warning = 18
    module.last_vehicle_send = 17

    module.cmd_map3d(['set', 'positionsource', 'sitl'])
    assert module.map3d_settings.positionsource == 'sitl'
    assert module.last_sim_state == 0
    assert module.last_sim_request == 0
    assert module.last_sim_warning == 0
    assert module.last_vehicle_send == 0
    module.send_cached_state.assert_not_called()

    module.cmd_map3d(['set', 'positionsource', 'estimate'])
    assert module.map3d_settings.positionsource == 'estimate'
    module.send_cached_state.assert_called_once_with()


def test_sitl_source_ignores_estimated_fallbacks(module):
    module.map3d_settings.set('positionsource', 'sitl')
    module.mavlink_packet(packet('POS', Lat=1, Lng=2, Alt=3))
    module.mavlink_packet(packet('GPS_RAW_INT', fix_type=3, lat=1, lon=2,
                                 alt=3, cog=0))
    module.send_vehicle_position.assert_not_called()


def test_sitl_pose_reaches_viewer_while_estimate_is_frozen(module, monkeypatch):
    now = [100.0]
    monkeypatch.setattr(mavproxy_map3d.time, 'time', lambda: now[0])
    monkeypatch.setattr(mavproxy_map3d.time, 'monotonic', lambda: now[0])
    module.map3d_settings.set('positionsource', 'sitl')
    module.map.origin_set.return_value = True
    module.last_vehicle_send = 0
    module.send_vehicle_position = (
        mavproxy_map3d.Map3DModule.send_vehicle_position.__get__(module))
    estimate = packet('GLOBAL_POSITION_INT', lat=601000000, lon=240000000,
                      alt=100000, relative_alt=99000)
    for lat, lon, alt in ((601695006, 248304476, 100.0),
                          (601696006, 248305476, 102.0)):
        module.mavlink_packet(estimate)
        module.mavlink_packet(packet('SIM_STATE', lat_int=lat, lon_int=lon,
                                     alt=alt, roll=0.1, pitch=0.2, yaw=0.3))
        now[0] += 1

    calls = module.map.set_vehicle.call_args_list
    assert len(calls) == 2
    assert calls[0].args == pytest.approx((60.1695006, 24.8304476, 100.0,
                                          0.1, 0.2, 0.3))
    assert calls[1].args == pytest.approx((60.1696006, 24.8305476, 102.0,
                                          0.1, 0.2, 0.3))
