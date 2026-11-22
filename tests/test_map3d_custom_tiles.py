import pytest

from MAVProxy.modules.mavproxy_map3d.map3d import (
    CUSTOM_TILE_URL_ENV, resolve_tile_service)
from MAVProxy.modules.mavproxy_map3d import make_settings


def test_builtin_tile_service_is_unchanged():
    assert resolve_tile_service('MicrosoftSat', {}) == ('MicrosoftSat', None)


def test_custom_tile_service_uses_environment_template():
    url = 'https://tiles.example/${ZOOM}/${X}/${Y}.png?key=secret'
    service, resolved_url = resolve_tile_service(
        'Custom', {CUSTOM_TILE_URL_ENV: url})

    assert service.startswith('Map3DCustom-')
    assert resolved_url == url
    assert 'secret' not in service


def test_custom_tile_service_cache_namespace_depends_on_url():
    first, _ = resolve_tile_service(
        'Custom', {CUSTOM_TILE_URL_ENV: 'https://one/${Z}/${X}/${Y}.png'})
    second, _ = resolve_tile_service(
        'Custom', {CUSTOM_TILE_URL_ENV: 'https://two/${Z}/${X}/${Y}.png'})

    assert first != second


def test_custom_tile_service_requires_environment_variable():
    with pytest.raises(ValueError, match=CUSTOM_TILE_URL_ENV):
        resolve_tile_service('Custom', {})


def test_custom_tile_service_rejects_bad_template():
    with pytest.raises(ValueError, match='not a valid tile URL template'):
        resolve_tile_service(
            'Custom', {CUSTOM_TILE_URL_ENV: 'https://tiles/${UNKNOWN}'})


def test_high_resolution_settings_preserve_existing_defaults():
    settings = make_settings()

    assert settings.terrainzoom == 12
    assert settings.imagerymaxzoom == 19
