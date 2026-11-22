'''Local imagery refinement must not change the downloaded terrain surface.'''

import math
import queue
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

vtk = pytest.importorskip('vtk')
pytest.importorskip('quantized_mesh_tile')

from vtkmodules.util.numpy_support import vtk_to_numpy  # noqa: E402
from MAVProxy.modules.mavproxy_map3d import terrain  # noqa: E402
from MAVProxy.modules.mavproxy_map3d.camera import TerrainCamera  # noqa: E402


def make_tile(lat=-35.36, lon=149.16, width=180.0 / 4096):
    bbox = (lon, lat, lon + width, lat + width)
    verts = np.array([
        (lon, lat, 100), (lon + width, lat, 120),
        (lon, lat + width, 130), (lon + width, lat + width, 150)])
    return terrain.Tile(
        dict(bbox=bbox, verts=verts, idx=np.array(((0, 1, 2), (1, 3, 2)))),
        lat, lon, 1.0, 0.0)


def imagery(max_zoom=19):
    return SimpleNamespace(min_zoom=1, max_zoom=max_zoom)


def area(geo):
    triangles = vtk.vtkTriangleFilter()
    triangles.SetInputData(geo)
    triangles.Update()
    geo = triangles.GetOutput()
    return sum(vtk.vtkTriangle.TriangleArea(
        *(geo.GetPoint(geo.GetCell(i).GetPointId(j)) for j in range(3)))
        for i in range(geo.GetNumberOfCells()))


def patch_image(params):
    mt = SimpleNamespace(area_to_image=lambda lat, lon, width, height, *args, **kwargs:
                         np.zeros((height, width, 3), dtype=np.uint8))
    return terrain.build_texture_image(mt, threading.Lock(), *params)


def test_close_view_gets_submetre_imagery_without_finer_terrain():
    tile = make_tile()
    # 100 m above a corner, deliberately far from the original tile centre.
    changed, requests = tile.texture_requests((0, 0, 200), 800, imagery(), 90)
    assert changed
    assert 1 < len(requests) < 200
    closest, key, params = min(requests, key=lambda req: sum(
        bound ** 2 for bound in req[0].geo.GetBounds()[::2][:2]))
    west, south, east, north, zoom, tex_w = params
    metres = terrain.R * math.cos(math.radians(north)) * math.radians(east - west)
    assert metres / tex_w < 1.0
    assert zoom == 19
    assert closest.geo.GetNumberOfCells() > 0
    # The original full-tile 1024-pixel texture resolves only ~4 m/pixel here.
    whole_width = terrain.R * math.cos(math.radians(north)) * math.radians(
        tile.bbox[2] - tile.bbox[0])
    assert whole_width / 1024 > 3.0
    assert tile.geo.GetNumberOfCells() == 2


@pytest.mark.parametrize('lat,lon', [
    (-35.36, 149.16), (80, 20), (0, 179.9560546875), (0, -180)])
def test_patches_cover_original_geometry_and_have_bounded_textures(lat, lon):
    tile = make_tile(lat, lon)
    original = vtk_to_numpy(tile.geo.GetPoints().GetData()).copy()
    normals = vtk_to_numpy(tile.geo.GetPointData().GetNormals())[0]
    tile.texture_requests((0, 0, 200), 800, imagery(), 90)
    assert sum(area(patch.geo) for patch in tile.patches.values()) == pytest.approx(
        area(tile.geo), rel=1e-7)
    for patch in tile.patches.values():
        key, params = patch.texture_spec((0, 0, 200), 800, imagery(), 90)
        img = patch_image(params)
        assert max(img.shape[:2]) <= terrain.MAX_TEXTURE_SIZE
        patch.apply_texture(img, *params[:4], params[5], key)
        uv = vtk_to_numpy(patch.geo.GetPointData().GetTCoords())
        assert uv.min() >= 0
        assert uv.max() <= 1
        assert np.ptp(uv[:, 0]) > 0.99
        assert np.ptp(uv[:, 1]) > 0.99
        assert vtk_to_numpy(patch.geo.GetPointData().GetNormals()) == pytest.approx(
            np.tile(normals, (patch.geo.GetNumberOfPoints(), 1)))
    assert vtk_to_numpy(tile.geo.GetPoints().GetData()) == pytest.approx(original)
    for fraction in (0.0, 0.25, 0.75):
        east, north = fraction * original[1, 0], fraction * original[2, 1]
        assert tile.height_at(east, north) == pytest.approx(
            100 + east * 20 / original[1, 0] + north * 30 / original[2, 1])


def test_zooming_out_merges_patches_and_preserves_render_settings():
    tile = make_tile()
    tile.texture_requests((0, 0, 200), 800, imagery(), 90)
    tile.apply_render_settings(0.8, False, True)
    for patch in tile.patches.values():
        prop = patch.actor.GetProperty()
        assert not prop.GetLighting()
        assert prop.GetRepresentation() == vtk.VTK_WIREFRAME
    changed, requests = tile.texture_requests((0, 0, 100000), 800, imagery(), 30)
    assert changed
    assert list(tile.patches) == [()]
    assert tile.actor.GetParts().GetNumberOfItems() == 1
    assert tile.root.children is None
    assert len(requests) == 1
    changed, _ = tile.texture_requests((0, 0, 100000), 800, imagery(), 30)
    assert not changed


def test_low_resolution_source_does_not_trigger_unnecessary_subdivision():
    tile = make_tile()
    changed, requests = tile.texture_requests((0, 0, 200), 800, imagery(12), 90)
    assert not changed
    assert len(requests) == 1
    assert requests[0][1][0] == 12


@pytest.mark.parametrize('lat', [85, 90 - 180.0 / 4096, -90])
def test_narrow_polar_patches_still_bound_both_texture_dimensions(lat):
    tile = make_tile(lat, 20)
    _, requests = tile.texture_requests((0, 0, 200), 800, imagery(), 90)
    assert len(requests) < 200
    for _, _, params in requests:
        assert max(patch_image(params).shape[:2]) <= terrain.MAX_TEXTURE_SIZE


def test_requested_native_mosaics_are_bounded(tmp_path):
    tile = make_tile()
    _, requests = tile.texture_requests((700, 700, 200), 800, imagery(), 90)
    mt = terrain.mp_tile.MPTile(cache_path=str(tmp_path), download=False)
    for _, _, params in requests:
        west, south, east, north, zoom, width = params
        height = patch_image(params).shape[0]
        ground_width = terrain.R * math.cos(math.radians(north)) * math.radians(east - west)
        grid = mt.area_tile_grid(north, west, width, height, ground_width, zoom)
        # Choosing zoom from patch texture size, not just proximity, avoids
        # building enormous native mosaics that would then be downsampled.
        assert max(grid[3:5]) <= 2 * terrain.MAX_TEXTURE_SIZE


def test_subdivision_inherits_parent_texture_until_new_imagery_arrives():
    tile = make_tile()
    _, requests = tile.texture_requests((0, 0, 100000), 800, imagery(), 30)
    root, key, params = requests[0]
    img = patch_image(params)
    root.apply_texture(img, *params[:4], params[5], key)
    texture = root.actor.GetTexture()
    tile.texture_requests((0, 0, 200), 800, imagery(), 90)
    for patch in tile.patches.values():
        assert patch.actor.GetTexture() is texture
        uv = vtk_to_numpy(patch.geo.GetPointData().GetTCoords())
        assert uv.min() >= 0 and uv.max() <= 1
    tile.root.invalidate_texture()
    assert root.tex_key is None
    assert all(patch.tex_key is None for patch in tile.patches.values())


def manager(tile, monkeypatch):
    # Keep the real manager's queues and renderer, but never start downloads.
    monkeypatch.setattr(threading.Thread, 'start', lambda self: None)
    mt = imagery()
    mt.download_revision = Mock(return_value=0)
    mt.tiles_pending = Mock(return_value=0)
    ren = vtk.vtkRenderer()
    tm = terrain.TerrainManager(ren, mt, tile.bbox[1], tile.bbox[0])
    key = (12, 7490, 1243)
    tm.tiles[key] = tile
    ren.AddActor(tile.actor)
    monkeypatch.setattr(tm, 'desired_set', lambda *args: {key})
    tc = TerrainCamera(ren.GetActiveCamera(), (0, 0, 100), 100)
    tc.set_down((0, 0, 200), 0, 0, 0)
    tc.cam.SetViewAngle(90)
    return tm, tc, key


def drain_jobs(tm):
    jobs = []
    while not tm.jobs.empty():
        jobs.append(tm.jobs.get_nowait())
    return jobs


def test_async_texture_results_refresh_and_ignore_retired_patches(monkeypatch):
    tile = make_tile()
    tm, tc, _ = manager(tile, monkeypatch)
    tm.update(tc)
    jobs = drain_jobs(tm)
    assert len(jobs) == len(tile.patches)
    # Repeated camera updates do not queue duplicate jobs.
    tm.update(tc)
    assert tm.jobs.empty()
    for kind, jid, payload in jobs:
        params = payload[4]
        tm.results.put((kind, jid, (patch_image(params), payload)))
    assert tm.process(tc)
    assert all(patch.actor.GetTexture() is not None for patch in tile.patches.values())
    assert not tm.inflight
    # A completed download invalidates placeholders and schedules all leaves.
    tm.mt.download_revision.return_value = 1
    tm.process(tc)
    refresh = drain_jobs(tm)
    assert len(refresh) == len(tile.patches)
    old_patch = tile.patches[refresh[0][2][1]]
    tc.set_down((0, 0, 100000), 0, 0, 0)
    tm.update(tc)
    assert old_patch not in tile.patches.values()
    # Then return to the same view: paths match, but these are new patches.
    tc.set_down((0, 0, 200), 0, 0, 0)
    tm.update(tc)
    replacement = tile.patches[old_patch.path]
    assert replacement is not old_patch
    for kind, jid, payload in refresh:
        tm.results.put((kind, jid, (patch_image(payload[4]), payload)))
    tm.process(tc)
    assert replacement.actor.GetTexture() is None
    assert tm.jobs.qsize() > 0


def test_texture_worker_uses_patch_extent(monkeypatch):
    tile = make_tile()
    tm, tc, _ = manager(tile, monkeypatch)
    tm.update(tc)
    kind, jid, payload = tm.jobs.get_nowait()
    params = payload[4]
    img = patch_image(params)

    def build(*args):
        assert args[2:8] == params
        tm.stop = True
        return img
    monkeypatch.setattr(terrain, 'build_texture_image', build)
    tm.jobs = queue.Queue()
    tm.jobs.put((kind, jid, payload))
    tm._worker()
    result_kind, result_jid, (result_img, result_payload) = tm.results.get_nowait()
    assert result_kind == 'texture'
    assert result_jid == jid
    assert result_img is img
    assert result_payload == payload


def test_paging_out_a_tile_removes_all_its_patch_actors(monkeypatch):
    tile = make_tile()
    tm, tc, _ = manager(tile, monkeypatch)
    tm.update(tc)
    assert tile.actor.GetParts().GetNumberOfItems() > 1
    monkeypatch.setattr(tm, 'desired_set', lambda *args: set())
    tm.update(tc)
    assert not tm.tiles
    assert tm.ren.GetViewProps().GetNumberOfItems() == 0
    for kind, jid, payload in drain_jobs(tm):
        tm.results.put((kind, jid, (patch_image(payload[4]), payload)))
    tm.process(tc)
    assert not tm.inflight
    assert tm.ren.GetViewProps().GetNumberOfItems() == 0


def test_worker_errors_are_reported(monkeypatch, capsys):
    tm, tc, _ = manager(make_tile(), monkeypatch)
    tm.results.put(('error', ('D', 12, 1, 1), RuntimeError('tile unavailable')))
    tm.process(tc)
    assert 'tile unavailable' in capsys.readouterr().out


def test_rendered_detail_improves_over_single_full_tile_texture():
    tile = make_tile()
    renderer = vtk.vtkRenderer()
    renderer.SetBackground(1, 0, 1)
    window = vtk.vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.SetMultiSamples(0)
    window.SetSize(256, 256)
    window.AddRenderer(renderer)
    renderer.AddActor(tile.actor)
    camera = renderer.GetActiveCamera()
    tc = TerrainCamera(camera, (0, 0, 100), 100)
    tc.set_down((700, 700, 220), 0, 0, 0)
    camera.SetViewAngle(90)
    tile.apply_render_settings(1.0, False, False)
    tile.property.SetColor(1, 1, 1)
    capture = vtk.vtkWindowToImageFilter()
    capture.SetInput(window)
    capture.SetInputBufferTypeToRGB()
    capture.ReadFrontBufferOff()

    def stripes(params):
        img = patch_image(params)
        west, south, east, north, zoom, width = params
        e, _, _ = terrain.enu(south, west, 0, tile.root.lat0, tile.root.lon0)
        metres = math.radians(east - west) * terrain.R * math.cos(
            math.radians(tile.root.lat0))
        step = metres / width
        # Pixel-area average of a 4-metre-period detail pattern.
        samples = 127.5 + 127.5 * np.sinc(step / 4.0) * np.sin(
            2 * math.pi * (e + (np.arange(width) + 0.5) * step) / 4.0)
        img[:] = np.round(samples).astype(np.uint8)[None, :, None]
        return img

    def render():
        window.Render()
        capture.Modified()
        capture.Update()
        return vtk_to_numpy(capture.GetOutput().GetPointData().GetScalars()).reshape(
            256, 256, 3).copy()

    try:
        params = (*tile.bbox, 19, 1024)
        tile.root.apply_texture(stripes(params), *tile.bbox, 1024, (19, 1024))
        coarse = render()
        _, requests = tile.texture_requests(tc.pos, 256, imagery(), 90)
        for patch, key, params in requests:
            patch.apply_texture(stripes(params), *params[:4], params[5], key)
        fine = render()
        # Also catches missing geometry, incorrectly oriented UVs and seams.
        assert not np.any(np.all(fine == (255, 0, 255), axis=2))
        assert fine[128, :, 0].std() > 40
        assert fine[128, :, 0].std() > 3 * coarse[128, :, 0].std()
    finally:
        window.Finalize()
