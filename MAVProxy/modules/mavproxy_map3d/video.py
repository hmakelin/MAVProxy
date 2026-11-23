'''Terrain-only, vehicle-mounted render output for Linux v4l2loopback.'''

import time

import numpy as np
import vtk
from vtk.util.numpy_support import vtk_to_numpy

from MAVProxy.modules.mavproxy_map3d.camera import TerrainCamera
from MAVProxy.modules.mavproxy_map3d.terrain import (
    TerrainManager, enu)


class VideoOutput:
    def __init__(self):
        self.camera = None
        self.window = None
        self.terrain = None
        self.next_frame = 0.0
        self.render_settings = None

    def open(self, config, mt, lat0, lon0, zexag):
        try:
            import pyvirtualcam
        except ImportError as ex:
            raise RuntimeError(
                "install video support with pip install 'MAVProxy[map3d-video]'"
            ) from ex
        device, self.width, self.height, self.fps, self.view = config
        self.camera = pyvirtualcam.Camera(
            width=self.width, height=self.height, fps=self.fps,
            device=device, fmt=pyvirtualcam.PixelFormat.RGB,
            backend='v4l2loopback')
        self.zexag = zexag
        self.window = vtk.vtkRenderWindow()
        self.window.SetOffScreenRendering(1)
        self.window.SetSize(self.width, self.height)
        self.window.SetMultiSamples(0)
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.45, 0.62, 0.85)
        self.window.AddRenderer(self.renderer)
        self.tc = TerrainCamera(self.renderer.GetActiveCamera(), (0, 0, 0), 100)
        # Independent paging keeps map navigation from changing video coverage.
        self.terrain = TerrainManager(
            self.renderer, mt, lat0, lon0, zexag=zexag,
            screen_h=self.height)
        self.capture = vtk.vtkWindowToImageFilter()
        self.capture.SetInput(self.window)
        self.capture.SetInputBufferTypeToRGB()
        self.capture.ReadFrontBufferOff()
        self.capture.ShouldRerenderOff()

    def send(self, pose, fov, render_settings, mount=(0.0, 0.0, 0.0)):
        now = time.monotonic()
        if now < self.next_frame:
            return
        if self.next_frame == 0.0:
            self.next_frame = now
        lat, lon, amsl, roll, pitch, yaw = pose
        position = enu(lat, lon, amsl * self.zexag,
                       self.terrain.lat0, self.terrain.lon0)
        if self.view == 'down':
            self.tc.set_down(position, roll, pitch, yaw, mount=mount)
        else:
            self.tc.set_fpv(position, yaw, pitch, roll=roll, mount=mount)
        self.tc.cam.SetViewAngle(fov)
        if render_settings != self.render_settings:
            self.terrain.set_render_settings(*render_settings)
            self.render_settings = render_settings
        self.terrain.update(self.tc)
        self.terrain.process(self.tc, budget=0.008)
        self.window.Render()
        # VTK caches filter output and stores rows bottom-up.
        self.capture.Modified()
        self.capture.Update()
        image = vtk_to_numpy(self.capture.GetOutput().GetPointData().GetScalars())
        frame = np.ascontiguousarray(
            image.reshape(self.height, self.width, 3)[::-1])
        self.camera.send(frame)
        self.next_frame = max(self.next_frame + 1.0 / self.fps, now)

    def close(self):
        if self.terrain is not None:
            self.terrain.shutdown()
            self.terrain = None
        try:
            if self.camera is not None:
                self.camera.close()
        finally:
            self.camera = None
            if self.window is not None:
                self.window.Finalize()
                self.window = None
