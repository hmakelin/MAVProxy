# 3D map cameras and virtual video

Load with `module load map3d`. The **FPV View** and **Down View** buttons select
vehicle-mounted cameras. Both follow full MAVLink roll, pitch and yaw, including
inverted flight; FPV is no longer horizon-stabilized. Down View looks along body
down, with body forward at the top of the image. This is a fixed mount, not a
stabilized gimbal.

The same views are available from the console:

```
map3d view down
map3d view fpv
map3d view map
```

Returning to the map restores its previous camera and field of view.
`map3d set fpvfov 90` sets the vertical field of view of both mounted views
and video output.

## SITL ground-truth position

Map3d normally follows the autopilot's estimated position and attitude.
For ArduPilot SITL, use the simulator's actual position, altitude and attitude
for both the map and video camera instead:

```
map3d set positionsource sitl
```

Map3d requests `SIM_STATE` at 10 Hz. MAVLink 2 is required for its precise
latitude and longitude fields. If the simulator state stops arriving, Map3d
warns and retries the request; it does not silently switch to the estimated
position. The last rendered pose remains visible until fresh truth arrives.
This only changes the rendering source: it does not send ground truth to the
autopilot, change the EKF, or replace its navigation telemetry. Switch back
with `map3d set positionsource estimate`. The default is `estimate`, including
for real vehicles and log review.

## Custom imagery tiles

Map3d can use a custom XYZ/Web Mercator imagery source. Set the tile URL in the
environment before starting MAVProxy. The URL uses the same `string.Template`
variables as the built-in tile services; `${ZOOM}`, `${X}`, and `${Y}` are the
usual XYZ variables.

```
export MAVPROXY_MAP3D_TILE_URL='https://tiles.example.com/${ZOOM}/${X}/${Y}.png?key=...'
```

Select the custom service and restart the map so the imagery source is applied:

```
map3d stop
map3d set service Custom
map3d set terrainzoom 12
map3d set imagerymaxzoom 22
map3d start
```

The URL is read only when `Custom` is selected. Each distinct URL uses a
separate hashed cache namespace, and the URL itself is not included in the
cache path. The endpoint must return image tiles; authorization can be included
in the URL query string. `imagerymaxzoom` is the highest zoom map3d may request
from the imagery source, not a forced zoom level. The source must actually
provide useful imagery at that zoom.

## Imagery detail

Terrain meshes and imagery resolution are independent. Map3d automatically
clips existing terrain into smaller textured patches near the camera, then
merges the patches when zooming out. Each texture is at most 1024 pixels in
either dimension. All patches cover their complete piece of terrain, including
in FPV and Down View, without stretching a small image across a large mesh.
This also applies to virtual video output.

No new terrain server or higher-resolution terrain downloads are needed.
`terrainzoom` controls the downloaded mesh tiles; keep it at the default 12
for the ArduPilot terrain server, which advertises a maximum zoom of 12.
Raising it to 14 or 16 is not a way to increase imagery detail and can request
missing terrain tiles. `imagerymaxzoom` and `terrainzoom` take effect when
the map is started.

Detail remains limited by the imagery source and camera distance. Closer views
can use more textures and imagery downloads; loading is asynchronous, so
existing textures remain visible while finer imagery arrives.

## Linux v4l2loopback output

Install the optional Python dependency into the Python environment running
MAVProxy:

```
python3 -m pip install 'MAVProxy[map3d-video]'
```

Install your distribution's v4l2loopback kernel module and utilities (on
Debian/Ubuntu, `v4l2loopback-dkms` and `v4l-utils`). Create an unused loopback
device before starting the producer:

```
sudo modprobe v4l2loopback video_nr=10 card_label="MAVProxy map3d" exclusive_caps=1
```

The user running MAVProxy needs write access to `/dev/video10` (typically through
the `video` group). If v4l2loopback is already loaded, use its existing device or
configure another using your distribution's tools; do not unload a module used
by other applications.

In the MAVProxy console:

```
map3d set videowidth 640
map3d set videoheight 480
map3d set videofps 20
map3d set videoview down
map3d video start /dev/video10
```

The device defaults to `/dev/video10` if omitted. Start your application's V4L2
reader after MAVProxy reports that video started, especially with
`exclusive_caps=1`. For example:

```
ffplay -f v4l2 -video_size 640x480 -i /dev/video10
```

Output is a separate, fixed-resolution, terrain-only render: no vehicle,
trail, mission, fence, labels, KML, status text, or GUI controls. Navigating or
resizing the interactive map does not move or resize the video camera.
`videoview` selects `down` (default) or `fpv` independently of the map window.
Both video views follow full vehicle attitude. Use `zexag=1` for realistic
geometry.

Video settings are applied when `map3d video start` is issued; issue it again
after changing them. `fpvfov` and terrain brightness/shading/wireframe changes
apply live. Frames begin once the viewer has a vehicle position; terrain and
imagery load asynchronously and may initially be incomplete. The frame rate is
a target, limited by terrain loading and rendering speed; telemetry is not
interpolated. The viewer still needs its normal graphical display/OpenGL
environment; this is not a headless MAVProxy mode.

Stop with `map3d video stop`. Stopping/closing the map also releases the virtual
camera. Device/dependency failures are reported in the MAVProxy console and
stop video without closing the map; correct the problem and start video again.
