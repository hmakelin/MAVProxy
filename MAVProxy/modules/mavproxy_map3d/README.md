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
