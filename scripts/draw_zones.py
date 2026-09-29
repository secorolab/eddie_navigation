#!/usr/bin/env python3
"""Draw a map's doors by hand and save them as its zone file.

Usage: draw_zones.py maps/secoro_slam.yaml maps/secoro_slam_zones.yaml
Left-click both jambs of a door (inner edges), one door after the other. Right-click or
Backspace removes the last door. Press Enter or close the window to save; each door is squared
to the map's wall directions.
"""

import math
import os
import sys

import matplotlib
import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

matplotlib.use('QtAgg')
import matplotlib.pyplot as plt  # noqa: E402

# eddie_navigation/msg/MotionConstraints, by name; as scripts/zones_from_fpm.py
DOOR_CONSTRAINTS = {'heading_mode': 'along', 'max_speed': 0.1, 'align_tolerance_xy': 0.01,
                    'align_tolerance_yaw': 0.02, 'stop_time': 1.0}


def wall_angle(img, occupied_thresh):
    """The map's dominant wall direction modulo 90 deg [rad], from its gradient orientations."""
    occ = ((255.0 - img) / 255.0 > occupied_thresh).astype(float)
    gx, gy = ndimage.sobel(occ, axis=1), -ndimage.sobel(occ, axis=0)
    mag = np.hypot(gx, gy)
    # 4x the angle folds the four wall directions onto one
    return math.atan2((mag * np.sin(4 * np.arctan2(gy, gx))).sum(),
                      (mag * np.cos(4 * np.arctan2(gy, gx))).sum()) / 4


def door(name, a, b, walls):
    """A zone from its two jambs, its normal squared to the nearest wall direction."""
    along = b - a
    width = float(np.linalg.norm(along))
    heading = math.atan2(-along[0], along[1])
    heading = walls + round((heading - walls) / (math.pi / 2)) * math.pi / 2
    # either sign: ZoneOnPath takes the direction of travel from the path
    return {'name': name, 'type': 'door',
            'centre': [round(float(v), 3) for v in (a + b) / 2],
            'normal': [round(math.cos(heading), 4), round(math.sin(heading), 4)],
            'width': round(width, 3),
            'constraints': dict(DOOR_CONSTRAINTS)}


def main():
    map_yaml, out = sys.argv[1:3]
    with open(map_yaml) as f:
        info = yaml.safe_load(f)
    img = np.array(Image.open(os.path.join(os.path.dirname(map_yaml), info['image'])))
    res = info['resolution']
    ox, oy = info['origin'][:2]

    fig, ax = plt.subplots(figsize=(10, 12))
    ax.imshow(img, cmap='gray', origin='upper',
              extent=[ox, ox + img.shape[1] * res, oy, oy + img.shape[0] * res])
    ax.set_aspect('equal')
    title = 'click both jambs of each door; right-click = undo; Enter = save'
    ax.set_title(title)
    doors, artists, jamb = [], [], []

    def redraw_title():
        ax.set_title(f'{len(doors)} doors | {title}')
        fig.canvas.draw_idle()

    def on_click(event):
        if event.inaxes != ax or fig.canvas.toolbar.mode:
            return
        if event.button == 3:
            undo()
            return
        jamb.append((event.xdata, event.ydata))
        artists.append(ax.plot(event.xdata, event.ydata, 'o', color='tab:orange')[0])
        if len(jamb) == 2:
            a, b = np.array(jamb)
            doors.append((a, b))
            artists.append(ax.plot(*np.array([a, b]).T, '-', color='tab:blue', lw=4)[0])
            artists.append(ax.annotate(f'door-{len(doors)}', (a + b) / 2, color='tab:blue'))
            jamb.clear()
        redraw_title()

    def undo():
        if jamb:
            jamb.clear()
            artists.pop().remove()
        elif doors:
            doors.pop()
            for _ in range(4):
                artists.pop().remove()
        redraw_title()

    def on_key(event):
        if event.key == 'backspace':
            undo()
        elif event.key == 'enter':
            plt.close(fig)

    fig.canvas.mpl_connect('button_press_event', on_click)
    fig.canvas.mpl_connect('key_press_event', on_key)
    plt.show()

    walls = wall_angle(img, info['occupied_thresh'])
    zones = [door(f'door-{i + 1}', a, b, walls) for i, (a, b) in enumerate(doors)]
    if not zones:
        print('no doors drawn; nothing saved')
        return
    with open(out, 'w') as f:
        f.write(f'# drawn by hand on {os.path.basename(map_yaml)} (scripts/draw_zones.py)\n')
        yaml.safe_dump({'zones': zones}, f, default_flow_style=None, sort_keys=False)
    for z in zones:
        print(f"{z['name']}: centre {z['centre']}, width {z['width']} m, "
              f"heading {math.degrees(math.atan2(z['normal'][1], z['normal'][0])):.0f} deg")
    print(f'{len(zones)} zones -> {out}')


if __name__ == '__main__':
    main()
