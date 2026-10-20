#!/usr/bin/env python3
"""Turns a run log into the timing sheet you watch while the car runs.

Reads race_result/race_result_<timestamp>.json and draws one screen: a header
band, the run's readouts, the lap-by-lap timing table with a gap bar per lap,
and the track with every collision marked.

    ros2 run race_manager report                 # newest log in ./race_result
    ros2 run race_manager report path/to/log.json
    ros2 run race_manager report --watch         # keep redrawing while the run goes on

The scorer rewrites the log on every lap and every collision, so --watch just
polls its mtime. Without a log path it follows whichever log is newest, which
means the window can stay open across one run and into the next.
"""

import argparse
from dataclasses import dataclass
import glob
import json
import math
import os
import sys
import time

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from race_manager.track import Track

PAGE = '#ffffff'
SURFACE = '#ffffff'
LIFT = '#f1f3f5'
INK = '#171717'
INK2 = '#4f4f4f'
MUTED = '#707070'
RULE = '#dddddd'
OBSTACLE = '#d95926'
WALL = '#3987e5'
RACELINE = '#5548ad'
SANS = ['Source Sans 3', 'DejaVu Sans', 'sans-serif']
MONO = ['JetBrains Mono', 'DejaVu Sans Mono', 'monospace']
# Labels wear the sans, figures the mono; both fall back if Plex is not installed.
# Screen text stays in English: the fallback fonts have no Hangul.

FIGSIZE = (9.0, 5.42)
SCREEN_DPI = 100
PNG_DPI = 200
SHEET_RECT = [0.038, 0.055, 0.465, 0.650]

COLW = 40.0
COLUMNS = (('LAP', 4.0), ('TIME', 13.0), ('GAP', 22.5), ('HITS', 29.0))
MAX_VISIBLE_LAPS = 24


@dataclass(frozen=True)
class SheetLayout:
    laps: list
    columns: int
    range_label: str = ''
    pinned_best: dict = None


def _gap_label(lap, best):
    """What the GAP column says about one lap.

    PART is a lap timed from where the car was put back on track: it covers
    less than a lap, so it is not compared against the best.
    """
    if lap.get('partial'):
        return 'PART'
    if best is None:
        return '—'
    if math.isclose(lap['time'], best, abs_tol=0.0005):
        return 'BEST'
    return f'{lap["time"] - best:+.2f}'


def _sheet_layout(laps, best):
    visible = list(laps[-MAX_VISIBLE_LAPS:])
    clipped = len(visible) < len(laps)
    range_label = ''
    pinned_best = None
    if clipped:
        range_label = f"SHOWING LAPS {visible[0]['lap']}–{visible[-1]['lap']}"
        pinned_best = next(
            (lap for lap in laps[:-MAX_VISIBLE_LAPS]
             if best is not None and math.isclose(lap['time'], best, abs_tol=0.0005)),
            None,
        )
    return SheetLayout(visible, 1 if len(visible) <= 10 else 2,
                       range_label, pinned_best)


def find_newest(log_dir='race_result'):
    found = sorted(glob.glob(os.path.join(log_dir, 'race_*.json')), key=os.path.getmtime)
    return found[-1] if found else None


def newest_log(log_dir='race_result'):
    found = find_newest(log_dir)
    if not found:
        sys.exit(f'no run logs in {os.path.abspath(log_dir)}/ — run the scorer first')
    return found


def _panel(fig, rect, face=SURFACE):
    ax = fig.add_axes(rect)
    ax.set_facecolor(face)
    ax.set_xticks([])
    ax.set_yticks([])
    for side in ax.spines.values():
        side.set_visible(False)
    return ax


def draw(data, track, out_path, fig=None):
    if fig is None:
        fig = plt.figure(figsize=FIGSIZE, dpi=SCREEN_DPI, facecolor=PAGE)
    else:
        # --watch reuses one window; a fresh figure per redraw would pile them up
        fig.clear()
        fig.patch.set_facecolor(PAGE)

    coll = data['collisions']
    laps = data['laps']
    best = data['best_lap']

    _header(fig, data)
    _readouts(fig, data, coll, best)
    _sheet(fig, laps, best)
    _map(fig, data, track)

    fig.savefig(out_path, dpi=PNG_DPI, facecolor=PAGE)
    return fig


def _header(fig, data):
    ax = _panel(fig, [0.0, 0.895, 1.0, 0.105], PAGE)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.text(0.038, 0.54, str(data.get('map') or 'track').upper(), fontsize=18,
            color=INK, family=SANS, fontweight='bold', va='center')
    meta = f"SEED {data['seed']}   ·   {data['started_at'].replace('T', '   ')}"
    ax.text(0.962, 0.54, meta, fontsize=9.5, color=MUTED, family=MONO,
            fontweight='medium', ha='right', va='center')
    ax.plot([0, 1], [0.02, 0.02], color=RULE, lw=1.2, clip_on=False)


def _readouts(fig, data, coll, best):
    ax = _panel(fig, [0.038, 0.735, 0.924, 0.145], PAGE)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.text(0.0, 0.82, 'BEST LAP', fontsize=9, color=MUTED, family=SANS,
            fontweight='semibold', va='center')
    ax.text(0.0, 0.25, f'{best:.2f}' if best else '—', fontsize=28,
            color=INK, family=MONO, fontweight='bold', va='center')
    if best:
        ax.text(0.173, 0.22, 's', fontsize=11, color=MUTED, family=SANS,
                fontweight='semibold', va='center')

    ax.plot([0.235, 0.235], [0.08, 0.92], color=RULE, lw=1.0)
    ax.text(0.278, 0.82, 'LAPS', fontsize=9, color=MUTED, family=SANS,
            fontweight='semibold', va='center')
    ax.text(0.278, 0.25, str(data['laps_completed']), fontsize=22, color=INK,
            family=MONO, fontweight='bold', va='center')

    ax.plot([0.455, 0.455], [0.08, 0.92], color=RULE, lw=1.0)
    ax.text(0.500, 0.82, 'HITS', fontsize=9, color=MUTED, family=SANS,
            fontweight='semibold', va='center')
    ax.text(0.500, 0.25, str(coll['total']), fontsize=22, color=INK,
            family=MONO, fontweight='bold', va='center')
    _summary_kind(ax, 0.665, 'OBS', coll['obstacle'], OBSTACLE, 'o')
    _summary_kind(ax, 0.835, 'WALL', coll['wall'], WALL, 'x')


def _summary_kind(ax, x, label, value, color, marker):
    ax.plot([x], [0.80], marker=marker, ms=7, color=color, mew=2,
            transform=ax.transAxes, clip_on=False)
    ax.text(x + 0.022, 0.82, label, fontsize=8.5, color=MUTED, family=SANS,
            fontweight='semibold', va='center', transform=ax.transAxes)
    ax.text(x, 0.25, str(value), fontsize=19, color=INK2, family=MONO,
            fontweight='bold', va='center', transform=ax.transAxes)


def _sheet(fig, laps, best):
    ax = _panel(fig, SHEET_RECT)
    if not laps:
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        ax.text(0.5, 0.5, 'waiting for first lap', fontsize=10, color=MUTED,
                family=SANS, fontweight='medium', ha='center', va='center')
        return

    layout = _sheet_layout(laps, best)
    rows = len(layout.laps)
    cols = layout.columns
    per_col = math.ceil(rows / cols)
    height_pt = SHEET_RECT[3] * fig.get_figheight() * 72.0
    width_pt = SHEET_RECT[2] * fig.get_figwidth() * 72.0
    top_rows = 3.4 if layout.range_label else 2.6
    by_row = height_pt / (per_col + top_rows) * 0.62
    by_col = width_pt / (COLW * cols * 0.57)
    size = max(8.5, min(11.5, by_row, by_col))

    ax.set_xlim(0, COLW * cols)
    ax.set_ylim(per_col + 0.45, -3.0 if layout.range_label else -2.2)

    if layout.range_label:
        ax.text(0.5, -2.35, layout.range_label, fontsize=7.5, color=MUTED,
                family=SANS, fontweight='semibold', ha='left', va='center')
        if layout.pinned_best:
            pinned = layout.pinned_best
            ax.text(COLW * cols - 1.5, -2.35,
                    f"BEST  LAP {pinned['lap']}  ·  {pinned['time']:.2f} s",
                    fontsize=8.0, color=INK2, family=MONO, fontweight='bold',
                    ha='right', va='center')

    for c in range(cols):
        ox = c * COLW
        for label, cx in COLUMNS:
            ax.text(ox + cx, -1.5, label, fontsize=size * 0.86, color=MUTED,
                    family=SANS, fontweight='semibold', ha='right', va='center')
        ax.plot([ox + 0.5, ox + COLW - 2.0], [-1.0, -1.0], color=RULE, lw=1.0)

        for i, l in enumerate(layout.laps[c * per_col:(c + 1) * per_col]):
            top = best is not None and math.isclose(l['time'], best, abs_tol=0.0005)
            if top:
                ax.add_patch(Rectangle((ox + 0.5, i - 0.42), COLW - 2.5, 0.84,
                                       color=LIFT, zorder=0))
            cells = (
                (COLUMNS[0][1], str(l['lap']), MUTED),
                (COLUMNS[1][1], f"{l['time']:.2f}", MUTED if l.get('partial') else INK),
                (COLUMNS[2][1], _gap_label(l, best), INK2),
            )
            for cx, text, color in cells:
                ax.text(ox + cx, i, text, fontsize=size, color=color,
                        family=MONO, ha='right', va='center', zorder=2,
                        fontweight='bold' if top else 'medium')
            _hit_markers(ax, ox + COLUMNS[3][1], i, l)


def _hit_markers(ax, x, y, lap):
    markers = [('o', OBSTACLE)] * lap['obstacle'] + [('x', WALL)] * lap['wall']
    shown = markers[:4]
    for i, (marker, color) in enumerate(shown):
        ax.plot([x + i * 1.75], [y], marker=marker, ms=4.5, color=color,
                mew=1.35, ls='none', zorder=3)
    if len(markers) > len(shown):
        ax.text(x + len(shown) * 1.75, y, f'+{len(markers) - len(shown)}',
                fontsize=7, color=MUTED, family=MONO, va='center')


def _map(fig, data, track):
    ax = _panel(fig, [0.545, 0.055, 0.417, 0.650])
    left, right = track.boundaries()
    ax.plot(left[:, 0], left[:, 1], color=INK, lw=1.3, zorder=1)
    ax.plot(right[:, 0], right[:, 1], color=INK, lw=1.3, zorder=1)
    ax.plot(track.x, track.y, color=RACELINE, lw=1.6, zorder=2)
    ax.text(0.0, 0.98, 'TRACK', fontsize=9, color=MUTED, family=SANS,
            fontweight='semibold', transform=ax.transAxes, va='top')
    a, b, _ = track.finish_line()
    ax.plot([a[0], b[0]], [a[1], b[1]], color=INK, lw=3.4, zorder=3)
    ax.annotate('FINISH', ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0),
                xytext=(8, 7), textcoords='offset points', fontsize=8,
                color=MUTED, family=SANS, fontweight='semibold',
                ha='left', va='bottom')
    for kind, color, marker in (('obstacle', OBSTACLE, 'o'), ('wall', WALL, 'x')):
        pts = [(e['x'], e['y']) for e in data['collision_events'] if e['kind'] == kind]
        if pts:
            ax.plot([p[0] for p in pts], [p[1] for p in pts], marker=marker, ms=8,
                    mfc=color, mec=SURFACE if marker == 'o' else color,
                    color=color, mew=2.0, ls='none', zorder=5)
    ax.set_aspect('equal', adjustable='datalim')
    ax.margins(0.10)
    _map_legend(ax)


def _map_legend(ax):
    items = ((RACELINE, '-', 'RACELINE'), (OBSTACLE, 'o', 'OBSTACLE'),
             (WALL, 'x', 'WALL'))
    x = 0.34
    for color, marker, label in items:
        if marker == '-':
            ax.plot([x, x + 0.06], [0.035, 0.035], color=color, lw=2,
                    transform=ax.transAxes, clip_on=False)
        else:
            ax.plot([x + 0.025], [0.035], marker=marker, ms=5.5, color=color,
                    mew=1.5, transform=ax.transAxes, clip_on=False)
        ax.text(x + 0.073, 0.035, label, fontsize=7.5, color=MUTED, family=SANS,
                fontweight='semibold', transform=ax.transAxes, va='center')
        x += 0.23


def watch(args, period):
    """Redraw the one window whenever the log changes. Ends when it is closed."""
    if not os.environ.get('DISPLAY') and sys.platform.startswith('linux'):
        sys.exit('--watch needs a display; use plot:=false or run report --no-show')
    # Every plt.pause() calls show(), which would pull the window over whatever is focused
    matplotlib.rcParams['figure.raise_window'] = False
    plt.ion()
    fig = track = None
    shown, stamp = None, None
    print(f'watching every {period:g} s — close the window to stop')

    while True:
        path = args.log or find_newest(args.log_dir)
        if path is not None:
            try:
                mtime = os.path.getmtime(path)
            except OSError:
                mtime = None
            if mtime is not None and (path != shown or mtime != stamp):
                try:
                    with open(path) as f:
                        data = json.load(f)
                except (OSError, ValueError):
                    # The scorer swaps the file atomically; on a bad read just try next tick
                    data = None
                if data is not None:
                    if path != shown:
                        track = Track(args.waypoints or data['waypoints_file'])
                        shown = path
                        print(f'following {path}')
                    fig = draw(data, track, os.path.splitext(path)[0] + '.png', fig)
                    stamp = mtime
                    coll = data['collisions']
                    print(f"{time.strftime('%H:%M:%S')}  "
                          f"lap {data['laps_completed']}  "
                          f"best {data['best_lap'] or 0:.2f} s  "
                          f"obstacle {coll['obstacle']}  wall {coll['wall']}")

        if fig is not None and not plt.fignum_exists(fig.number):
            return
        plt.pause(period)


def main():
    parser = argparse.ArgumentParser(description='Render a race_manager run log')
    parser.add_argument('log', nargs='?', help='path to a run log (default: newest in ./race_result)')
    parser.add_argument('--log-dir', default='race_result',
                        help='directory to search when no log file is specified')
    parser.add_argument('--waypoints', help='override the waypoints file named in the log')
    parser.add_argument('--no-show', action='store_true', help='save the PNG without opening a window')
    parser.add_argument('--watch', nargs='?', type=float, const=1.0, default=None,
                        metavar='SECONDS',
                        help='keep one window open and redraw it whenever the log changes')
    # Launched as a <node> everything from --ros-args on is the launcher's, not ours
    argv = sys.argv[1:]
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    args = parser.parse_args(argv)

    if args.watch is not None:
        if args.no_show:
            sys.exit('--watch and --no-show contradict each other')
        if args.watch <= 0:
            sys.exit('--watch needs a positive interval')
        try:
            watch(args, args.watch)
        except KeyboardInterrupt:
            pass
        return

    path = args.log or newest_log(args.log_dir)
    with open(path) as f:
        data = json.load(f)

    track = Track(args.waypoints or data['waypoints_file'])
    out_path = os.path.splitext(path)[0] + '.png'

    if args.no_show:
        matplotlib.use('Agg')
    draw(data, track, out_path)
    print(f'saved {out_path}')
    if not args.no_show:
        plt.show()


if __name__ == '__main__':
    main()
