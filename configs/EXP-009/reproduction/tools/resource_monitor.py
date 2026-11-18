#!/usr/bin/env python3
"""Read-only, one-second resource samples; no ROS publishers or system changes."""
import argparse
import json
import signal
import time
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument('output', type=Path)
p.add_argument('--once', action='store_true')
a = p.parse_args()
running = True

def stop(*_):
    global running
    running = False

def read(path):
    try:
        return Path(path).read_text().strip()
    except OSError:
        return None

signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
with a.output.open('a') as f:
    while running:
        sample = {'wall': time.time(), 'monotonic': time.monotonic(),
                  'loadavg': read('/proc/loadavg'),
                  'cpu_stat': read('/proc/stat').splitlines()[0],
                  'cgroup_cpu_stat': read('/sys/fs/cgroup/cpu.stat'),
                  'cpu_pressure': read('/proc/pressure/cpu'),
                  'thermal': {}, 'frequency_khz': {}}
        for zone in Path('/sys/class/thermal').glob('thermal_zone*'):
            sample['thermal'][zone.name] = {'type': read(zone/'type'), 'millidegrees': read(zone/'temp')}
        for cpu in Path('/sys/devices/system/cpu').glob('cpu[0-9]*'):
            value = read(cpu/'cpufreq/scaling_cur_freq')
            if value is not None:
                sample['frequency_khz'][cpu.name] = value
        f.write(json.dumps(sample) + '\n')
        f.flush()
        if a.once:
            break
        time.sleep(1)
