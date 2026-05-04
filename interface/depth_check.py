"""
depth_check.py — Live center-pixel depth readout for tape-measure verification.

Point the camera at a flat wall (perpendicular!), hold still, measure
distance with a tape, and compare to what the OAK-D reports.

Usage:
    /usr/bin/python3 depth_check.py             # runs forever, Ctrl-C to stop
    /usr/bin/python3 depth_check.py --seconds 8 # auto-stop after N seconds

Healthy depth accuracy on OAK-D Lite:
    - At 1 m  : ±1-2 cm
    - At 2 m  : ±3-5 cm
    - At 3 m  : ±5-10 cm
    - Past 4 m: ±10-20 cm (gets unreliable)

If your tape says 1.00 m and the camera says 1.10 m at 1 m → 10% scale error → recalibrate.
If both agree within ±2 cm → calibration is fine; mapping issues are drift.
"""

import argparse
import time

import depthai as dai
import numpy as np


def build_pipeline():
    p = dai.Pipeline()

    left = p.create(dai.node.MonoCamera)
    left.setBoardSocket(dai.CameraBoardSocket.LEFT)
    left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

    right = p.create(dai.node.MonoCamera)
    right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

    stereo = p.create(dai.node.StereoDepth)
    # Preset name differs across DepthAI versions:
    #   2.x : HIGH_ACCURACY available
    #   3.x : renamed to DEFAULT / FAST_ACCURACY
    PresetMode = dai.node.StereoDepth.PresetMode
    preset = (getattr(PresetMode, 'HIGH_ACCURACY', None)
              or getattr(PresetMode, 'DEFAULT', None)
              or getattr(PresetMode, 'FAST_ACCURACY', None))
    if preset is not None:
        stereo.setDefaultProfilePreset(preset)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(True)
    left.out.link(stereo.left)
    right.out.link(stereo.right)

    xout = p.create(dai.node.XLinkOut)
    xout.setStreamName('depth')
    stereo.depth.link(xout.input)
    return p


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seconds', type=float, default=None,
                    help='Auto-stop after N seconds (default: run forever)')
    ap.add_argument('--patch', type=int, default=5,
                    help='Half-window size for the center patch (default 5 = 11x11 px)')
    args = ap.parse_args()

    pipeline = build_pipeline()
    print('OAK-D depth running. Point at a flat wall, hold still.')
    print('Sampling the median of an 11x11 patch at the depth-frame center.')
    print()

    end = time.time() + args.seconds if args.seconds else None

    with dai.Device(pipeline) as device:
        q = device.getOutputQueue('depth', 4, False)
        last_print = 0.0
        while True:
            if end and time.time() >= end:
                print()
                break

            msg = q.tryGet()
            if msg is None:
                time.sleep(0.05)
                continue

            depth = msg.getFrame()                  # uint16 mm
            h, w = depth.shape
            cy, cx = h // 2, w // 2
            patch = depth[cy - args.patch: cy + args.patch + 1,
                          cx - args.patch: cx + args.patch + 1]
            valid = patch[patch > 0]
            if valid.size == 0:
                continue

            mm     = float(np.median(valid))
            mn     = float(valid.min())
            mx     = float(valid.max())
            spread = mx - mn

            now = time.time()
            if now - last_print >= 0.1:
                print(f'  center: {mm/1000.0:6.3f} m  '
                      f'({mm:5.0f} mm)   '
                      f'spread within patch: {spread:4.0f} mm   '
                      f'valid: {valid.size}/{patch.size} px',
                      end='\r')
                last_print = now


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print()
