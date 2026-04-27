#!/usr/bin/env python3
"""
test_robot.py
=============
Standalone serial test benchmark for TurtleBot3 Conveyor (4SWD swerve drive).
Run this directly on the Raspberry Pi — no ROS required.

Usage:
    python3 test_robot.py

What it does:
    - Connects to OpenCR via /dev/ttyACM0 at 115200 baud
    - Runs each test case for TEST_DURATION seconds
    - Sends commands at SEND_HZ (faster than the 500ms watchdog)
    - Pauses between tests so you can observe
    - Prints a full pass/fail report at the end

How to judge pass/fail:
    Watch the robot during each test. After each motion, the script pauses
    and asks you to press Enter. If the motion looked correct, press Enter.
    If not, type 'f' then Enter to mark it as failed.
"""

import serial
import time
import sys

# ── Configuration ──────────────────────────────────────────────────────────────
PORT         = '/dev/ttyACM0'
BAUD         = 115200
BOOT_WAIT    = 6.0    # seconds to wait for OpenCR boot + homing
TEST_DURATION = 2.5   # seconds each motion runs
PAUSE_BETWEEN = 1.0   # seconds of stop between tests
SEND_HZ      = 10     # command send rate (must be > 2 to beat 500ms watchdog)

# ── Test cases ─────────────────────────────────────────────────────────────────
# Format: (name, x_dot, y_dot, gamma_dot, description)
TEST_CASES = [
    ("FORWARD",        0.15,  0.0,  0.0,  "Robot moves forward (+X)"),
    ("BACKWARD",      -0.15,  0.0,  0.0,  "Robot moves backward (-X)"),
    ("STRAFE LEFT",    0.0,   0.15, 0.0,  "Robot slides left (+Y), no rotation"),
    ("STRAFE RIGHT",   0.0,  -0.15, 0.0,  "Robot slides right (-Y), no rotation"),
    ("PIVOT CCW",      0.0,   0.0,  1.0,  "Robot spins counter-clockwise in place"),
    ("PIVOT CW",       0.0,   0.0, -1.0,  "Robot spins clockwise in place"),
    ("DIAGONAL FWD-L", 0.10,  0.10, 0.0,  "Robot moves diagonally forward-left (45°)"),
    ("DIAGONAL FWD-R", 0.10, -0.10, 0.0,  "Robot moves diagonally forward-right (-45°)"),
    ("FWD + TURN CCW", 0.10,  0.0,  0.5,  "Robot moves forward while curving left"),
    ("FWD + TURN CW",  0.10,  0.0, -0.5,  "Robot moves forward while curving right"),
]

# ── Helpers ────────────────────────────────────────────────────────────────────

def send_cmd(ser: serial.Serial, x: float, y: float, g: float) -> str:
    """Send one velocity command and return the reply line (or empty)."""
    cmd = f"{x:.3f} {y:.3f} {g:.3f}\n"
    ser.write(cmd.encode('ascii'))
    time.sleep(0.01)
    if ser.in_waiting:
        try:
            return ser.readline().decode('ascii', errors='replace').strip()
        except Exception:
            return ""
    return ""

def stop(ser: serial.Serial):
    """Send stop command several times to make sure it's received."""
    for _ in range(5):
        ser.write(b"0.0 0.0 0.0\n")
        time.sleep(0.05)
    # Drain any replies
    time.sleep(0.2)
    ser.reset_input_buffer()

def drain(ser: serial.Serial):
    time.sleep(0.1)
    ser.reset_input_buffer()


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    print("=" * 58)
    print("  TurtleBot3 Conveyor — IK Motion Test Benchmark")
    print("=" * 58)
    print(f"\nConnecting to {PORT} @ {BAUD} baud...")

    try:
        ser = serial.Serial(
            PORT, BAUD,
            timeout=0.5,
            dsrdtr=False,
            rtscts=False,
        )
    except serial.SerialException as e:
        print(f"[ERROR] Cannot open {PORT}: {e}")
        sys.exit(1)

    print(f"Connected. Waiting {BOOT_WAIT:.0f}s for OpenCR boot + homing...")
    print("(Watch the robot steer to X-home position)\n")

    # Drain boot messages
    t0 = time.time()
    while time.time() - t0 < BOOT_WAIT:
        if ser.in_waiting:
            line = ser.readline().decode('ascii', errors='replace').strip()
            if line:
                print(f"  [OpenCR] {line}")
        else:
            time.sleep(0.05)

    print("\nBoot complete. Starting test sequence.\n")
    print("-" * 58)
    print(f"Each test runs for {TEST_DURATION}s. Watch the robot, then confirm.")
    print("-" * 58)

    results = []   # list of (name, passed: bool | None)
    interval = 1.0 / SEND_HZ

    for idx, (name, xd, yd, gd, description) in enumerate(TEST_CASES):
        print(f"\n[{idx+1}/{len(TEST_CASES)}] {name}")
        print(f"  Command : x={xd:+.2f} m/s  y={yd:+.2f} m/s  γ={gd:+.2f} rad/s")
        print(f"  Expected: {description}")
        input("  Press Enter to run this test... ")

        # Run the motion
        print(f"  Running for {TEST_DURATION}s  ▶▶▶", end="", flush=True)
        t_start = time.time()
        ok_count = 0
        err_count = 0

        while time.time() - t_start < TEST_DURATION:
            reply = send_cmd(ser, xd, yd, gd)
            if reply.startswith("OK"):
                ok_count += 1
            elif reply.startswith("ERR"):
                err_count += 1
            time.sleep(interval)

        print(f"  Done. (OK:{ok_count}  ERR:{err_count})")

        # Stop
        stop(ser)
        print(f"  Stopped. Pausing {PAUSE_BETWEEN}s...")
        time.sleep(PAUSE_BETWEEN)

        # Ask user
        ans = input("  Did the robot move correctly? [Enter=YES / f=FAIL / s=SKIP]: ").strip().lower()
        if ans == 'f':
            results.append((name, False))
            print("  ✗ Marked FAIL")
        elif ans == 's':
            results.append((name, None))
            print("  – Skipped")
        else:
            results.append((name, True))
            print("  ✓ Marked PASS")

    # ── Report ─────────────────────────────────────────────────────────────────
    print("\n" + "=" * 58)
    print("  TEST REPORT")
    print("=" * 58)

    passed = sum(1 for _, r in results if r is True)
    failed = sum(1 for _, r in results if r is False)
    skipped = sum(1 for _, r in results if r is None)

    for name, r in results:
        if r is True:
            mark = "✓ PASS"
        elif r is False:
            mark = "✗ FAIL"
        else:
            mark = "– SKIP"
        print(f"  {mark}  {name}")

    print("-" * 58)
    print(f"  Total: {passed} passed, {failed} failed, {skipped} skipped")

    if failed == 0 and skipped == 0:
        print("\n  All tests passed! Ready to proceed to ROS 2 bringup.")
    elif failed > 0:
        print(f"\n  {failed} test(s) failed. Check wheel direction and IK mapping.")

    print("=" * 58)

    # Final stop and close
    stop(ser)
    ser.close()


if __name__ == "__main__":
    main()
