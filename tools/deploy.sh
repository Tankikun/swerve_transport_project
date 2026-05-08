#!/bin/bash
# tools/deploy.sh
# ---------------
# Sync this branch's code to both Pis and rebuild the workspace there.
# Run on the LAPTOP from the repo root after `git pull` on
# feature/depth-obstacle-avoid.
#
# Usage:
#   bash tools/deploy.sh                                 # defaults below
#   PI1=pi1@192.168.1.101 PI2=pi2@192.168.1.102 bash tools/deploy.sh
#   bash tools/deploy.sh --pi1-only                      # skip pi2
#   bash tools/deploy.sh --pi2-only                      # skip pi1
#
# Idempotent. Re-running just rsyncs the diff and re-runs colcon.
#
# Why a script: the demo touches three packages (swerve_formation,
# swerve_bringup, opencr_firmware) and you want both robots running the
# same code. One human-readable command beats six rsync/ssh lines.

set -euo pipefail

PI1="${PI1:-pi1@192.168.1.101}"
PI2="${PI2:-pi2@192.168.1.102}"

# Branch we expect to be on.
EXPECTED_BRANCH="feature/depth-obstacle-avoid"

# Packages to build remotely (skip the rest of the workspace).
PACKAGES="swerve_formation swerve_bringup"

skip_pi1=false
skip_pi2=false
for arg in "$@"; do
    case "$arg" in
        --pi1-only) skip_pi2=true ;;
        --pi2-only) skip_pi1=true ;;
        *) echo "unknown flag: $arg"; exit 2 ;;
    esac
done

# ── sanity ────────────────────────────────────────────────────────────────

current_branch=$(git rev-parse --abbrev-ref HEAD)
if [[ "$current_branch" != "$EXPECTED_BRANCH" ]]; then
    echo "WARNING: laptop is on branch '$current_branch', expected '$EXPECTED_BRANCH'."
    echo "         Continue anyway? (y/N) "
    read -r ans
    [[ "$ans" == "y" || "$ans" == "Y" ]] || exit 1
fi

if ! git diff --quiet HEAD; then
    echo "WARNING: working tree has uncommitted changes; they WILL be deployed."
    git status --short
    echo "Continue? (y/N) "
    read -r ans
    [[ "$ans" == "y" || "$ans" == "Y" ]] || exit 1
fi

# ── deploy to one Pi ──────────────────────────────────────────────────────

deploy_one() {
    local target="$1"
    echo
    echo "=========================================================="
    echo "Deploying to ${target}"
    echo "=========================================================="

    echo "→ rsync ros2_ws/src/ ..."
    rsync -av --delete \
        --exclude='__pycache__' \
        --exclude='install' \
        --exclude='build' \
        --exclude='log' \
        ros2_ws/src/swerve_formation \
        ros2_ws/src/swerve_bringup \
        "${target}:~/ros2_ws/src/"

    echo "→ rsync tools/ to ~/swerve_demo_tools/ (so demo_aliases.sh is at a stable path) ..."
    rsync -av tools/ "${target}:~/swerve_demo_tools/"

    echo "→ colcon build (symlink-install, packages: ${PACKAGES}) ..."
    ssh "$target" bash -lc "'
        set -e
        cd ~/ros2_ws
        source /opt/ros/humble/setup.bash
        colcon build --symlink-install --packages-select ${PACKAGES}
        echo \"  ament index: \$(ros2 pkg prefix swerve_formation)\"
        # Confirm the demo entry points are registered.
        if ros2 pkg executables swerve_formation | grep -q obstacle_avoidance_node; then
            echo \"  obstacle_avoidance_node: registered\"
        else
            echo \"  WARNING: obstacle_avoidance_node not registered — colcon build may have skipped setup.py\"
        fi
    '"
}

if ! $skip_pi1; then deploy_one "$PI1"; fi
if ! $skip_pi2; then deploy_one "$PI2"; fi

echo
echo "=========================================================="
echo "Deployment complete."
echo "=========================================================="
echo
echo "On each Pi, the demo aliases are now at: ~/swerve_demo_tools/demo_aliases.sh"
echo
echo "Per-terminal:"
echo "  pi1:    source ~/swerve_demo_tools/demo_aliases.sh && demo_follower"
echo "  pi2:    source ~/swerve_demo_tools/demo_aliases.sh && demo_leader"
echo "  laptop: source tools/demo_aliases.sh && demo_preflight && demo_run"
