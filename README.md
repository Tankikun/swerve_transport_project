# Swerve Transport Project

Single team repository for high-level ROS 2 control and low-level OpenCR firmware.

## Repository Layout

- `ros2_ws/src/swerve_formation/` — high-level Python ROS 2 package
- `opencr_firmware/swerve_kinematics/` — low-level OpenCR C++ firmware

## Branching Rules

- `main` = only tested code (real robots / validated simulation)
- `feature/laplacian-python` = Python formation and transport logic
- `feature/opencr-swerve` = OpenCR motor + kinematics firmware

## Daily Workflow (Python side)

1. Sync with main
   - `git checkout main`
   - `git pull origin main`
2. Switch to feature branch
   - `git checkout feature/laplacian-python`
3. Implement + test
4. Commit small working chunks
   - `git status`
   - `git add ros2_ws/src/swerve_formation/...`
   - `git commit -m "<descriptive message>"`
5. Push
   - `git push origin feature/laplacian-python`

## Build (inside `ros2_ws`)

```bash
colcon build
source install/setup.bash
```

## Pull Request Policy

- Open PR to merge feature branch into `main`
- Require at least one teammate review
- Merge only after simulation or hardware verification
