# Development environment

This repository is a ROS 2 Humble colcon workspace. The recommended checks are
the same locally, in the dev container, and in GitHub Actions:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
```

## GitHub Actions

`.github/workflows/ci.yml` runs two checks:

- builds and tests the colcon workspace in `ros:humble-ros-base`
- builds the repository `Dockerfile` to catch development image regressions

The workflow is intentionally small and uses package manifests plus `rosdep`
as the dependency source of truth.

## Docker

Build the development image:

```bash
docker build -t ros2-medical-physical-ai-dev .
```

Run it with the workspace mounted:

```bash
docker run --rm -it --network=host \
  -v "$PWD:/workspace" \
  -w /workspace \
  ros2-medical-physical-ai-dev
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install
```

## Dev container

Open the repository in a dev-container-compatible editor. The dev container:

- builds from the root `Dockerfile`
- mounts the repository at `/workspace`
- runs `rosdep install` and `colcon build --symlink-install` after creation
- uses `--network=host` so ROS 2 discovery works for local multi-terminal demos

If you need a clean rebuild:

```bash
rm -rf build install log
colcon build --symlink-install
```
