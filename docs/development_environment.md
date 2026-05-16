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

To also run the CI runtime smoke test locally:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
bash scripts/ci_launch_topic_smoke.sh
```

The smoke test starts `icu_multi_patient.launch.py` for `patient_01` and
verifies one message from each representative topic:

- `/patient_01/patient_vitals`
- `/patient_01/alerts`
- `/patient_01/control_actions`

To exercise a closed-loop action in the smoke test, run the same script with a
SpO2-drop scenario and an expected control rule:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
SCENARIO=spo2_drop \
CONTROL_LOW_SPO2=99.0 \
EXPECTED_CONTROL_RULE_ID=control.oxygen_boost \
  bash scripts/ci_launch_topic_smoke.sh
```

To run the rosbag reproducibility smoke test locally:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
bash scripts/ci_rosbag_replay_smoke.sh
```

The rosbag smoke test records `/patient_01/patient_vitals`, replays the bag
through `icu_replay.launch.py` without starting a sensor, and verifies that
`/patient_01/alerts` receives a replay-generated alert.

## GitHub Actions

`.github/workflows/ci.yml` runs these checks:

- builds and tests the colcon workspace in `ros:humble-ros-base`
- starts the launch graph and verifies representative topics with
  `ros2 topic echo --once`
- runs a SpO2-drop closed-loop smoke test and verifies
  `control.oxygen_boost`
- records and replays a rosbag, then verifies replay-generated alerts
- builds the repository `Dockerfile` to catch development image regressions

The workspace job uploads `.ci_artifacts/` as `ros2-ci-smoke-artifacts` so
failed smoke tests keep their launch, echo, rosbag record, and rosbag replay
logs for debugging.

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
