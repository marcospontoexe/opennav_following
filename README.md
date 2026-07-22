## opennav_following (jazzy) - 1.0.0-1

The packages in the `opennav_following` repository were released into the `jazzy` distro by running `/usr/bin/bloom-release --ros-distro jazzy --track jazzy opennav_following` on `Mon, 20 Jul 2026 17:20:58 -0000`

The `opennav_following` package was released.

Version of package(s) in repository `opennav_following`:

- upstream repository: https://github.com/marcospontoexe/opennav_following.git
- release repository: unknown
- rosdistro version: `null`
- old version: `null`
- new version: `1.0.0-1`

Versions of tools used:

- bloom version: `0.13.0`
- catkin_pkg version: `1.1.0`
- rosdep version: `0.26.0`
- rosdistro version: `1.0.1`
- vcstools version: `0.1.42`


# Nav2 Following (Jazzy port)

This package contains an action server for **dynamic object following**: the
robot keeps a configurable distance from a moving object (a person, a cart, a
robot, etc.), taking either a live pose from a topic or a TF frame as the
tracking source, driven by the same graceful control law used by the Nav2
Docking Framework (`opennav_docking::Controller`).

It is a single, self-contained package:

- `opennav_following`: the `FollowingServer` action server and its own
  `follow_object` action (`opennav_following/action/FollowObject`).

## Origin of this package

`opennav_following` and the `FollowObject` action were authored upstream by
Open Navigation LLC / Alberto J. Tudela Roldán and live on the
[`main`](https://github.com/ros-navigation/navigation2/tree/main/nav2_following)
branch of `ros-navigation/navigation2`, targeting the newer `nav2_ros_common`
abstraction (`nav2::LifecycleNode`, `nav2::SimpleActionServer`, etc.) that was
introduced after the Jazzy release branch was cut. Because of that,
`opennav_following` never shipped in the Jazzy distro (neither via `apt`
nor in the `jazzy` branch source), and the `FollowObject` action does not
exist in Jazzy's `nav2_msgs`.

**This repository is an independent Jazzy-API backport** of that same
package: same action fields and behavior, rewritten against the
`nav2_util`-based API that Jazzy's `nav2_util`/`opennav_docking` actually
expose, and packaged so it installs standalone on Jazzy (see below) — it is
**not** affiliated with or endorsed by Open Navigation LLC / `ros-navigation`,
and it is **not** distributed through the official Jazzy release of
`navigation2`. If/when upstream ships `opennav_following` for your ROS
distro, prefer that version; this package exists to cover Jazzy in the
meantime.

### License and attribution

This package is a derivative work of `nav2_following/opennav_following` from
[`ros-navigation/navigation2`](https://github.com/ros-navigation/navigation2)
(`main` branch), Copyright (c) 2024 Open Navigation LLC and Alberto J. Tudela
Roldán, licensed under [Apache-2.0](./LICENSE). The original copyright
notices are preserved at the top of every source file carried over from
upstream; files that were changed for this port carry an additional note
saying so, per the license's requirements. See the table below for exactly
what was changed.

## What changed to make it build on Jazzy

| Area | `main` branch (upstream) | This Jazzy port |
|---|---|---|
| Node base class | `nav2::LifecycleNode` (`nav2_ros_common`) | `nav2_util::LifecycleNode` |
| Action server | `nav2::SimpleActionServer<T>` (`SharedPtr`) | `nav2_util::SimpleActionServer<T>` (`unique_ptr`) |
| Parameters | `nav2_util::ParameterHandler<T>` template + separate `ParameterHandler` class | Declared inline in `on_configure`, plain member fields, one `dynamicParametersCallback` (mirrors `opennav_docking::DockingServer`'s own pre-`nav2_ros_common` pattern) |
| Publishers/Subscriptions | `nav2::Publisher`, `nav2::Subscription`, `nav2::qos::StandardTopicQoS` | `rclcpp_lifecycle::LifecyclePublisher`, `rclcpp::Subscription`, plain `rclcpp::QoS` |
| Action package | `nav2_msgs` ships `FollowObject.action` | This package ships its own `action/FollowObject.action` and generates `opennav_following::action::FollowObject` (Jazzy's official `nav2_msgs` doesn't have this action, and this repo can't publish a modified `nav2_msgs` under that same name) |
| Search-rotation control | `opennav_docking::Controller::computeRotateToHeadingCommand()` | **Not available** in Jazzy's `opennav_docking::Controller` (added upstream *after* Jazzy). Replaced with a small, self-contained accel-limited P controller inside `following_server.cpp` (see below) |

### Search-rotation control law (deviation from upstream)

When `search_by_rotating: true` and the object is momentarily lost, the
server rotates in place to try to re-detect it. Upstream does this by calling
`opennav_docking::Controller::computeRotateToHeadingCommand()`, a bang-bang,
acceleration-limited controller with a Torricelli-style slow-down curve, added
to `opennav_docking::Controller` alongside two new parameters
(`controller.rotate_to_heading_angular_vel`,
`controller.rotate_to_heading_max_angular_accel`). That method does not exist
in Jazzy's `opennav_docking` (checked against both `ros-jazzy-opennav-docking`
1.3.12 and the `1.3.12` source tree), and this package intentionally avoids
patching/rebuilding `opennav_docking` itself, since it's a shared dependency
already used in production by the docking station.

Instead, `FollowingServer::rotateToObject()` computes its own accel-limited
proportional command:

```
desired = clamp(2.0 * angular_error, -rotate_angular_velocity, rotate_angular_velocity)
command = current + clamp(desired - current, -max_step, max_step)   # max_step = rotate_angular_acceleration * dt
```

controlled by two package-local parameters: `rotate_angular_velocity` (default
`0.5` rad/s) and `rotate_angular_acceleration` (default `1.0` rad/s²). This
only affects the opt-in search-rotation behavior — `search_by_rotating`
defaults to `false`, so out of the box this deviation has no effect.

## Architecture

- `FollowingServer`: the lifecycle node and action server implementing
  `follow_object`.
- `opennav_docking::Controller`: reused as-is from the Docking Framework for
  the approach control law (`computeVelocityCommand`).
- `opennav_docking::PoseFilter`: reused as-is to low-pass filter the detected
  object pose.
- Tracking source: either a `geometry_msgs/PoseStamped` topic (`pose_topic`)
  or a TF frame (`tracked_frame`), selected per-goal.

The control loop:
1. Get the object's pose (from topic detection or TF lookup), filtered.
2. Compute a target pose `desired_distance` meters from the object, transform
   it into `base_frame`, and hand it to `Controller::computeVelocityCommand`.
3. If the target is reached, stop and hold position; if the object stays put
   for `static_object_timeout` seconds, report success.
4. If detection fails, retry up to `max_retries` times — either using the last
   known heading, or (if `search_by_rotating: true`) rotating in place to
   search for the object again.

## Interface

### `follow_object` action (`opennav_following/action/FollowObject`)

**Goal**
| Field | Type | Description |
|---|---|---|
| `pose_topic` | `string` | Topic to subscribe to for the object's pose. Mutually exclusive with `tracked_frame`. |
| `tracked_frame` | `string` | TF frame to track instead of a topic. |
| `max_duration` | `builtin_interfaces/Duration` | Maximum time to run the action (`0` = unlimited). |

**Result**
| Field | Type | Description |
|---|---|---|
| `error_code` | `uint16` | `NONE=0`, `TF_ERROR=901`, `FAILED_TO_DETECT_OBJECT=902`, `FAILED_TO_CONTROL=903`, `UNKNOWN=999` |
| `error_msg` | `string` | Human-readable error detail. |
| `total_elapsed_time` | `builtin_interfaces/Duration` | Total time spent on the action. |
| `num_retries` | `uint16` | Number of retries attempted. |

**Feedback**
| Field | Type | Description |
|---|---|---|
| `state` | `uint16` | `INITIAL_PERCEPTION=1`, `CONTROLLING=2`, `STOPPING=3`, `RETRY=4` |
| `following_time` | `builtin_interfaces/Duration` | Elapsed time since the action started. |
| `num_retries` | `uint16` | Retries attempted so far. |

## Configuration

| Parameter | Description | Type | Default |
|---|---|---|---|
| `controller_frequency` | Control loop frequency (Hz) | double | `50.0` |
| `detection_timeout` | Time (s) to wait for a fresh object detection | double | `2.0` |
| `rotate_to_object_timeout` | Time (s) allowed to rotate searching for the object per angle | double | `10.0` |
| `static_object_timeout` | Time (s) an object may stay still before the action succeeds (`<=0` disables) | double | `-1.0` |
| `linear_tolerance` | Distance (m) tolerance to consider the goal pose reached | double | `0.15` |
| `angular_tolerance` | Angle (rad) tolerance to consider the goal pose reached | double | `0.15` |
| `max_retries` | Maximum detection/control retries before failing | int | `3` |
| `base_frame` | Robot base frame | string | `"base_link"` |
| `fixed_frame` | Fixed frame for control (recommend a smooth odometry frame, **not** `map`) | string | `"odom"` |
| `desired_distance` | Distance (m) to keep from the object | double | `1.0` |
| `skip_orientation` | Ignore the detected orientation and point towards the object instead | bool | `true` |
| `search_by_rotating` | Rotate in place to search for a lost object (vs. keep last heading) | bool | `false` |
| `search_angle` | Search sweep angle (rad) relative to the current heading | double | `M_PI_2` |
| `transform_tolerance` | TF lookup tolerance (s) | double | `0.1` |
| `odom_topic` | Odometry topic for `OdomSmoother` | string | `"odom"` |
| `odom_duration` | `OdomSmoother` averaging window (s) | double | `0.3` |
| `controller.use_collision_detection` | **Must be `false`** — collision detection is not supported by the following server; `on_configure` fails if `true` | bool | `false` |
| `filter_coef` | Low-pass filter coefficient for the detected pose | double | `0.1` |
| `rotate_angular_velocity` | Max angular speed (rad/s) for the search-rotation controller *(Jazzy-port-only, see above)* | double | `0.5` |
| `rotate_angular_acceleration` | Max angular acceleration (rad/s²) for the search-rotation controller *(Jazzy-port-only, see above)* | double | `1.0` |

Plus every `controller.*` parameter of `opennav_docking::Controller` (the
approach control law), documented in `nav2_docking/README.md`.

## Building on Jazzy

Unlike the earlier revision of this port, this package is fully
self-contained — it builds against a **stock, unmodified** ROS 2 Jazzy
install (no patched `nav2_msgs`, no overlay tricks needed):

```bash
sudo apt install ros-jazzy-opennav-docking ros-jazzy-opennav-docking-core
mkdir -p ~/opennav_following_ws/src
cd ~/opennav_following_ws/src
git clone <this-repo-url> opennav_following
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select opennav_following
source install/setup.bash
```

## Note on `apt` distribution

This package is **not** distributed via `apt`/the official ROS package
index, and there's no plan to submit it there. A release was attempted
through the normal `rosdistro` process, but the Nav2 project maintainer
closed the pull request, noting that `opennav_*`-prefixed packages are
considered part of the Nav2 project's own namespace and that independent
releases under that prefix should be coordinated with the Nav2 team first
(see [ros/rosdistro#52776](https://github.com/ros/rosdistro/pull/52776)).
Build it from source as shown above.

## Known limitations

- The 3 launch-based smoke tests (`test_following_server_topic`,
  `_frame`, `_search`) can be flaky in constrained/virtualized environments:
  they hit a TF extrapolation race (~ms-scale) on the very first detection
  attempt which, combined with the abort-on-first-failure path in
  `rotateToObject()` (present upstream too, not introduced by this port),
  can cause the action to abort quickly instead of retrying. The deterministic
  tests (`test_following_server_unit` — 7/7 — and `test_following_server_skip_pose`)
  are unaffected and pass reliably.
- The search-rotation control law is a simplified stand-in for upstream's
  `computeRotateToHeadingCommand` (see above) — same purpose, different
  tuning/response curve.
- `test_following_server_unit` (gtest): all 7 assertions pass, but the test
  *process* segfaults during exit-time teardown after the `ErrorExceptions`
  case (which exercises a real action client/server round-trip over DDS).
  This is isolated to the test binary — the actual `opennav_following_node`
  executable starts, transitions through its lifecycle, and shuts down
  cleanly with exit code `0` (verified manually). Root cause not fully
  identified; suspected `rmw_fastrtps_cpp` teardown ordering interacting with
  this package's locally-generated (rather than `nav2_msgs`-provided) action
  typesupport plugin. CI/ctest will report this test as failed despite every
  assertion passing — check the gtest XML output, not just the exit code, if
  you rely on this in CI.
