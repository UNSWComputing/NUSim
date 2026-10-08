# Native motion-policy players

NUSim publishes `rt/nusim/gt/world_v1` in each robot's configured DDS domain alongside
its own Booster sensor and control topics. This enables a minimal native player build
in the companion `NUbots_K1` checkout, on `feature/simulated-team-players`.

The player's build and run guide, `docs/NATIVE_PLAYER.md`, lives in that checkout. Build NUSim first, then configure and build the companion's `player` role
with `NUBOTS_NATIVE_PLAYER=ON`. The role uses ONNX Runtime CPU for the selected walk,
kick and get-up checkpoints from `jmontano/nusim-local`, and the existing Director and
`K1Servos` arbitration.

Each player process selects `NUSIM_ROBOT_ID` and its matching `NUSIM_DDS_DOMAIN`.
For the example six-robot roster, robot IDs 1..6 use domains 0..5. The adapter supplies
perfect pose and localisation messages after localisation, so cameras and image
processing are not required. The minimal approach/settle/side-foot-kick behaviour
exercises low-level joint control; team tactics are subsequent work.

The companion's `tools/native/try_player.py` runs headless integration checks on
isolated domains 45 and 46. Acceptance requires physical approach and ball displacement
during the kick, followed by opposite movements from two independent player processes.
Model loading alone is not an acceptance result. The current checkpoint produces a
modest side-foot strike; passing accuracy has not been established.

The simulator tests additionally check ball geom offsets and rotational contributions
to centre velocity, capture stamps and reset generation. Existing single-robot wire
interfaces are preserved. Current team attack directions are the startup roster values;
GameController half changes are not yet wired into the snapshot.

Validated on 8 October 2026 on this Apple Silicon Mac: one native player approached
1.26 m and displaced the ball 8.6 cm during its kick, then remained standing. Two
independent processes moved +1.47 m and -1.40 m in the same world. The get-up policy
also completed recovery from `lying_front`. NUSim's macOS and Linux ARM64 suites
passed, including the new capture/reset tests. See the companion guide for repeatable
commands and the limits of this initial motion role.
