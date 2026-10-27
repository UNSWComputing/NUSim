# NUSim

A self-contained **MuJoCo** simulator for the **Booster Robotics K1** humanoid, used by
[NUbots](https://nubook.nubots.net) for RoboCup development. It replaces **both** Booster's gated Webots build
**and** their closed-source `mck` motion runner with one inspectable, NUClear-based simulator that speaks
the **Booster SDK over FastDDS** — the same wire protocol as the real robot — so `NUbots_K1` binaries drive
it **unchanged**.

```
sim/soccer  (native macOS / Linux docker container, NUClear)
   MuJoCo physics + servo/mode machine + head camera + GLFW viewer + GameController supervisor
        │  Booster SDK over FastDDS (domain 0)            │  camera frames → shared memory
        ▼                                                 ▼
 NUbots_K1  (K1WalkPolicy / K1GetUpPolicy, behaviour)   input::K1Camera → ImageCompressor → NUsight
        ▲
        │  RoboCup GameController (UDP, direct)
```

- **One process** simulates the K1 body — physics, servos, mode machine and fall detection — while the
  locomotion policies run on the NUbots side and drive it through CUSTOM mode + `rt/joint_ctrl`, exactly as
  they drive the real robot. No Booster downloads, no separate motion runner.
- **Vision** is rendered by MuJoCo and handed to NUbots' *unchanged* `input::K1Camera` over shared memory,
  so `CompressedImage` reaches **NUsight** exactly as on the real robot.
- **GameController** is heard directly by NUbots over the network; the sim additionally runs a supervisor
  that places the ball/robots per game phase.

## Quick start

On **Linux**, requirements are **Docker** (everything else is baked into the image). Full setup, config reference, and
troubleshooting are in **[docs/K1_MUJOCO_SETUP.md](docs/K1_MUJOCO_SETUP.md)**.

```bash
./b configure                   # configure the sim build
./b build                       # build the sim in docker (first run builds the image)
./b run sim/soccer              # launch the soccer sim (viewer + DDS + camera + supervisor)
```

On **macOS**, `./b` builds and runs natively using Apple's OpenGL for the viewer and offscreen camera:

```bash
brew install cmake ninja boost yaml-cpp openjdk@17
export JAVA_HOME="$(brew --prefix openjdk@17)/libexec/openjdk.jdk/Contents/Home"
export PATH="$JAVA_HOME/bin:$PATH"
./mujoco/tools/install_deps.sh   # pinned MuJoCo + DDS libraries + IDL generator
./b configure
./b build
./b test
./b run sim/soccer
```

See [native macOS setup](docs/K1_MUJOCO_SETUP.md#native-macos) for prerequisites, headless rendering and
the networking/camera limitations when connecting to a Linux container. Use `K1SIM_BACKEND=docker`
to select Docker explicitly, or `K1SIM_BACKEND=native` for a native Linux build.

Then drive it from **[`NUbots_K1`](https://github.com/NUbots/NUbots_K1)** exactly as against the real robot:

```bash
cd ~/NUbots_K1
./b run keyboardwalk
# e = walk on/off, w/s/a/d = velocity (0.01 m/s per press — tap ~10x), z/x = turn, arrows = head
```

The Booster SDK **requires** a FastDDS profiles file (participant profile `booster_dds`) or it refuses to
create its DDS participant (`Failed to create participant`). `./b run` now defaults
`FASTRTPS_DEFAULT_PROFILES_FILE` to the repo's copy, so no flag is needed — see
[Networking](#networking) for the caveat if you pass `--environment` yourself.

The full autonomous stack works too — `./b run nusim/behaviour` has the robot find the ball by vision and
dribble it goalward. See [docs/K1_MUJOCO_SETUP.md](docs/K1_MUJOCO_SETUP.md) for the end-to-end walkthrough
and the NUbots_K1-side requirements (`skill::K1WalkPolicy` + `skill::K1GetUpPolicy` in the role,
`VisualMesh.yaml` camera entry).

> Older revisions of this README told you to mount an OpenVINO overlay over the image, because the
> NUbots_K1 image was built with `ENABLE_INTEL_CPU=OFF` and had no CPU inference device at all
> (`Device with "CPU" name is not registered`). Fixed upstream — the image now ships
> `libopenvino_intel_cpu_plugin.so`, and inference prefers TensorRT on the GPU anyway. No overlay,
> no `--volume`, no `LD_LIBRARY_PATH`.

## Command reference

`./b` selects a native workflow on macOS and the container workflow on Linux; everything after the role name passes straight through to the
binary (`argparse.REMAINDER`), so sim flags are never parsed by `./b`.

| Command | Description |
| --- | --- |
| `./b configure [-i] [--clean] [--set-role R] [--unset-role R]` | CMake-configure with the selected backend; `-i` opens `ccmake`, `--clean` wipes the build dir. |
| `./b build [targets...]` | Ninja build with the selected backend (default: everything; e.g. `./b build sim-soccer`). |
| `./b run <role> [args]` | Run `bin/<role>` natively or in the container. Only role today: `sim/soccer`. |
| `./b roles` | List sim roles and their enabled/disabled state. |
| `./b image` | (Re)build the docker toolchain image. |
| `./b test` | Build and run the C++ unit tests with the selected backend. |

### `sim/soccer` flags

Parsed in [`mujoco/shared/CliOptions.hpp`](mujoco/shared/CliOptions.hpp); `--help` prints the same list.

| Flag | Default | Description |
| --- | --- | --- |
| `--headless` | off | Run without the GLFW viewer window (CI / headless servers). Physics, DDS and the camera bridge all still run. |
| `--field <name>` | `simulation.yaml`'s `field` (`middle`) | Field to play on, one of `simulation.yaml`'s `fields`: `middle` — the RoboCup 2026 Humanoid Soccer League M-Field (14 × 9 m), the field of the Middle Division the K1 plays in; `kidsize` — RoboCup KidSize under the pre-2026 rules (9 × 6 m); `no-field` — a bare flat floor with no field or ball. NUbots' `FieldDescription.yaml` `field_type` must match for localisation. |
| `--game <n>` | none | A match of `n` robots a side, 1–11 (so 2–22 K1s), on `simulation.yaml`'s `game.field` (`middle`). Sets the field and robot count itself, so it can't be combined with `--field` or `--robots`. Team 1 is in the -x half, team 2 is its mirror image; by default each team lines up just off the touchlines facing into the field, two at a time on alternate sides, starting level with its penalty mark and filling towards the halfway line. The main robot is team 1's first. Only the main robot is controlled, the rest are PD-held at the `ready` pose like `--robots` extras. `--keyframe` still sets the main robot's pose, moved to its game spot. |
| `--on-field-positions` | off | With `--game`, start in kickoff positions facing the opponent's goal instead of on the touchlines: the attacker (the main robot), then the goalkeeper (2v2+), left wing (3v3+), right wing (4v4+), and the 5th to 11th robots spread evenly through the rest of the team's half (outside the centre circle). Positions are in `simulation.yaml`'s `game`. |
| `--config-dir <dir>` | `mujoco/config` | Config directory to read the YAML from. |
| `--keyframe <name>` | `ready` | Startup keyframe for the **main** robot (e.g. `lying_front` to start fallen and exercise the get-up chain). |
| `--rtf <factor>` | `simulation.yaml`'s `real_time_factor` | Real-time factor; `0` = free-run (uncapped, for tests/sweeps). |
| `--robots <n>` | `1` | **Total** K1s on the field, 1–20. `n−1` extra copies are attached via MuJoCo's `mjSpec` attach API with `subNN_` name prefixes, so the main robot's unprefixed joint/sensor names — and every DDS and shared-memory contract — are untouched. Extras spawn standing on a 5×4 grid clear of the `y = 0` main-robot/ball lane and are PD-held at the `ready` pose: uncontrolled obstacles for dribbling and navigation practice, not extra DDS endpoints. Sim resets re-place them; `--keyframe` does not apply to them. |
| `--help`, `-h` | — | Print the flag list and exit. |

```bash
./b run sim/soccer --headless                            # no viewer window (CI / server)
./b run sim/soccer --field kidsize                       # the KidSize field instead of the M-Field
./b run sim/soccer --field no-field                      # bare robot on a flat floor, no field/ball
./b run sim/soccer --game 3                              # 3 a side lined up on the M-Field touchlines
./b run sim/soccer --game 5 --on-field-positions         # 5 a side in kickoff positions
./b run sim/soccer --rtf 0                               # free-run (uncapped real-time factor)
./b run sim/soccer --keyframe lying_front                # start fallen, to exercise GetUp
./b run sim/soccer --robots 5                            # 4 extra K1s on the field (max 20 total)
```

### Environment variables

| Variable | Effect |
| --- | --- |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | **Required** by the Booster SDK (see Networking). `./b run` defaults it to the repo's copy. |
| `K1SIM_CONFIG_DIR` | Config directory, same as `--config-dir` (the flag wins). |
| `K1_DDS_UDP_ONLY` | `1` strips the FastDDS shared-memory transport, leaving UDPv4 only — the fallback when the sim and NUbots are on opposite sides of a docker boundary. Equivalent to `udp_only: true` in `config/dds.yaml`. |
| `K1SIM_BACKEND` | `native` or `docker`; defaults to native on macOS and Docker elsewhere. |
| `K1SIM_BUILD_DIR` | Build directory relative to `mujoco/` (default `build-native` or `build-docker` for the selected backend). |

### Viewer keys

Standard MuJoCo mouse camera (left-drag rotate, right-drag pan, scroll zoom, ctrl/shift modifiers for
perturbation drags), plus:

| Key | Action |
| --- | --- |
| `Backspace` | Reset the sim to its startup state (and re-place `--robots` extras). |
| `F` | Shove the robot over — a deterministic topple for testing fall detection / `GetUp`, harder than a mouse-drag perturb, which the push-randomised policy usually rides out. |
| `Esc` | Close the window. |

`Space` is **not** a pause: physics stepping belongs to `module::Simulation`'s 1 kHz thread and no pause
switch is exposed yet.

## Networking

The sim speaks the **Booster SDK wire protocol over FastDDS**, so `NUbots_K1` binaries connect to it with
no build flag, no shim and no code change — the same code path they use against the real robot.

- **DDS domain 0**, because NUbots' `platform::Booster::HardwareIO` hardcodes `ChannelFactory::Init(0)`.
  Configurable in [`mujoco/config/dds.yaml`](mujoco/config/dds.yaml), but changing it means changing NUbots too.
- **Transport** is UDPv4 + shared memory by default. `./b run` gives the container `--network host --ipc host`
  precisely so both survive the docker boundary; set `K1_DDS_UDP_ONLY=1` (or `udp_only: true`) if only UDP works
  in your setup.
- **Topics** — sim publishes `rt/low_state` (IMU + serial/parallel motor state, 50 Hz), `rt/odometer_state`
  and `rt/odom` (the controller's odometry: ground truth, or drifting with the error model in
  [`mujoco/config/odometry.yaml`](mujoco/config/odometry.yaml) enabled), `rt/head_pose`, `rt/fall_down` (on change,
  ≥1 Hz keepalive) and `rt/battery_state` (constant SOC); sim subscribes to
  `rt/joint_ctrl` (`LowCmd`, PD-tracked at 1 kHz in CUSTOM mode). The RPC pair `rt/LocoApiTopicReq` /
  `rt/LocoApiTopicResp` serves `CHANGE_MODE`, `MOVE`, `ROTATE_HEAD`, `LIE_DOWN`, `GET_UP`,
  `GET_UP_WITH_MODE`, `VISUAL_KICK` and `GET_MODE`; unimplemented `api_id`s are accepted with a warning
  (`unknown_api_status` in `dds.yaml`). Full wire details in
  [`mujoco/module/SdkBridge/PROTOCOL.md`](mujoco/module/SdkBridge/PROTOCOL.md).
- **FastDDS profiles are mandatory.** `ChannelFactory::Init(0)` refuses to create its participant unless
  `FASTRTPS_DEFAULT_PROFILES_FILE` points at an XML containing a participant profile named `booster_dds`
  (`Failed to create participant`). `./b run` sets it for you. If you pass `--environment` yourself, note it
  takes ONE comma-separated argument and **replaces** the default — re-include the FastDDS var or all DDS dies.
- **Vision is not DDS.** Head-camera frames are rendered offscreen and written to a Boost.Interprocess
  shared-memory segment (`_boostercamera_head_rgb`, rgb8 544×448 @ 30 Hz) whose header matches NUbots'
  `input::K1Camera` byte-for-byte; head pose goes to a second segment (`_head_pose`, `K1Sensors` "NBPO"
  layout) and is the only path torso tilt takes into NUbots, i.e. what makes fall detection and the get-up
  chain work. Segment names must match `K1Camera.yaml` / `K1Sensors.yaml` on the NUbots side. Only the left
  camera is rendered — `K1Camera` warn-retries harmlessly on the right one; stereo is future work.

## GameController

This is the **RoboCup GameController** (the UDP match-control broadcast), not a USB gamepad — there is no
joypad/SDL input path in NUSim, and manual driving is done from `NUbots_K1`'s `keyboardwalk` role.

- **NUbots hears the GameController itself**, directly and independently over its own socket. That path
  needs nothing from the sim.
- The sim additionally ships a **supervisor** ([`mujoco/module/Supervisor`](mujoco/module/Supervisor)) that
  listens to the same broadcast and moves MuJoCo bodies per game phase — ball to the centre circle on
  kickoff, penalised robots to the sideline — the job Webots' Supervisor role used to do. It parses
  **RoboCup GameControlData protocol version 20** (`RGme` header), receive-only: it never sends a reply.
- **It is OFF by default** (`enabled: false` in [`mujoco/config/supervisor.yaml`](mujoco/config/supervisor.yaml)).
  With `--network host`, NUbots binds GameController port **3838** on the same host; a second binder steals
  it and breaks NUbots with `Unable to bind the UDP socket: Address already in use`. Enable the supervisor
  only when running the sim **standalone**, with no NUbots GameController on the host.
- With no GameController on the network it idles silently — no errors either way.
- Ball placement, per-robot team/player mapping, home and penalty poses, and which events reset the ball
  (`finished`, `goal`, `half_change`) are all configurable in `supervisor.yaml`. Exact RoboCup per-player
  penalty spacing is **not** implemented — the penalty spot is a single configurable pose.

## Locomotion policy

NUSim does **one thing: simulate**. It neither trains nor runs locomotion policies: the sim is a
servo-command listener (CUSTOM mode + `rt/joint_ctrl` LowCmd, PD-tracked at 1 kHz), and inference runs on
the NUbots side (`NUbots_K1` `module/skill/K1WalkPolicy` and `module/skill/K1GetUpPolicy`, OpenVINO,
50 Hz). Policies are trained in the NUbots
**[mujoco_playground fork](https://github.com/Tom0Brien/mujoco_playground)** (branch `feat/k1-training`,
`K1JoystickFlatTerrain` / `K1Getup` tasks) and exported to ONNX with `learning/export_k1_onnx.py`. The
walk observation/action interface is pinned in
**[docs/OBS_ACTION_CONTRACT.md](docs/OBS_ACTION_CONTRACT.md)** — anything that trains a walk policy for
the K1 must match it.

See [the MJWarp assessment](docs/MJWARP_ASSESSMENT.md) for the local CPU comparison and why native
MuJoCo remains the default for this interactive simulator.

## Layout

| Path | Description |
| --- | --- |
| `b`, `mujoco/b.py`, `mujoco/tools/` | NUbots-style `./b` command dispatcher (`run`, `build`, `configure`, `roles`, `image`). |
| `mujoco/roles/` | Role files (`sim/soccer.role`) → `bin/<role>` binaries. |
| `mujoco/module/` | NUClear modules: `Simulation`, `SdkBridge` (DDS), `Locomotion`, `Camera`, `Supervisor`, `Viewer`. |
| `mujoco/models/k1/` | Vendored MuJoCo K1 model (BSD-3, `booster_assets`) + RoboCup/flat scenes. |
| `mujoco/docker/` | Toolchain image + `k1sim.sh` (the container workflow `./b` wraps). |
| `docs/K1_MUJOCO_SETUP.md` | Setup, config reference, end-to-end with `NUbots_K1`, and troubleshooting. |
| `docs/OBS_ACTION_CONTRACT.md` | ONNX policy interface contract (obs/action layout the sim expects). |
| `mujoco/module/SdkBridge/PROTOCOL.md` | Booster SDK DDS wire surface: topics, message layouts, RPC `api_id`s. |

Forked from [NUWebots](https://github.com/NUbots/NUWebots); the Webots/NUgus simulation has been removed in
favour of the MuJoCo path (see git history if you need it).
