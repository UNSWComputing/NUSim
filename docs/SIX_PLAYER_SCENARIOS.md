# Six-player native match sessions

The native player and match launcher live in the sibling `NUbots_K1` checkout on
`feature/simulated-team-players`. Build that checkout's `team` role and this fork's
`mujoco/build-native/k1_mujoco_sim`. Then, from `NUbots_K1`:

```bash
# Repeatable acceptance scenarios, headless (70 seconds).
build-deps/venv/bin/python tools/native/run_match.py --verify --output /tmp/nusim-match-001
# Longer visual demo: both teams and all six players remain active throughout.
caffeinate -i build-deps/venv/bin/python tools/native/run_match.py \
  --duration 180 --video --output /tmp/nusim-match-demo-001
```

The launcher generates the complete six-robot roster, per-player configuration,
separate team communication ports and seven GameController destinations. It records
configuration, source/model provenance, per-process logs, scenario observations,
resource samples and separate functional/performance results. Output directories
must be new. Its shutdown handler stops every owned process.

`--video` records only the simulator render in a 1280 × 720 MP4 at 15 fps, with
team/player labels and no desktop or window borders. It starts once all six
controllers are ready and checks the finished movie duration. Install `ffmpeg`
and `ffprobe` on PATH to use it. Video and rendering overhead are measured in
addition to the headless acceptance session.

The companion guide `docs/NATIVE_MATCH.md` documents the packet adapter, schedule,
checks and measurement definitions. The launcher's GameController is a local packet
source using the official v20 layout. It does not run the official graphical
GameController or send robot replies. Decision-making stays in each player; no
central attacker or supporter assignment is introduced.

## Simulator additions

Both entry points now install NUClear's IOController. The minimal executable also
installs Supervisor, allowing the same headless binary to process GameController
packets and perform placements. Supervisor remains disabled in the default config;
the launcher enables it and generates placements for all six robots.

Supervisor configuration entries accept `robot_id` alongside body/team/player IDs.
Penalty/unpenalisation placements clear only that robot's controller, increment its
individual reset count and refresh MuJoCo's derived state. A transition from another
GameController phase back to INITIAL restores the whole match, including startup
poses, controllers, simulation time and reset generation. The initial INITIAL
packet does not reset an already fresh world.

`simulation.yaml: scenario_port` defaults to zero (disabled). When enabled, it accepts
loopback UDP JSON commands:

```json
{"action":"ball","x":0.0,"y":-2.0}
{"action":"fall","robot_id":3}
{"action":"reset"}
```

Ball coordinates refer to the sphere centre in world metres. Fall places only the
selected robot prone; other robot contexts and the ball are untouched. Reset is the
same full-world operation as the viewer's reset. Invalid requests are rejected.
The acceptance launcher resets through GameController's INITIAL transition.

`locomotion.yaml: low_cmd_timeout` defaults to zero, retaining existing one-shot
servo targets. The launcher sets 0.25 seconds: a disconnected controller then holds
the pose at timeout until a fresh command arrives.

## Measurements and limits

`SIM_METRICS` records paced RTF and average wall time spent in `mj_step` alone.
`CONTROL_METRICS` records each robot's new-command mailbox-to-application mean and
count. It excludes DDS transport, player inference and complete sensor-to-action
latency. The launcher measures CPU and RSS separately with `ps`.

The proposed headless acceptance target is at least **0.95 RTF in every measured
one-second window after warm-up**, while functional checks verify all six identities,
controller streams, both teams' communication, phases, penalties/placements,
relocation, fall detection, player and GameController reconnection and match reset.
Rendering needs a separate measurement. Repeatable initial conditions and scenario
scheduling do not imply identical learned-policy trajectories or robust competition
motion.

## Recorded demo

The full three-minute, six-player recording and measurements are saved locally in
`mujoco/build-native/sessions/3v3-demo-20261008/` (`demo.mp4`, `results.json`,
`video.json` and individual logs). The video is exactly 180 seconds, 1280 × 720
and 15 fps. Mean RTF was 1.000000; the minimum measured window was 0.999823.
All six players received both teammates and streamed independent servo commands.
The longer run also exposed a policy limitation: robot 125/1 was marked fallen
in about 72% of samples. See the companion guide for full resource measurements
and motion limitations. The headless scenario evidence is retained separately in
`mujoco/build-native/sessions/3v3-acceptance-20261008/`.
