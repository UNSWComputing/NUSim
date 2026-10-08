# Multi robot identities and ground truth contract

Six independent players share one NUSim physics world. Each player receives its own joint and
IMU data, plus a coherent snapshot of the ball and all robot poses. Adapters use that snapshot
to supply the player's normal localisation messages for teamwork tests with perfect information.

The simulator loads the roster and supports independent controllers and sensor streams.
The full-world DDS snapshot is published in every robot domain. A minimal native player adapter
in the companion repository supplies the pose and localisation inputs used by the motion policies. Existing single-robot Booster topic names and wire layouts
are preserved.

## Robot identities and match configuration

`mujoco/config/match_3v3.yaml` is the initial roster for `--game 3`. Team numbers 125 and 126 are
local example values; configure them to match the player processes and GameController.
`shared/sim/MatchConfig.hpp` loads and validates this schema independently of NUClear.
Select it with `--game 3 --match match_3v3.yaml`; the path is relative to the config directory
unless absolute. Without `--match`, IDs follow scene order, team IDs default to 125 and 126,
and domains increase from `dds.yaml`'s `domain`.

| Robot ID | Model index | Trunk body | Team ID | Player ID | DDS domain |
| --- | --- | --- | --- | --- | --- |
| 1 | 0 | `Trunk` | 125 | 1 | 0 |
| 2 | 1 | `sub01_Trunk` | 125 | 2 | 1 |
| 3 | 2 | `sub02_Trunk` | 125 | 3 | 2 |
| 4 | 3 | `sub03_Trunk` | 126 | 1 | 3 |
| 5 | 4 | `sub04_Trunk` | 126 | 2 | 4 |
| 6 | 5 | `sub05_Trunk` | 126 | 3 | 5 |

Robot IDs are match-wide and equal the zero-based model index plus one. Player IDs are team-local:
`(team_id, player_id)` identifies a player. List order determines model indices and follows
`--game`: every robot on the first team, followed by every robot on the second. The two teams
must contain the same number of robots, from one to eleven. Body and joint prefixes derive from
the model index through `robot_prefix`; configuration cannot assign a conflicting body name.

The validator rejects repeated DDS domains, repeated players within a team, incorrect robot IDs,
invalid team assignment, out-of-range values and unsupported schema versions. Team IDs are distinct
numbers in 1..255, and player IDs are in 1..11. The teams initially attack in opposite directions.
Model order does not assign goalkeeper or attacker behaviour; the player launcher must configure
roles explicitly.

## Transport and command routing

Each robot has one dedicated DDS domain containing its unchanged Booster state, low-level
commands and RPC topics. The simulator joins every configured domain. A player process joins
only its own domain, configured before any module initialises its DDS factory. Domain 0 preserves
the first robot's existing default. Domains 0..63 are a local configuration convention, with no
repeats within a match; concurrent matches must also use disjoint domain sets.

The simulator publishes an identical full-world snapshot in every robot domain on
`rt/nusim/gt/world_v1`, registered as `nusim_msgs::msg::dds_::WorldSnapshot_`. The IDL source is
`mujoco/idl/nusim_msgs/msg/WorldSnapshot.idl`; CMake generates its Fast-DDS support alongside the
existing types. Constants live in `shared/sim/GroundTruthContract.hpp`.

The intended snapshot QoS is best effort, volatile, keep-last with depth one. Each sample contains
all robots, so a dropped packet cannot create a mixture of separately received robot and ball
states. Players consume the latest complete sample and use local monotonic receipt time to detect
staleness. A full world reset changes the generation described below. Ordinary Booster messages
have no generation field. Simulator resets clear the selected controller's mailbox, mode and
fall state; resetting remote policy state and rejecting commands still in transport require a
player lifecycle mechanism in the adapter milestone.

Routing selects the robot from the receiving domain, then uses its internal robot ID. No robot ID
is added to Booster messages. Team communication runs separately from these domains; it must
retain actual player intentions and avoid cross-team delivery. Physics truth supplies poses and
velocities, not intended roles or decisions.

## Snapshot timing and reset semantics

`SnapshotStamp_` applies to every value in a `WorldSnapshot_`:

| Field | Meaning |
| --- | --- |
| `schema_version` | 1 for this layout and its semantics |
| `session_id` | A new UUID for each launch: 36 ASCII characters, without a terminator |
| `reset_generation` | Starts at zero; increments for every full world reset |
| `sample_sequence` | Starts at zero; increases with every captured snapshot within a generation |
| `step_count` | Completed physics steps since the latest full reset |
| `sim_time` | MuJoCo time in seconds since the latest full reset |
| `capture_time_unix_ns` | UTC capture time in nanoseconds, for mapping to player message timestamps |

The physics thread must copy the ball, every robot's sensors and pose, and the current team attack
directions together while holding the physics lock, after a completed step and updated forward
kinematics. DDS publication uses the copy outside that lock. The initial sample after a reset may
be taken at step zero, after forward kinematics. Capture cadence will default to the existing
state publish cadence, normally 50 Hz; the timestamp represents capture, not later publication.

A consumer binds to one session, accepts increasing generations and sequences, and rejects older
or duplicate samples. A new generation clears cached localisation, velocity history and policy
state. Rebinding to a different session requires clearing those caches too. Simulation time may
return to zero on reset; elapsed-time calculations must not span generations. UTC may jump and
must not be used for control intervals or receipt freshness. Pausing or stopping publication makes
input stale even if the last sample's simulation time remains unchanged.

Supervisor placements and ball relocations are state changes in the current generation. A full
reset restarts simulation time and clears all simulator controllers. Player processes will need
a lifecycle notification to clear their policy state when the ground-truth bridge is connected.

`SimCore::reset_robot(robot_id)` and the internal NUClear `RobotResetRequest` restore only that
robot's startup pose, zero its velocities and controls, and clear its controller mailbox and fall
state. Other robots, the ball, world time and the step counter are preserved. Controllers return
to `locomotion.yaml`'s initial mode. Backspace or `SimResetRequest` resets the whole world.
These resets do not add a Booster RPC or DDS reset topic. Shared physical contacts can still
affect neighbouring robots on subsequent physics steps.

## Coordinate frames and measurements

All distances use metres, angles radians, time seconds, joint torques N m and accelerations m/s².
The simulator world `{s}` is the field's fixed, right-handed frame: the centre spot is the origin,
+Z points upwards, +X points towards the positive-X goal, and +Y completes the frame. Team changes
and half changes never rotate the physics world.

`BodyTruth_` describes the `Trunk` body origin. `position_s`, `linear_velocity_s` and
`angular_velocity_s` are expressed in `{s}`. Linear velocity belongs to that origin, not the body's
centre of mass. `orientation_body_to_s` is a unit quaternion in **w, x, y, z** order and rotates a
body-frame vector into `{s}`. Copying free-joint angular velocity directly is insufficient if it is
expressed in a local frame; the producer must convert it to `{s}`.

`BallTruth_` gives the ball geom centre, its centre velocity and angular velocity in `{s}`. The
geom can be offset from its free-body origin. Centre velocity must include the angular contribution
for that offset. `valid=false` means no ball is available, such as on the bare-floor scene; its
numeric fields must then be ignored. A valid field-world sample includes every configured robot,
in model order, exactly once. A missing robot invalidates the sample rather than implying a pose
at the origin. All available measurements must be finite; orientation quaternions must have unit
norm. Default zero-filled generated objects are not valid measurements.

Each `RobotTruth_` also contains 22 joints in `JointIndexK1` serial order, including actuator torque,
and the simulated IMU. `orientation_imu_to_s` rotates IMU-frame vectors into `{s}`; gyro and
accelerometer readings remain in the IMU frame. Acceleration is specific force: a stationary,
upright supported robot reads approximately +9.81 along its upward IMU axis. Mode, fall state and
getting-up status use the existing Booster conventions.

The optional head pose uses the existing head frame 0.08 m above `Head_pitch`, expressed in the
yaw-only base footprint frame. Its quaternion rotates head-frame vectors into that footprint.
Consumers ignore head values when `valid=false`.

The two `TeamFrame_` entries carry the current team IDs and attack directions. Initial directions
come from the roster; GameController half changes will update them atomically with world state.
A team's behaviour field frame `{f}` points +X towards the goal it attacks. For a team attacking
+X, `{f}` equals `{s}`. For a team attacking -X, rotate both X and Y by 180 degrees about +Z.
Player odometry `{w}` remains distinct from this team frame.

## Player adapter outputs

The first adapter may choose `{w} = {s}` to simplify perfect-state testing, while explicitly
constructing all transforms. For a torso pose `Hst`, world-to-torso `Htw` is its inverse in this
choice of odometry frame. `Field.Hfw` maps odometry world into the team's current field frame.
The adapter must apply these conventions consistently to sensor kinematics, ball positions, robot
positions and velocities, including on the opposing team and after a half change.

| Player message | Required adapter behaviour |
| --- | --- |
| `RawSensors` | Only this robot's joints, IMU and status |
| `Sensors` | Preserve those measurements; provide consistent `Htw`, `Hrw`, `vTw`, joint, foot and head transforms |
| `localisation::Field` | Populate `Hfw` and set `localised=true` only with valid, fresh truth |
| `localisation::Ball` | Express position and velocity in `{w}`; populate confidence and measurement time |
| `localisation::Robots` | Other robots in `{w}`, team membership and team-local player IDs; distinguish opposing players with the same ID |

Adapters must preserve the player's expected camera-transform metadata where required, even when
no image is rendered. Perfect measurements use an explicitly configured covariance floor rather
than introducing random noise. Only one producer may own each normal localisation message:
image localisers are excluded from this role. Field geometry comes from the same selected field
configuration used by NUSim and the players.

A stale or invalid snapshot must not continue to produce confident localisation or fresh ball
measurements. Consumers must cease motion safely when required inputs expire; the timeout and
command fallback belong to the control/adapter milestones. No field-line detections or camera
processing are required for this initial mode.

## Implementation sequence

The roster is checked against the selected match size at startup. Robot contexts hold independent
maps, controllers and sensor identities; all controllers run before one shared physics step.
`RobotStatesUpdate` captures every robot together, and each DDS domain publishes only its own
Booster state. The main robot still supplies the viewer heartbeat and camera images.

Full-world capture and DDS publication now support the native player bridge, including the ball
geom centre and its velocity. Session UUID, reset generation, monotonically increasing sample
sequence and capture timestamps accompany every snapshot. Per-robot and full-world resets advance
the generation; only full-world reset rewinds physics time. Team directions currently come from
startup configuration; GameController half-change updates remain future work.

See [Native motion-policy player](NATIVE_MOTION_PLAYER.md) for the companion build, behavioural
checks and the adapter's current scope. Complete sensor kinematics and team communication remain
necessary before enabling full competition behaviours.

## Six-player sessions

See [native 3v3 scenarios](SIX_PLAYER_SCENARIOS.md) for the launcher, GameController
fan-out, per-robot placement/reset notifications and performance measurement scope.
