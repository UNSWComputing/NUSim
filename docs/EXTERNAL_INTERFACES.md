# External interfaces of the simulator container

How NUSim talks to the outside world: joint/sensor data out, commands in, and how several
robots are kept apart with one **DDS domain per robot**.

Sources: `mujoco/module/SdkBridge/` (`DdsParticipant.cpp`, `StatePublisher.cpp`, `PROTOCOL.md`),
`mujoco/config/{dds,simulation,supervisor,match_3v3}.yaml`, `mujoco/docker/k1sim.sh`.

## 1. Overview

The container runs with `--network host --ipc host`, so it shares the host's network stack and
`/dev/shm`. There is no port mapping. Everything below is visible on the host as if the simulator
were a native process.

```mermaid
flowchart LR
    subgraph C["NUSim container (--network host --ipc host)"]
        direction TB
        PHYS["MuJoCo physics<br/>1 kHz PD + servo/mode machine"]
        BR["module::SdkBridge<br/>one Fast-DDS DomainParticipant<br/>PER ROBOT"]
        CAM["module::Camera"]
        SUP["module::Supervisor<br/>(off by default)"]
        SCN["module::Simulation<br/>scenario listener<br/>(off by default)"]
        PHYS <--> BR
        PHYS --> CAM
    end

    subgraph H["Host / other containers"]
        direction TB
        P1["Player / NUbots_K1<br/>robot 1, domain 0"]
        P2["Player / NUbots_K1<br/>robot 2, domain 1"]
        PN["... robot N, domain N-1"]
        GC["RoboCup GameController"]
        TOOL["Test launcher / scripts"]
    end

    BR -- "RTPS over UDPv4 or SHM<br/>domain 0: state out, cmd in" <--> P1
    BR -- "RTPS, domain 1" <--> P2
    BR -- "RTPS, domain N-1" <--> PN
    CAM -- "Boost.Interprocess shared memory<br/>(not DDS, not network)" --> P1
    GC -- "UDP 3838 broadcast<br/>GameControlData v20" --> SUP
    GC -- "UDP 3838" --> P1
    TOOL -- "UDP 47200 loopback, JSON" --> SCN
```

## 2. Data direction: joints out, commands in

```mermaid
flowchart LR
    subgraph SIM["Simulator (per robot, domain D)"]
        MJ["MuJoCo state<br/>q, dq, tau, IMU, base pose"]
        SP["StatePublisher"]
        RPC["RpcServer"]
        LOC["Locomotion<br/>servo PD @ 1 kHz"]
        MJ --> SP
        RPC --> LOC
        LOC --> MJ
    end

    subgraph PLAYER["Player (same domain D)"]
        SUBS["ChannelSubscriber"]
        POL["Policy / behaviour"]
        PUBC["ChannelPublisher"]
        CLI["LocoClient (RPC)"]
        SUBS --> POL
        POL --> PUBC
        POL --> CLI
    end

    SP -- "rt/low_state  50 Hz<br/>IMU + motor q/dq/tau (serial + parallel)" --> SUBS
    SP -- "rt/odometer_state, rt/odom" --> SUBS
    SP -- "rt/head_pose, rt/joint_states, rt/imu/data" --> SUBS
    SP -- "rt/fall_down (on change, >=1 Hz)" --> SUBS
    SP -- "rt/battery_state" --> SUBS
    SP -- "rt/nusim/gt/world_v1 (ground truth)" --> SUBS

    PUBC -- "rt/joint_ctrl  LowCmd_<br/>(CUSTOM mode, PD targets)" --> LOC
    CLI -- "rt/LocoApiTopicReq  (RELIABLE)<br/>CHANGE_MODE, MOVE, ROTATE_HEAD,<br/>GET_UP, LIE_DOWN, VISUAL_KICK" --> RPC
    RPC -- "rt/LocoApiTopicResp  (RELIABLE)" --> CLI
```

| Direction (from sim) | Topic | Rate / QoS |
|---|---|---|
| out | `rt/low_state` (joints + IMU) | 50 Hz, RELIABLE, VOLATILE, KEEP_LAST(5) |
| out | `rt/odometer_state`, `rt/odom`, `rt/head_pose`, `rt/joint_states`, `rt/imu/data`, `rt/battery_state` | same QoS |
| out | `rt/fall_down` | on change, at least 1 Hz keepalive |
| out | `rt/nusim/gt/world_v1` | ground truth for native players |
| in | `rt/joint_ctrl` (`LowCmd`) | PD-tracked at 1 kHz in CUSTOM mode |
| in | `rt/LocoApiTopicReq` | RELIABLE, KEEP_LAST(10) |
| out | `rt/LocoApiTopicResp` | RELIABLE, KEEP_LAST(5) |

Topic names are identical in every domain. The domain is what separates robots, not the name.

## 3. Multiple robots: one domain id each

Each robot gets its own `DdsParticipant` (`SdkBridge.cpp:51`,
`DdsParticipant(robot.dds_domain, udp_only, local_only)`). A DDS participant only discovers and
exchanges data with peers in the same domain, so robot 2's `rt/joint_ctrl` can never reach robot 1.

```mermaid
flowchart TB
    R["--game N  or  --match match_3v3.yaml"] --> Q{"roster given?"}
    Q -- "yes" --> M["use dds_domain from roster<br/>robot 1..6 -> domains 0..5"]
    Q -- "no" --> D["dds.yaml: domain (0)<br/>robot k -> domain + k"]
    M --> S
    D --> S

    subgraph S["Simulator container"]
        direction LR
        B1["Participant domain 0"]
        B2["Participant domain 1"]
        B3["Participant domain 2"]
        B4["Participant domain 3"]
        B5["Participant domain 4"]
        B6["Participant domain 5"]
    end

    B1 <--> P1["Player 1<br/>NUSIM_ROBOT_ID=1<br/>NUSIM_DDS_DOMAIN=0"]
    B2 <--> P2["Player 2<br/>domain 1"]
    B3 <--> P3["Player 3<br/>domain 2"]
    B4 <--> P4["Player 4<br/>domain 3"]
    B5 <--> P5["Player 5<br/>domain 4"]
    B6 <--> P6["Player 6<br/>domain 5"]
```

The player must call `ChannelFactory::Init(<its domain>)` before any hardware module starts.
NUbots' stock `HardwareIO` hardcodes `Init(0)`, so unmodified NUbots can only drive robot 1.

## 4. Ports and protocols

| What | Protocol | Port | Notes |
|---|---|---|---|
| Joint state out, commands in (all `rt/*` topics) | RTPS (DDS wire protocol) over **UDPv4**, plus **shared memory** by default | see table below | Fast-DDS 2.13, Booster SDK types |
| Camera frames | Boost.Interprocess shared memory, segment `_boostercamera_head_rgb` (rgb8 544x448 @ 30 Hz) | none | host-local only; needs `--ipc host` |
| Head pose to NUbots | shared memory, segment `_head_pose` | none | host-local only |
| GameController | UDP broadcast, GameControlData v20 | **3838** | NUbots binds it; sim Supervisor only if `enabled: true` |
| Scenario control | UDP, JSON | **47200**, loopback | `simulation.yaml: scenario_port`, 0 disables |

### RTPS port numbers per domain

The repo does not set explicit ports. It uses Fast-DDS default port mapping
(PB = 7400, DG = 250, PG = 2, d0 = 0, d1 = 10, d3 = 11), where `pid` is the participant index
within a process on that domain (0 for the first):

| Traffic | Formula | Domain 0 | Domain 1 | Domain 2 | Domain 3 | Domain 4 | Domain 5 |
|---|---|---|---|---|---|---|---|
| Discovery, multicast (239.255.0.1) | 7400 + 250 x D | 7400 | 7650 | 7900 | 8150 | 8400 | 8650 |
| Discovery, unicast | 7410 + 250 x D + 2 x pid | 7410 | 7660 | 7910 | 8160 | 8410 | 8660 |
| User data, unicast | 7411 + 250 x D + 2 x pid | 7411 | 7661 | 7911 | 8161 | 8411 | 8661 |

So separate domains also mean separate UDP port ranges, which you can see in `ss -ulpn`.
These are derived from the Fast-DDS defaults, not read from a config in this repo. Confirm on a
running sim with `ss -ulpn | grep -E '7[4-9]|8[0-6]'` if it matters. A custom
`FASTRTPS_DEFAULT_PROFILES_FILE` could change them.

### Transport selection

```mermaid
flowchart TD
    A["DdsParticipant(domain, udp_only, local_only)"] --> B{"local_only?<br/>dds.yaml"}
    B -- "yes" --> L["UDPv4 on 127.0.0.1 only<br/>non-blocking send<br/>player must set it too"]
    B -- "no" --> C{"udp_only?<br/>dds.yaml or K1_DDS_UDP_ONLY=1"}
    C -- "yes" --> U["UDPv4 only<br/>use across a docker boundary<br/>without shared /dev/shm"]
    C -- "no" --> D["UDPv4 + SHM (default)<br/>same-host peers use SHM,<br/>others fall back to UDP"]
```

With the default transport, two processes on the same host mostly talk over shared memory, so you
will see little UDP traffic for data even though UDP ports are open for discovery.

## 5. Caveats

- `FASTRTPS_DEFAULT_PROFILES_FILE` must point to an XML with a participant profile named
  `booster_dds`, or the Booster SDK fails with `Failed to create participant`. `./b run` sets it.
- The Supervisor is off by default because both it and NUbots would bind UDP 3838. A second binder
  breaks NUbots with `Address already in use`.
- Port 3838 is shared by every robot. GameController is not per-domain.
