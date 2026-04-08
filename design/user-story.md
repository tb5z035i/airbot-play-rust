# AIRBOT Play Rust Driver User Story

## Summary
This crate provides a Rust driver for the CAN-based AIRBOT Play arm and its supported end effectors. The crate must work both as a Rust library and as standalone executables. The library is the core product. Executables are adapters on top of the same semantics. For transport serving, the current requirement is only the WebSocket executable, but the probe and diagnostic executables are also required utility tools.

The design is intentionally simple:
- two kinds of feedback;
- two kinds of commands;
- three arm states;
- two end-effector states;
- motor-level CAN parsing;
- one built-in kinematics and dynamics layer with a unified `FK / IK / FD / ID` interface, initially backed by Pinocchio over FFI.

## Primary User Story
As a robotics application developer, I want to use a Rust crate that can talk to AIRBOT Play over SocketCAN, expose the arm's realtime joint feedback, answer slow parameter queries, and control the arm in free-drive and command-following modes, so that I can use the robot directly from Rust or through a WebSocket process without changing the control semantics.

## Hardware Scope

### Supported motor types
- `OD`
- `DM`

`ODM` is explicitly out of scope.

### Supported robot composition
- AIRBOT Play arm: `OD, OD, OD, DM, DM, DM`
- AIRBOT G2: one `DM`
- AIRBOT E2: one `OD`

### Arm communication scope
Communication to the arm is not only communication to the six motors.

The AIRBOT Play arm also has board-level communication paths:
- play base board;
- play end board.

Some identifiers, metadata, and configuration live on these boards rather than on the six joint motors.

### Parsing rule
CAN frame parsing is maintained at the motor level for motor communication, just like the legacy driver. In addition, the crate must support board-level parsing for arm-related base-board and end-board communication. The arm and end-effector layers are composition layers on top of motor parsers and board parsers. They do not own low-level frame parsing.

The frame protocol and the actual CAN transport should be separate:
- protocol modules own frame parsing and frame generation;
- CAN I/O modules own actual sending and receiving on SocketCAN.

## Feedback Model
The crate exposes two kinds of feedback.

These are feedback categories, not required Rust type names. The implementation may expose several structs or streams inside each category, and the API does not need one single giant response struct.

### 1. Realtime response
This is the high-frequency response parsed from motion-related CAN frames. For the arm, it includes:
- joint positions;
- joint velocities;
- joint torques;
- timestamp;
- validity.

This feedback is intended for control loops and monitoring.

### 2. Requested response
This is a slower response produced only after explicit requests. It includes:
- serial number;
- product or board identifiers;
- end-effector type;
- base-board and end-board parameters;
- motor parameters;
- board-level configuration values;
- ping or liveness replies;
- other non-realtime parameter values.

This feedback is available even when the arm is disabled.

## Warning Events
The crate must also expose warning events for non-fatal runtime problems.

Warning events are not a third response category. They are side-channel events used to report degraded behavior, timing problems, or late replies while keeping the two response categories unchanged.

Examples include:
- requested responses not obtained within the configured timeout;
- realtime motion feedback not updated in time;
- control loop rate not reaching the expected 250 Hz target;
- repeated control tick overruns;
- command slot contents becoming too old while still being replayed.

Warning events should be observable by both readonly clients and control clients.

## Command Model
The crate accepts two kinds of commands.

### 1. Non-realtime commands
Non-realtime commands are for:
- arm state changes;
- parameter queries;
- ping or liveness checks.

These commands do not participate in the high-frequency control loop.

### 2. Realtime commands
Realtime commands are for:
- joint-space control targets;
- task-space control targets.

Realtime commands go through a fixed-rate control loop. The user does not need to send commands at the hardware servo frequency. The crate keeps the latest valid target in a slot or buffer and replays it at the loop rate.

## Client Access Modes

| Mode | Allowed operations |
| --- | --- |
| `readonly` | subscribe to realtime responses, request non-realtime responses such as params or ping |
| `control` | everything in `readonly`, plus arm state changes and realtime control commands |

A readonly client must not be able to change robot state or send motion commands.
It should still be able to receive warning events.

## Robot Arm States
The arm has exactly three states.

### 1. Disabled
This is the initial state.

Behavior:
- the arm does not accept realtime control commands;
- the arm does not expose realtime arm feedback through the public control stream;
- requested responses such as SN and parameter queries remain available;
- clients may still ask the arm to change state through non-realtime commands if they have control permission.

### 2. Free drive
In this state the arm should be draggable by hand while gravity is compensated.

Behavior:
- the control loop reads the latest joint positions;
- the crate uses inverse dynamics to compute gravity compensation torques;
- the computed torques are multiplied by the AIRBOT gravity compensation coefficient vector;
- the mounted end-effector type determines which URDF and coefficient set are used;
- the arm instance does not own end-effector control or end-effector state, but it must still know the mounted end-effector type because that changes the model used for gravity compensation.

### 3. Command following
In this state the arm follows external commands.

Behavior:
- the crate always uses MIT mode for command following;
- the user may send either joint-space or task-space commands;
- task-space commands are converted to joint targets by the model layer before being sent to hardware;
- the command loop stores the latest target in a slot or buffer and keeps replaying it at the loop frequency;
- target velocity is always zero because the hardware velocity feedback is too noisy for target tracking.
- if the effective control rate falls below the expected 250 Hz target, the crate should emit warning events.

The command-following control law for this project is:

```text
torque = kp(current_pos - target_pos)
       + kd(current_vel - target_vel)
       + ff_torque
```

Where:
- `target_vel = 0` for all joints;
- `ff_torque` is the gravity compensation torque from inverse dynamics and the AIRBOT gravity coefficient vector.

The recommended default gains are:

```text
kp = [200, 200, 200, 50, 50, 50]
kd = [3, 3, 3, 1, 1, 1]
```

## End-Effector Model
The end effectors are simpler than the arm.

### Scope
- The arm instance does not own end-effector control or end-effector state.
- End effectors are independent logical instances that happen to share the CAN bus.

### End-effector states
Each end effector has exactly two states:
- `disabled`
- `enabled`

### End-effector feedback and control
- End-effector feedback is derived from its single motor state.
- The crate must convert motor position, velocity, and torque to actual parallel position, velocity, and torque using the same transforms as the legacy driver.
- AIRBOT G2 is read-write.
- AIRBOT E2 is feedback-oriented and simpler.

## Kinematics and Dynamics
- The crate includes its own in-process kinematics and dynamics layer.
- The kinematics and dynamics layer exposes one unified backend-agnostic interface.
- The unified interface must cover:
  - forward kinematics;
  - inverse kinematics;
  - forward dynamics;
  - inverse dynamics.
- The initial implementation uses Pinocchio through a C++ FFI bridge.
- The public API must not be tied to Pinocchio-specific types so that future backends can be added later.
- URDF files are shipped inside the crate.
- The mounted end-effector type determines which bundled URDF is used for the arm model.
- The model layer is required for:
  - inverse dynamics for free drive;
  - inverse dynamics feedforward for command following;
  - task-space to joint-space conversion.

## Library and Executable Usage
- The crate must be usable directly as a Rust library.
- The crate must also support standalone executables that expose the same semantics through a transport layer.
- The long-term transport set may include WebSocket, DDS, and iceoryx2.
- The current transport server implementation only needs to provide WebSocket.
- In addition to the transport server, two utility executables are required:
  - a probe executable for finding AIRBOT instances on available CAN interfaces;
  - a diagnostic executable for semantic CAN inspection.

Transport choice must not change:
- robot states;
- command categories;
- feedback categories;
- readonly behavior;
- validation rules.

## Probe Executable
The crate must include a probe executable that is responsible for finding robot instances.

This executable should:
- scan candidate CAN interfaces;
- send the required probe traffic to identify AIRBOT Play instances;
- read back identifiers and metadata needed to distinguish instances;
- report enough information for the user or higher-level software to select the intended robot instance.

Instance discovery must not rely on the WebSocket executable. The probe executable is required.

## Diagnostics Tool
The crate must include a diagnostic tool that:
- listens on CAN;
- decodes frames using the same motor-level parsers as the main crate;
- decodes base-board and end-board frames using the same board parsers as the main crate;
- shows realtime motion feedback and requested responses in physical units;
- translates outgoing or observed control frames back into meaningful control operations for debugging.

This diagnostic tool should behave like `candump` plus semantic decoding. It is not a separate protocol implementation. It is another consumer of the same parsers and command encoders, and it serves as a validation tool for the protocol parsers.

## Acceptance Criteria
- The arm exposes exactly two feedback categories: realtime response and requested response.
- Warning events are emitted for non-fatal issues such as requested-response timeout, realtime-feedback timeout, and control rate falling below 250 Hz.
- The arm exposes exactly three states: `disabled`, `free drive`, and `command following`.
- Requested responses such as serial number and params are available while the arm is disabled.
- The crate supports board-level requested communication for the play base board and play end board in addition to motor-level parameter communication.
- Realtime commands are rejected unless the client has control permission and the arm is in the correct state.
- Free drive uses inverse dynamics plus AIRBOT gravity coefficients, and the mounted end-effector type changes the selected URDF.
- Command following always uses MIT mode with gravity feedforward and zero target velocity.
- The latest valid realtime command is held in a slot or buffer and replayed by the control loop.
- Motor-level parsing is preserved for `OD` and `DM`, and board-level parsing is supported for the arm base board and end board.
- Frame protocol logic is separated from the actual SocketCAN send and receive layer.
- The kinematics and dynamics layer exposes a unified `FK / IK / FD / ID` interface, with Pinocchio as the initial backend and future backends allowed without changing the public API shape.
- A probe executable is required to find available robot instances.
- A diagnostic executable is required and functions like `candump` plus semantic CAN-frame decoding for protocol-parser validation.
- The crate can be used directly from Rust and through a WebSocket executable with the same semantics.
