# Mini Rocket — 6-DOF 2-Axis TVC & Recovery Simulation

6-DOF flight-dynamics and control simulation of a miniature rocket with a 2-axis (pitch/yaw) thrust-vector-control system, plus a prototype flight-controller firmware baseline.

## Files

| File | Description |
|---|---|
| `rocket_6DOF_2Axis_TVC_Recovery.m` | Main simulation: 6-DOF rigid body, quaternion attitude, 2-axis TVC, powered recovery mission |
| `tvc_flight_controller/tvc_flight_controller.ino` | Teensy 4.1 firmware baseline: state machine, quaternion PID TVC, SD logging, servo-released parachute recovery |
| `rocket.m` | Earlier pitch-plane (2D) MATLAB baseline |
| `figures/` | Plots from the pitch-plane baseline |

## Run

```matlab
out = rocket_6DOF_2Axis_TVC_Recovery;            % with plots
out = rocket_6DOF_2Axis_TVC_Recovery(false);     % no plots
out = rocket_6DOF_2Axis_TVC_Recovery(false,12);  % other noise seed
```

Tested in GNU Octave 8.4. Run time is about 35 s.

## Model

- 17 states: position (3), velocity (3), quaternion (4), body rates (3), mass, TVC pitch angle, TVC yaw angle, actual thrust
- Frames: world x east, y north, z up. Body x = nose and thrust axis. Quaternion is scalar-first, body to world
- Integration: RK4, dt = 2.5 ms
- Thrust along body +x, deflected by pitch and yaw gimbal angles, with a constant thrust misalignment
- Exponential atmosphere, drag, normal force with CP/CG moment, roll damping, base wind plus gusts
- Propellant burn from Isp, moving CG, changing inertia tensor
- TVC servo: first-order lag, rate limit, angle limit. Engine: first-order throttle lag
- Sensors: gyro (noise and bias), accelerometer, barometer, GPS-like position and velocity

## Flight Software (100 Hz)

1. Pad calibration: gyro bias and initial alignment from the accelerometer
2. Estimator: gyro-integrated quaternion, 2nd-order complementary filters for position and velocity
3. Phase machine: pad, powered ascent, hover, lateral divert, controlled descent
4. Guidance: vertical-rate and lateral-position loops give a desired thrust vector, with a tilt limit
5. Quaternion attitude error: shortest arc from the nose axis to the desired thrust axis, in the body frame
6. PID on angular acceleration, rate damping on the measured gyro, anti-windup
7. Mapping to pitch and yaw gimbal angles using the thrust, CG and inertia estimates

## Mission

Pad → powered ascent to 30 m → hover → lateral divert to (12 m, 6 m) → controlled powered descent → touchdown.

## Parameters (assumed, not a vehicle design)

| Parameter | Value |
|---|---:|
| Initial / dry mass | 0.90 kg / 0.62 kg |
| Isp | 120 s |
| Thrust range | 4.4 – 22 N |
| TVC limit | ±7° |
| Servo time constant / rate limit | 0.05 s / 90 °/s |
| Throttle time constant | 0.08 s |
| Length / diameter | 0.70 m / 0.06 m |
| CG (wet to dry) / CP from nose | 0.38 to 0.33 m / 0.47 m |
| Attitude loop | wn = 6 rad/s, zeta = 0.8, Ki = 8 |

## Results

| Case | Touchdown speed | Tilt | Landing error |
|---|---:|---:|---:|
| Default seed (7) | 0.61 m/s | 4.2° | 0.19 m |
| Seeds 1–6 | 0.45 – 0.78 m/s | 3.0 – 5.3° | 0.21 – 0.52 m |

All 7 runs met the safe-landing criterion (speed < 1 m/s, tilt < 10°, error < 2 m).

## Firmware

`tvc_flight_controller.ino` runs ascent attitude control on a commercial certified solid motor and deploys a parachute with a servo-released hatch. It has an arm switch, pad calibration, a launch/burnout detector, tilt and time abort limits, and SD logging. It has no igniter or pyro outputs.

Status: compiled for syntax only. It has not been run on hardware. Before any flight: bench test, hardware-in-the-loop or tilt-table test, static stand test, and range-safety approval.

## Repository Structure

```text
Mini-Rocket-TVC-Recovery/
├── rocket_6DOF_2Axis_TVC_Recovery.m
├── rocket.m
├── tvc_flight_controller/
│   └── tvc_flight_controller.ino
├── README.md
├── .gitignore
└── figures/
```

## Limitations

- The simulated engine is a throttleable-thrust model. Commercial solid motors cannot throttle, so this flight profile needs a throttleable propulsion system. Propulsion numbers are assumptions.
- 2-axis TVC cannot control roll. Roll is left uncontrolled.
- Attitude comes from gyro integration only (bias calibrated on the pad), so it drifts over time.
- No structural, slosh, or propulsion-transient models.
- Not flight-qualified software.

## Safety / Scope

This repository is for simulation and academic study. It does not provide instructions for building, modifying, or operating propulsion systems. Any hardware use must follow your range safety officer and local regulations.

## Author

**Sewon Zion S.**
Aeronautical Engineering | GNC | Rocket Propulsion | Flight Dynamics | Simulation
