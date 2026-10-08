# Mini Rocket — 6-DOF TVC & Recovery Simulation

This repository contains a mini-rocket simulation set built in MATLAB. The primary simulation in the active development code is a six-degree-of-freedom model with two-axis thrust-vector control and recovery logic. The legacy `rocket.m` script remains as a pitch-plane baseline for simpler analysis and comparison.

## Project overview

The main simulation models a small rocket from pad conditions through powered ascent, hover, lateral diversion, controlled powered descent, and touchdown. The dynamics are integrated in the time domain with rigid-body translational and rotational equations, a quaternion attitude state, changing mass properties, and a gimbaled thrust vector that is commanded by a flight-software loop.

The active 6-DOF model is structured around the following state vector:

```text
[x, y, z, vx, vy, vz, q0, q1, q2, q3, wx, wy, wz, m, gimbalPitch, gimbalYaw, thrust]
```

with:

- `x, y, z` — inertial position
- `vx, vy, vz` — body-frame/world-frame velocity components as used by the model
- `q0..q3` — quaternion attitude, scalar-first
- `wx, wy, wz` — body angular rates
- `m` — vehicle mass
- `gimbalPitch, gimbalYaw` — TVC gimbal angles
- `thrust` — engine thrust magnitude

The quaternion is normalized at each RK4 step, and the rotational dynamics are propagated using the body inertia tensor updated with burn progress.

## Main simulation files

The active simulation used by the development workflow is the 6-DOF script:

```matlab
out = rocket_6DOF_2Axis_TVC_Recovery;
```

and the no-plot form:

```matlab
out = rocket_6DOF_2Axis_TVC_Recovery(false);
out = rocket_6DOF_2Axis_TVC_Recovery(false,12);
```

The repository also retains the simpler pitch-plane baseline script:

```matlab
rocket
```

The baseline script is a 2D pitch-plane model with state:

```text
[x z vx vz theta q delta mass]
```

and it uses fixed-step integration.

## Baseline pitch-plane script: `rocket.m`

`rocket.m` implements a simplified pitch-plane rocket simulation with a single TVC channel in the longitudinal plane. It is not the primary 6-DOF model.

The code defines the baseline vehicle and propulsion parameters as:

- `m0 = 0.422 kg`
- `mDry = 0.320 kg`
- `Tmax = 18.0 N`
- `Isp = 70 s`
- `g0 = 9.80665 m/s^2`
- `L = 0.24 m`
- `Iyy = 0.018 kg·m^2`
- `tauTVC = 0.08 s`
- `deltaMax = ±8 deg`
- `dt = 0.01 s`
- `tEnd = 65 s`

The script exercises the following logic:

- thrust is scheduled in phases and turned off when mass is at or below dry mass
- target altitude commands are updated with time-based phases
- vertical velocity command is limited by phase
- attitude command is computed from lateral/altitude tracking and inversion of the vehicle states
- a TVC servo dynamics term is applied as a first-order lag
- acceleration and angular acceleration are computed from the gimbaled thrust vector
- mass flow rate is `mdot = -T / (Isp * g0)` while thrust is active
- the state is integrated with explicit stepping in time
- touchdown-like logic clamps the altitude and vertical velocity at zero when the rocket reaches the ground

The output figures are:

- altitude history
- thrust profile
- TVC angle history
- vertical velocity
- trajectory in the x-z plane
- pitch attitude history

## 6-DOF main simulation: `rocket_6DOF_2Axis_TVC_Recovery.m`

The 6-DOF script is the model described in the file comments and the implementation itself. The code is written as a single MATLAB function with optional plotting and optional noise-seed control. The core behaviors implemented in the script are:

### Dynamics and kinematics

- 17-state rigid-body model: position, velocity, quaternion, body rates, mass, gimbal angles, and thrust
- inertial frame is world x east, y north, z up; body x points along the nose/thrust axis
- quaternion is scalar-first and rotates body-to-world
- RK4 integration is used with `dt = 0.0025 s`
- the time step is fixed and the controller runs at `100 Hz`

### Propulsion and mass model

- `P.m0 = 0.90 kg`
- `P.mDry = 0.62 kg`
- `P.Isp = 120 s`
- `P.Tmax = 22 N`
- `P.Tmin = 4.4 N`
- `P.tauT = 0.08 s`
- engine is treated as a throttleable-thrust abstraction in the simulation
- the model applies propellant burn by reducing mass and updating the mass properties
- moving CG and inertia are updated through `vehProps(m,P)`
- the dry and wet inertia matrices are defined explicitly and interpolated as the mass burns

### Aerodynamics and atmosphere

- exponential atmosphere model: `rho = rho0 * exp(-max(z,0)/H)` with `rho0 = 1.225 kg/m^3`, `H = 8500 m`
- drag force acts opposite the relative velocity vector
- normal-force term is added using `P.CNa` and a reference area `Aref = pi*(D/2)^2`
- aerodynamic moment is computed from the CP/CG offset and rotational rate influence
- a base wind and gust model are used in the dynamic equations
- roll is not actively controlled by the TVC system; the code comments explicitly note that roll is left uncontrolled

### TVC and actuator models

- 2-axis TVC: pitch and yaw gimbal deflection are treated as separate states
- the gimbal commands are mapped through `dPdot` and `dYdot` with a servo time constant and rate limits
- `P.tauServo = 0.05 s`
- `P.servoRate = 90 deg/s`
- `P.gMax = 7 deg`
- `P.misalign = [0.3; -0.2] deg` is included as thrust misalignment
- engine throttle dynamics are modeled via a first-order lag `dTdot = (cmd(1)-T)/P.tauT`

### Flight software and control law

The flight software loop runs at 100 Hz and performs:

- pad calibration of gyro bias and alignment using accelerometer readings
- gyro-integrated quaternion propagation
- state estimation through barometer and GPS-like position/velocity updates
- phase-state machine for pad calibration, ascent, hover, lateral divert, and controlled descent
- guidance to command target altitude, lateral displacement, and descent rate
- quaternion attitude error generation using the current nose axis and desired thrust axis
- angular-acceleration PID on measured body rates with an anti-windup clamp
- TVC mapping from body-axis angular error to pitch and yaw gimbal commands

The control parameters in the model include:

- `P.wn = 6`, `P.zeta = 0.8`
- `P.Kp = P.wn^2`
- `P.Kd = 2 * P.zeta * P.wn`
- `P.Ki = 8`
- `P.intMax = 0.15`

The guidance parameters include:

- `P.tPad = 1.0 s`
- `P.hHover = 30 m`
- `P.tHover = 1.5 s`
- `P.target = [12; 6] m`
- `P.Kh = 0.8`
- `P.vzUp = 6 m/s`
- `P.Kvz = 2.5`
- `P.Kpos = 0.5`
- `P.vLatMax = 3 m/s`
- `P.Kvlat = 1.2`
- `P.aLatMax = 2 m/s^2`
- `P.tiltMax = 15 deg`
- `P.aDesc = 0.5 m/s^2`
- `P.vDescMax = 4 m/s`
- `P.vTouch = 0.30 m/s`

### Sensors and noise

The code includes sensor models for:

- gyro noise and bias: `P.gyroSigma = 0.12 deg`, `P.gyroBias = [0.35; -0.25; 0.15] deg`
- accelerometer noise: `P.accSigma = 0.05`
- barometer noise: `P.baroSigma = 0.35`
- GPS-like position noise: `P.gpsPosSigma = 0.7 m`
- GPS-like velocity noise: `P.gpsVelSigma = 0.15 m/s`

### Mission and recovery phase logic

The phase logic in the controller is explicit and implemented in the code:

- phase 0: pad calibration
- phase 1: powered ascent
- phase 2: hover
- phase 3: lateral divert
- phase 4: controlled descent

The script also implements landing checks and safe-landing evaluation based on:

- touchdown speed
- touchdown vertical speed
- touchdown tilt angle
- landing position error
- maximum tilt and gimbal deflection
- propellant used and remaining mass

## Numerical method and integration

The 6-DOF simulation uses fourth-order Runge-Kutta integration, as called explicitly in the main loop:

```matlab
f1 = dyn(t,X,cmd,P);
f2 = dyn(t+dt/2,X+dt*f1/2,cmd,P);
f3 = dyn(t+dt/2,X+dt*f2/2,cmd,P);
f4 = dyn(t+dt,X+dt*f3,cmd,P);
X = X + dt*(f1+2*f2+2*f3+f4)/6;
```

This is the integration method used in the active 6-DOF script. The baseline `rocket.m` uses an explicit time-stepping approach without RK4.

## Figures

The repository contains the following generated SVG figures in `figures/`:

```text
figures/
├── mini_rocket_3d_recovery_path.svg
├── mini_rocket_altitude.svg
├── mini_rocket_tvc_command.svg
├── mini_rocket_vertical_velocity.svg
├── README.md
```

These are the current figures in the repository and should be referenced as generated outputs for the baseline and simulation plots. The figure set is organized as a simple gallery.

## Repository structure

```text
Mini-Rocket-TVC-Recovery/
├── .gitignore
├── README.md
├── rocket.m
├── figures/
│   ├── README.md
│   ├── mini_rocket_3d_recovery_path.svg
│   ├── mini_rocket_altitude.svg
│   ├── mini_rocket_tvc_command.svg
│   └── mini_rocket_vertical_velocity.svg
└── (6-DOF script under active development, not yet checked into this tree)
```

The checked-in repository tree currently contains the pitch-plane baseline and the figure gallery. The development 6-DOF script described above is the primary simulation logic for the project, but it is not part of the current tree shown here.

## Engineering topics

- rigid-body dynamics
- quaternion-based attitude propagation
- 2-axis TVC control
- guidance and navigation
- propulsion and mass depletion
- atmospheric flight and drag modeling
- attitude estimation and filtering
- numerical integration
- sensor modeling and filtering
- MATLAB simulation and visualization

## Limitations

The following are intentionally simplified or omitted in the code:

- The active 6-DOF model is not flight-qualified and is not intended for design certification.
- The engine is treated as a throttleable-thrust abstraction; the code comments explicitly note that commercial solid motors are not throttleable.
- Roll is not controlled; the TVC system is two-axis and does not regulate roll.
- Quaternion attitude comes from gyro integration and is only corrected through the estimator and sensor fusion logic; it is not a full inertial navigation solution.
- The model does not include full structural dynamics, slosh dynamics, or propulsion transients.
- The simulation does not include a complete aeroelastic model, exhaust plume interaction model, or high-fidelity thermal model.
- The baseline `rocket.m` script is a pitch-plane model only and does not include full 3D motion, yaw dynamics, or quaternion attitude.
- The code does not include a complete Monte Carlo or hardware-in-the-loop validation setup.

## Safety / scope

This project is intended for simulation and engineering study. It is not a flight-ready design package, and the control logic should not be used as-is for a real launch without independent validation, hardware-in-the-loop testing, and range safety review.

## Author

**Sewon Zion S.**

Aeronautical Engineering

Focus areas: aerospace GNC, rocket propulsion, flight dynamics, simulation, and control systems.
