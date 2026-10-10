"""6-DOF mini rocket, 2-axis TVC: F-class motor ascent -> coast -> apogee detection -> parachute recovery.

    python mini_rocket_6dof_tvc.py              run + print results + save plots
    python mini_rocket_6dof_tvc.py --noplot
    python mini_rocket_6dof_tvc.py --mc 20      Monte Carlo (wind, mass, Cd, noise)

API: out = simulate(seed=7, windBase=[3, 0, 0])
World: x east, y north, z up. Body x = nose/thrust axis. Quaternion scalar-first, body -> world.
State (16): pos(3) vel(3) quat(4) rate(3) mass pitchGimbal yawGimbal.

Model: RK4 at 200 Hz, flight software at 100 Hz. Exponential atmosphere, drag, normal force with CP/CG
moment, pitch/yaw/roll damping, wind + gusts, motor thrust curve (impulse and peak fixed by params),
mass/CG/inertia change with burn, thrust misalignment, TVC servo (lag + rate limit + angle limit).
Flight software: pad gyro-bias calibration, gyro-integrated quaternion estimator, baro/accelerometer
alpha-beta filter, GPS position/velocity update at 5 Hz, quaternion attitude error, PID (rate damping on
the measured gyro), TVC mapping, apogee detection, parachute command.

Limitations: the motor thrust curve is a representative shape (not a measured motor); the parachute is a
drag force at the CG with heavy rotational damping (no riser/pendulum dynamics); no fins model beyond a
fixed normal-force slope; TVC cannot control roll; not flight-qualified.
"""
import argparse
import math
from types import SimpleNamespace

import numpy as np

D2R = math.pi / 180.0


def build_motor(P):
    t = np.array([0.0, 0.04, 0.12, 0.30, 0.90, 1.40, 1.62, P.tBurn])
    base = np.array([0.0, 1.0, 0.88, 0.74, 0.66, 0.62, 0.30, 0.0])

    def curve(k):
        T = base.copy()
        T[2:-1] *= k
        T[1] = 1.0
        return T * P.Tpeak

    i0 = np.trapezoid(curve(0.0), t) if hasattr(np, "trapezoid") else np.trapz(curve(0.0), t)
    i1 = np.trapezoid(curve(1.0), t) if hasattr(np, "trapezoid") else np.trapz(curve(1.0), t)
    k = (P.Itot - i0) / (i1 - i0)
    P.cT, P.cTh = t, curve(k)


def params(**opts):
    d = D2R
    P = SimpleNamespace(
        g=9.80665, g0=9.80665, rho0=1.225, H=8500.0,
        m0=0.410, L=0.70, D=0.06, Cd=0.60, CNa=3.0, Clp=0.5, Cmq=10.0,
        cgWet=0.400, cgDry=0.395, cp=0.47,
        Iwet=np.array([0.00020, 0.0175, 0.0175]), Idry=np.array([0.00019, 0.0170, 0.0170]),
        Itot=40.0, tBurn=1.75, Tpeak=34.0, Isp=180.0,
        gMax=6 * d, tauServo=0.03, servoRate=120 * d, misalign=np.array([0.3, -0.2]) * d,
        chuteCd=1.5, chuteD=0.41, tOpen=0.4, tDeployDelay=0.25, rotRate=8.0,
        windBase=np.array([1.5, -1.0, 0.0]), gustAmp=np.array([1.2, 0.8, 0.3]), gustW=np.array([0.9, 1.3, 0.7]),
        gyroSigma=0.12 * d, gyroBias=np.array([0.35, -0.25, 0.15]) * d,
        accSigma=0.05, baroSigma=0.35, gpsPosSigma=0.7, gpsVelSigma=0.15,
        kBaro1=5.0, kBaro2=6.25, kGps1=2.4, kGps2=1.44,
        dt=0.005, ctrlHz=100, tEnd=150.0, gpsEvery=20, tPad=1.0, tilt0=0.3 * d,
        wn=10.0, zeta=0.8, Ki=15.0, intMax=0.1,
    )
    for k, v in opts.items():
        if not hasattr(P, k):
            raise KeyError(f"Unknown option: {k}")
        setattr(P, k, np.array(v, dtype=float) if isinstance(getattr(P, k), np.ndarray) else v)
    P.Aref = math.pi * (P.D / 2) ** 2
    P.chuteCdA = P.chuteCd * math.pi * (P.chuteD / 2) ** 2
    P.Kp = P.wn ** 2
    P.Kd = 2 * P.zeta * P.wn
    P.mDry = P.m0 - P.Itot / (P.Isp * P.g0)
    build_motor(P)
    return P


def thrust_at(t, P):
    tau = t - P.tPad
    if tau < 0.0 or tau > P.tBurn:
        return 0.0
    return float(np.interp(tau, P.cT, P.cTh))


def cross3(a, b):
    return np.array([a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]])


def quat2R(q):
    q = q / np.linalg.norm(q)
    a, b, c, d = q
    return np.array([[1 - 2 * (c * c + d * d), 2 * (b * c - a * d), 2 * (b * d + a * c)],
                     [2 * (b * c + a * d), 1 - 2 * (b * b + d * d), 2 * (c * d - a * b)],
                     [2 * (b * d - a * c), 2 * (c * d + a * b), 1 - 2 * (b * b + c * c)]])


def quat_mul(a, b):
    q = np.empty(4)
    q[0] = a[0] * b[0] - a[1:] @ b[1:]
    q[1:] = a[0] * b[1:] + b[0] * a[1:] + cross3(a[1:], b[1:])
    return q / np.linalg.norm(q)


def quat_integrate(q, w, dt):
    n = np.linalg.norm(w)
    th = n * dt
    if th < 1e-12:
        return q
    return quat_mul(q, np.concatenate(([math.cos(th / 2)], math.sin(th / 2) * w / n)))


def q_from_two_vec(a, b):
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    c = float(a @ b)
    if c < -1 + 1e-9:
        ax = cross3(a, np.array([0.0, 0.0, 1.0]))
        if np.linalg.norm(ax) < 1e-6:
            ax = cross3(a, np.array([0.0, 1.0, 0.0]))
        return np.concatenate(([0.0], ax / np.linalg.norm(ax)))
    q = np.concatenate(([1 + c], cross3(a, b)))
    return q / np.linalg.norm(q)


def veh_props(m, P):
    b = min(max((P.m0 - m) / (P.m0 - P.mDry), 0.0), 1.0)
    return P.cgWet + b * (P.cgDry - P.cgWet), P.Iwet + b * (P.Idry - P.Iwet)


def chute_frac(t, ctx, P):
    if not ctx.deployed:
        return 0.0
    return min(max((t - ctx.tDep) / P.tOpen, 0.0), 1.0) ** 2


def dyn(t, X, cmd, P, ctx):
    v, q, w, m = X[3:6], X[6:10], X[10:13], X[13]
    dP, dY = X[14], X[15]
    R = quat2R(q)
    cg, I = veh_props(m, P)
    T = thrust_at(t, P) if m > P.mDry + 1e-9 else 0.0
    wind = P.windBase + P.gustAmp * np.sin(P.gustW * t)
    vrel = v - wind
    V = float(np.linalg.norm(vrel))
    rho = P.rho0 * math.exp(-max(X[2], 0.0) / P.H)
    qd = 0.5 * rho * V * V
    vb = R.T @ vrel
    ub = vb / V if V > 1e-3 else np.zeros(3)
    Fd = -qd * P.Cd * P.Aref * ub
    Fn = -qd * P.Aref * P.CNa * np.array([0.0, ub[1], ub[2]])
    Vs = max(V, 1.0)
    k = qd * P.Aref * P.D * (P.D / (2 * Vs))
    Mdamp = k * np.array([-P.Clp * w[0], -P.Cmq * w[1], -P.Cmq * w[2]])
    Maero = cross3(np.array([cg - P.cp, 0.0, 0.0]), Fn) + Mdamp

    dPa, dYa = dP + P.misalign[0], dY + P.misalign[1]
    tb = T * np.array([math.cos(dPa) * math.cos(dYa), math.sin(dYa), math.sin(dPa) * math.cos(dYa)])
    Mthr = cross3(np.array([-(P.L - cg), 0.0, 0.0]), tb)

    fr = chute_frac(t, ctx, P)
    Fc = -0.5 * rho * P.chuteCdA * fr * V * vrel
    Mc = -P.rotRate * fr * I * w
    Fb = tb + Fd + Fn
    a = (R @ Fb + Fc) / m + np.array([0.0, 0.0, -P.g])
    fb = (Fb + R.T @ Fc) / m
    wd = (Maero + Mthr + Mc - cross3(w, I * w)) / I
    Om = np.array([[0, -w[0], -w[1], -w[2]], [w[0], 0, w[2], -w[1]], [w[1], -w[2], 0, w[0]], [w[2], w[1], -w[0], 0]])
    dX = np.empty(16)
    dX[0:3] = v
    dX[3:6] = a
    dX[6:10] = 0.5 * Om @ q
    dX[10:13] = wd
    dX[13] = -T / (P.Isp * P.g0) if (T > 0 and m > P.mDry + 1e-9) else 0.0
    dX[14] = min(max((cmd[1] - dP) / P.tauServo, -P.servoRate), P.servoRate)
    dX[15] = min(max((cmd[2] - dY) / P.tauServo, -P.servoRate), P.servoRate)
    return dX, fb


def sensor_sample(t, X, tick, on_pad, P, rng, ctx):
    _, fb = dyn(t, X, np.zeros(4), P, ctx)
    if on_pad:
        R = quat2R(X[6:10])
        fw = R @ fb
        fb = fb + R.T @ np.array([0.0, 0.0, max(P.g - fw[2], 0.0)])
    S = {"gyro": X[10:13] + P.gyroBias + P.gyroSigma * rng.standard_normal(3),
         "acc": fb + P.accSigma * rng.standard_normal(3),
         "baro": X[2] + P.baroSigma * rng.standard_normal(),
         "gpsNew": tick % P.gpsEvery == 0,
         "gpsPos": X[0:2] + P.gpsPosSigma * rng.standard_normal(2),
         "gpsVel": X[3:5] + P.gpsVelSigma * rng.standard_normal(2)}
    return S


def init_controller(P):
    C = SimpleNamespace(phase=0, padN=0, padGyro=np.zeros(3), padAcc=np.zeros(3), bE=np.zeros(3),
                        qE=np.array([1.0, 0, 0, 0]), pE=np.zeros(3), vE=np.zeros(3), intY=0.0, intZ=0.0,
                        mE=P.m0, eB=np.zeros(3), downCount=0, tDep=-1.0, tDetect=-1.0, hMax=0.0)
    return C


def flight_software(t, S, C, P):
    dtc = 1.0 / P.ctrlHz
    cmd = np.zeros(4)
    if C.phase == 0:
        C.padN += 1
        C.padGyro += S["gyro"]
        C.padAcc += S["acc"]
        if t >= P.tPad - dtc:
            C.bE = C.padGyro / C.padN
            fm = C.padAcc / C.padN
            fm = fm / np.linalg.norm(fm)
            q_up = np.array([math.cos(-math.pi / 4), 0.0, math.sin(-math.pi / 4), 0.0])
            C.qE = quat_mul(q_up, q_from_two_vec(fm, np.array([1.0, 0.0, 0.0])))
            C.phase = 1
        return C, cmd

    wE = S["gyro"] - C.bE
    C.qE = quat_integrate(C.qE, wE, dtc)
    RE = quat2R(C.qE)
    aW = RE @ S["acc"] - np.array([0.0, 0.0, P.g])
    C.pE = C.pE + C.vE * dtc + 0.5 * aW * dtc ** 2
    C.vE = C.vE + aW * dtc
    r = S["baro"] - C.pE[2]
    C.pE[2] += P.kBaro1 * dtc * r
    C.vE[2] += P.kBaro2 * dtc * r
    if S["gpsNew"]:
        Tg = P.gpsEvery * dtc
        rg = S["gpsPos"] - C.pE[0:2]
        C.pE[0:2] += P.kGps1 * Tg * rg
        C.vE[0:2] += P.kGps2 * Tg * rg + 0.3 * (S["gpsVel"] - C.vE[0:2])
    Tk = thrust_at(t, P)
    C.mE = max(C.mE - Tk / (P.Isp * P.g0) * dtc, P.mDry)
    cgE, IE = veh_props(C.mE, P)
    tb = t - P.tPad

    if C.phase == 1 and tb >= P.tBurn:
        C.phase = 2
    if C.phase == 2:
        C.hMax = max(C.hMax, C.pE[2])
        C.downCount = C.downCount + 1 if (C.vE[2] < -0.3 and tb > P.tBurn + 1.0) else 0
        if C.downCount >= 5 and C.tDep < 0:
            C.tDetect = t
            C.tDep = t + P.tDeployDelay
    if C.tDep >= 0 and t >= C.tDep:
        cmd[3] = 1.0
        C.phase = 3

    if C.phase == 1:
        aNose = RE[:, 0]
        dDes = np.array([0.0, 0.0, 1.0])
        qe = q_from_two_vec(aNose, dDes)
        eB = RE.T @ (2 * qe[1:4])
        C.eB = eB
        intY = min(max(C.intY + eB[1] * dtc, -P.intMax), P.intMax)
        intZ = min(max(C.intZ + eB[2] * dtc, -P.intMax), P.intMax)
        alY = P.Kp * eB[1] + P.Ki * intY - P.Kd * wE[1]
        alZ = P.Kp * eB[2] + P.Ki * intZ - P.Kd * wE[2]
        arm = P.L - cgE
        Teff = max(Tk, 5.0)
        dPraw = IE[1] * alY / (arm * Teff)
        dYraw = -IE[2] * alZ / (arm * Teff)
        if abs(dPraw) < P.gMax:
            C.intY = intY
        if abs(dYraw) < P.gMax:
            C.intZ = intZ
        cmd[1] = min(max(dPraw, -P.gMax), P.gMax)
        cmd[2] = min(max(dYraw, -P.gMax), P.gMax)
        cmd[0] = Tk
    return C, cmd


def simulate(seed=7, plots=False, verbose=True, **opts):
    P = params(**opts)
    rng = np.random.default_rng(seed)
    ctx = SimpleNamespace(deployed=False, tDep=0.0)
    dt = P.dt
    ctrl_every = round(1 / (P.ctrlHz * dt))
    n_steps = round(P.tEnd / dt)

    q_up = np.array([math.cos(-math.pi / 4), 0.0, math.sin(-math.pi / 4), 0.0])
    q0 = quat_mul(q_up, np.array([math.cos(P.tilt0 / 2), 0.0, math.sin(P.tilt0 / 2), 0.0]))
    X = np.concatenate((np.zeros(3), np.zeros(3), q0, np.zeros(3), [P.m0, 0.0, 0.0]))
    C = init_controller(P)
    cmd = np.zeros(4)
    logs = {k: [] for k in ("t", "X", "qE", "pE", "vE", "phase", "cmd", "eB", "T")}
    airborne, status, t_end = False, "TIMEOUT", P.tEnd

    for k in range(n_steps):
        t = k * dt
        on_pad = (not airborne) and X[2] <= 0.0
        if k % ctrl_every == 0:
            S = sensor_sample(t, X, k // ctrl_every, on_pad, P, rng, ctx)
            C, cmd = flight_software(t, S, C, P)
            if cmd[3] > 0.5 and not ctx.deployed:
                ctx.deployed, ctx.tDep = True, t
            logs["t"].append(t)
            logs["X"].append(X.copy())
            logs["qE"].append(C.qE.copy())
            logs["pE"].append(C.pE.copy())
            logs["vE"].append(C.vE.copy())
            logs["phase"].append(C.phase)
            logs["cmd"].append(cmd.copy())
            logs["eB"].append(C.eB.copy())
            logs["T"].append(thrust_at(t, P))
        Xp = X
        f1, _ = dyn(t, X, cmd, P, ctx)
        f2, _ = dyn(t + dt / 2, X + dt * f1 / 2, cmd, P, ctx)
        f3, _ = dyn(t + dt / 2, X + dt * f2 / 2, cmd, P, ctx)
        f4, _ = dyn(t + dt, X + dt * f3, cmd, P, ctx)
        X = X + dt * (f1 + 2 * f2 + 2 * f3 + f4) / 6
        X[6:10] /= np.linalg.norm(X[6:10])
        X[13] = max(X[13], P.mDry)
        X[14:16] = np.clip(X[14:16], -P.gMax, P.gMax)
        if X[2] > 0.3:
            airborne = True
        if X[2] <= 0:
            if airborne:
                a = Xp[2] / (Xp[2] - X[2])
                X = Xp + a * (X - Xp)
                status, t_end = "TOUCHDOWN", t + a * dt
                break
            X[2] = 0.0
            X[3:6] = np.array([0.0, 0.0, max(X[5], 0.0)])
            X[10:13] = 0.0

    L = {k: np.array(v) for k, v in logs.items()}
    Xs = L["X"]
    spd = np.linalg.norm(Xs[:, 3:6], axis=1)
    i_ap = int(Xs[:, 2].argmax())
    i_bo = int(np.searchsorted(L["t"], P.tPad + P.tBurn))
    tilt = np.degrees(np.arccos(np.clip([quat2R(q)[2, 0] for q in Xs[:, 6:10]], -1, 1)))
    burn = (L["t"] >= P.tPad) & (L["t"] <= P.tPad + P.tBurn)
    out = dict(status=status, tFlight=t_end)
    out["apogee"] = float(Xs[i_ap, 2])
    out["tApogee"] = float(L["t"][i_ap] - P.tPad)
    out["peakSpeed"] = float(spd.max())
    out["maxThrust"] = float(L["T"].max())
    out["totalImpulse"] = float(np.trapezoid(L["T"], L["t"]) if hasattr(np, "trapezoid") else np.trapz(L["T"], L["t"]))
    out["burnoutAlt"] = float(Xs[i_bo, 2])
    out["burnoutSpeed"] = float(spd[i_bo])
    out["maxGimbalCmdDeg"] = float(np.degrees(np.abs(L["cmd"][burn][:, 1:3]).max()))
    out["maxGimbalActDeg"] = float(np.degrees(np.abs(Xs[burn][:, 14:16]).max()))
    out["maxTiltBurnDeg"] = float(tilt[burn].max())
    out["tiltAtBurnoutDeg"] = float(tilt[i_bo])
    out["descentRate"] = float(-Xs[-1, 5])
    out["touchSpeed"] = float(np.linalg.norm(Xs[-1, 3:6]))
    out["downrange"] = float(np.linalg.norm(Xs[-1, 0:2]))
    out["landing"] = Xs[-1, 0:2].copy()
    out["tDetect"] = float(C.tDetect - P.tPad) if C.tDetect >= 0 else float("nan")
    out["tDeploy"] = float(ctx.tDep - P.tPad) if ctx.deployed else float("nan")
    out["apogeeDetectLagS"] = out["tDetect"] - out["tApogee"]
    out["deployAlt"] = float(Xs[np.searchsorted(L["t"], ctx.tDep), 2]) if ctx.deployed else float("nan")
    out["estApogee"] = float(C.hMax)
    out["propUsed"] = float(P.m0 - Xs[i_bo, 13])
    out["log"], out["params"] = L, P
    if verbose:
        print(f"Mini rocket 6-DOF 2-axis TVC simulation: {status}")
        print(f"  Motor               : {out['totalImpulse']:.1f} N.s, burn {P.tBurn:.2f} s, peak {out['maxThrust']:.1f} N")
        print(f"  Burnout             : {out['burnoutAlt']:.1f} m at {out['burnoutSpeed']:.1f} m/s")
        print(f"  Apogee              : {out['apogee']:.1f} m at T+{out['tApogee']:.1f} s (estimated {out['estApogee']:.1f} m)")
        print(f"  Peak velocity       : {out['peakSpeed']:.1f} m/s")
        print(f"  TVC (burn)          : max command {out['maxGimbalCmdDeg']:.2f} deg, max tilt {out['maxTiltBurnDeg']:.1f} deg")
        print(f"  Apogee detect/chute : detect T+{out['tDetect']:.1f} s ({out['apogeeDetectLagS']:.2f} s after apogee), deploy at {out['deployAlt']:.0f} m")
        print(f"  Descent rate        : {out['descentRate']:.2f} m/s at touchdown")
        print(f"  Landing             : {out['downrange']:.0f} m downrange ({out['landing'][0]:.0f} E, {out['landing'][1]:.0f} N)")
        print(f"  Flight time         : {out['tFlight'] - P.tPad:.1f} s after ignition")
    if plots:
        make_plots(out)
    return out


def make_plots(out, path="."):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    L, P = out["log"], out["params"]
    t = L["t"] - P.tPad
    X = L["X"]
    r2d = 180 / math.pi
    plt.rcParams.update({"font.size": 7, "axes.titlesize": 8.5, "axes.titleweight": "bold", "axes.grid": True,
                         "grid.alpha": 0.25, "axes.spines.top": False, "axes.spines.right": False})
    fig, ax = plt.subplots(3, 3, figsize=(11, 8.5))
    fig.suptitle("Mini Rocket - 6-DOF Simulation with 2-Axis TVC", fontsize=13, fontweight="bold")
    a = ax[0, 0]
    a.plot(t, X[:, 2], lw=1.4)
    a.plot(t, L["pE"][:, 2], "--", lw=1.0)
    a.set(title=f"Altitude (apogee {out['apogee']:.0f} m)", xlabel="t - ignition (s)", ylabel="m")
    a = ax[0, 1]
    a.plot(t, X[:, 5], lw=1.3, label="vertical")
    a.plot(t, np.hypot(X[:, 3], X[:, 4]), lw=1.3, label="horizontal")
    a.set(title=f"Velocity (peak {out['peakSpeed']:.0f} m/s)", xlabel="t (s)", ylabel="m/s")
    a.legend()
    a = ax[0, 2]
    a.plot(X[:, 0], X[:, 1], lw=1.3)
    a.plot(0, 0, "g^")
    a.plot(*out["landing"], "rx")
    a.set(title=f"Ground track ({out['downrange']:.0f} m downrange)", xlabel="east (m)", ylabel="north (m)", aspect="equal")
    w = (t >= -0.1) & (t <= P.tBurn + 0.3)
    a = ax[1, 0]
    a.plot(t[w], L["T"][w], lw=1.4)
    a.set(title=f"Motor thrust ({out['totalImpulse']:.0f} N.s, peak {out['maxThrust']:.0f} N)", xlabel="t (s)", ylabel="N")
    a = ax[1, 1]
    a.plot(t[w], L["cmd"][w, 1] * r2d, "--", lw=1, label="pitch cmd")
    a.plot(t[w], X[w, 14] * r2d, lw=1.2, label="pitch")
    a.plot(t[w], L["cmd"][w, 2] * r2d, "--", lw=1, label="yaw cmd")
    a.plot(t[w], X[w, 15] * r2d, lw=1.2, label="yaw")
    a.axhline(6, color="k", ls=":")
    a.axhline(-6, color="k", ls=":")
    a.set(title="TVC gimbal (limit +/-6 deg)", xlabel="t (s)", ylabel="deg")
    a.legend(ncol=2)
    tilt = np.degrees(np.arccos(np.clip([quat2R(q)[2, 0] for q in X[:, 6:10]], -1, 1)))
    a = ax[1, 2]
    a.plot(t[w], tilt[w], lw=1.3)
    a.set(title="Tilt from vertical during burn", xlabel="t (s)", ylabel="deg")
    a = ax[2, 0]
    a.plot(t[w], L["eB"][w, 1] * r2d, lw=1, label="pitch")
    a.plot(t[w], L["eB"][w, 2] * r2d, lw=1, label="yaw")
    a.set(title="Quaternion attitude error", xlabel="t (s)", ylabel="deg")
    a.legend()
    a = ax[2, 1]
    a.plot(t, X[:, 13], lw=1.3)
    a.set(title="Mass (propellant burn)", xlabel="t (s)", ylabel="kg")
    a = ax[2, 2]
    a.plot(t, np.linalg.norm(L["pE"] - X[:, 0:3], axis=1), lw=1.2)
    a.set(title="Estimator position error", xlabel="t (s)", ylabel="m")
    fig.text(0.5, 0.005, "Simulation study - rebuilt Python model; not flight-certified", ha="center", fontsize=6, color="#777")
    fig.tight_layout(rect=(0, 0.015, 1, 0.96))
    fig.savefig(f"{path}/mini_rocket_dashboard.png", dpi=250, facecolor="white")
    plt.close(fig)
    ax3 = plt.figure(figsize=(7, 6)).add_subplot(projection="3d")
    ax3.plot(X[:, 0], X[:, 1], X[:, 2], lw=1.5)
    ax3.set(xlabel="east (m)", ylabel="north (m)", zlabel="altitude (m)", title="Mini rocket trajectory")
    plt.gcf().savefig(f"{path}/mini_rocket_trajectory3d.png", dpi=250, facecolor="white")
    plt.close("all")


def monte_carlo(n=20):
    keys = ("apogee", "peakSpeed", "descentRate", "downrange", "maxGimbalCmdDeg", "maxTiltBurnDeg")
    rows = []
    for s in range(n):
        rng = np.random.default_rng(900 + s)
        o = simulate(seed=s, verbose=False, windBase=np.array([1.5, -1.0, 0.0]) * rng.uniform(0.3, 2.0),
                     m0=0.410 * rng.uniform(0.95, 1.05), Cd=0.60 * rng.uniform(0.85, 1.15))
        rows.append([o[k] for k in keys])
    a = np.array(rows)
    print(f"{'metric':18s} {'mean':>8s} {'std':>8s} {'min':>8s} {'max':>8s}")
    for k, c in zip(keys, a.T):
        print(f"{k:18s} {c.mean():8.2f} {c.std():8.2f} {c.min():8.2f} {c.max():8.2f}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--noplot", action="store_true")
    ap.add_argument("--mc", type=int, default=0)
    args = ap.parse_args()
    if args.mc:
        monte_carlo(args.mc)
    else:
        simulate(plots=not args.noplot)
