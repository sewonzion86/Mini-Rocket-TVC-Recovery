function out = rocket_6DOF_2Axis_TVC_Recovery(doPlots,seed)
% ROCKET_6DOF_2AXIS_TVC_RECOVERY
% 6-DOF mini rocket with 2-axis (pitch/yaw) thrust-vector control.
% Mission: pad -> powered ascent -> hover -> lateral divert (secondary powered
% maneuver) -> controlled powered descent and soft touchdown.
%
% Run:  out = rocket_6DOF_2Axis_TVC_Recovery;          (with plots)
%       out = rocket_6DOF_2Axis_TVC_Recovery(false);   (no plots)
%       out = rocket_6DOF_2Axis_TVC_Recovery(false,12); (other noise seed, for Monte Carlo)
%
% MODEL
%   * 17 states: position(3) velocity(3) quaternion(4) body rates(3) mass
%     TVC pitch angle, TVC yaw angle, actual thrust.   Integrator: RK4, dt = 2.5 ms
%   * Frames: world x east, y north, z up. Body x = nose (thrust axis), y, z right-handed.
%     Quaternion is scalar-first and rotates body -> world.
%   * Thrust acts along +x body, deflected by pitch (about y) and yaw (about z) gimbal.
%   * Exponential atmosphere, drag, normal force with CP/CG moment, roll damping,
%     base wind plus gusts.
%   * Propellant burn (Isp), moving CG, changing inertia tensor.
%   * Servo: first-order lag + rate limit + angle limit. Engine: first-order throttle lag.
%   * Sensors: gyro (noise + bias), accelerometer, barometer, GPS-like position/velocity.
%   * Flight software at 100 Hz: gyro-integrated quaternion estimator, alpha-beta
%     position/velocity filter, phase state machine, guidance, quaternion
%     attitude error, PID (rate damping on measured gyro), TVC mapping.
%
% LIMITATIONS (be honest in the report)
%   * Engine is a throttleable-thrust abstraction (Tmin..Tmax). Commercial solid
%     motors are NOT throttleable, so this flight profile needs a throttleable
%     propulsion system. All propulsion numbers are assumptions, not a design.
%   * TVC cannot control roll. Roll is left uncontrolled (low roll rate, no disturbance).
%   * Attitude comes from gyro integration only (bias calibrated on the pad), so it drifts.
%   * Not flight-qualified. No structural, slosh, or propulsion-transient models.

if nargin < 1, doPlots = true; end
if nargin < 2, seed = 7; end
P = params();
if exist('OCTAVE_VERSION','builtin') ~= 0, randn('state',seed); else, rng(seed); end

dt = P.dt;  ctrlEvery = round(1/(P.ctrlHz*dt));
nSteps = round(P.tEnd/dt);
nTick  = ceil(nSteps/ctrlEvery) + 2;

qUp = [cos(-pi/4); 0; sin(-pi/4); 0];            % nose (body x) pointing up
X = [0;0;0; 0;0;0; qUp; 0;0;0; P.m0; 0;0;0];
C = initController(P);
cmd = [0;0;0];

L.t = zeros(1,nTick);  L.X = zeros(17,nTick);  L.qE = zeros(4,nTick);
L.pE = zeros(3,nTick); L.vE = zeros(3,nTick);  L.phase = zeros(1,nTick);
L.eB = zeros(3,nTick); L.cmd = zeros(3,nTick);
nl = 0;  airborne = false;  status = 'TIMEOUT';  t = 0;

for k = 0:nSteps-1
    t = k*dt;
    onPad = ~airborne && X(3) <= 0.0;
    if mod(k,ctrlEvery) == 0
        S = sensorSample(t,X,k/ctrlEvery,onPad,P);
        [C,cmd] = flightSoftware(t,S,C,P);
        nl = nl + 1;
        L.t(nl) = t;  L.X(:,nl) = X;  L.qE(:,nl) = C.qE;  L.pE(:,nl) = C.pE;
        L.vE(:,nl) = C.vE;  L.phase(nl) = C.phase;  L.eB(:,nl) = C.eB;  L.cmd(:,nl) = cmd;
    end
    f1 = dyn(t,X,cmd,P);
    f2 = dyn(t+dt/2,X+dt*f1/2,cmd,P);
    f3 = dyn(t+dt/2,X+dt*f2/2,cmd,P);
    f4 = dyn(t+dt,X+dt*f3,cmd,P);
    X = X + dt*(f1+2*f2+2*f3+f4)/6;
    X(7:10) = X(7:10)/norm(X(7:10));
    X(14) = max(X(14),P.mDry);
    X(17) = max(X(17),0);
    X(15:16) = max(min(X(15:16),P.gMax),-P.gMax);

    if X(3) > 0.3, airborne = true; end
    if X(3) <= 0
        if airborne
            status = 'TOUCHDOWN';
            tEndSim = t + dt;
            break;
        else                                      % resting on the pad
            X(3) = 0;  X(4:6) = [0;0;max(X(6),0)];  X(11:13) = [0;0;0];
        end
    end
    tEndSim = t + dt;
end

nl = min(nl,nTick);
fn = fieldnames(L);
for i = 1:numel(fn), L.(fn{i}) = L.(fn{i})(:,1:nl); end

R = quat2R(X(7:10));
out.status      = status;
out.tFlight     = tEndSim;
out.touchSpeed  = norm(X(4:6));
out.touchVz     = X(6);
out.touchTiltDeg = acos(max(-1,min(1,R(3,1))))*180/pi;
out.landErr     = norm(X(1:2)-P.target);
out.propLeft    = X(14)-P.mDry;
out.propUsed    = P.m0 - X(14);
out.maxHeight   = max(L.X(3,:));
out.maxTiltDeg  = max(acos(max(-1,min(1,squeeze(tiltCos(L.X(7:10,:))))))*180/pi);
out.maxGimbalDeg = max(max(abs(L.X(15:16,:))))*180/pi;
out.safeLanding = strcmp(status,'TOUCHDOWN') && out.touchSpeed < 1.0 && ...
                  out.touchTiltDeg < 10 && out.landErr < 2.0;
out.log = L;
out.params = P;

fprintf('6-DOF 2-axis TVC recovery simulation: %s\n',status);
fprintf('  Flight time        : %.1f s\n',out.tFlight);
fprintf('  Max altitude       : %.1f m\n',out.maxHeight);
fprintf('  Touchdown speed    : %.2f m/s (vertical %.2f)\n',out.touchSpeed,out.touchVz);
fprintf('  Touchdown tilt     : %.1f deg\n',out.touchTiltDeg);
fprintf('  Landing error      : %.2f m from target\n',out.landErr);
fprintf('  Max tilt / gimbal  : %.1f deg / %.2f deg\n',out.maxTiltDeg,out.maxGimbalDeg);
fprintf('  Propellant used    : %.3f kg (left %.3f kg)\n',out.propUsed,out.propLeft);
if out.safeLanding, fprintf('  RESULT             : SAFE LANDING\n');
else, fprintf('  RESULT             : NOT a safe landing\n'); end

if doPlots, makePlots(L,P,out); end
end

% =========================================================================
function P = params()
d = pi/180;
P.g = 9.80665;  P.g0 = 9.80665;  P.rho0 = 1.225;  P.H = 8500;
% vehicle / propulsion (assumed, not a design)
P.m0 = 0.90;  P.mDry = 0.62;  P.Isp = 120;
P.Tmax = 22;  P.Tmin = 4.4;  P.tauT = 0.08;
P.L = 0.70;  P.D = 0.06;  P.Aref = pi*(P.D/2)^2;
P.Cd = 0.55;  P.CNa = 3.0;  P.Clp = 0.5;
P.cgWet = 0.38;  P.cgDry = 0.33;  P.cp = 0.47;            % from nose, m
P.Iwet = diag([0.0012 0.038 0.038]);
P.Idry = diag([0.0009 0.030 0.030]);
% TVC servo
P.tauServo = 0.05;  P.servoRate = 90*d;  P.gMax = 7*d;
P.misalign = [0.3;-0.2]*d;                                  % thrust misalignment
% atmosphere
P.windBase = [1.5;-1.0;0];  P.gustAmp = [1.2;0.8;0.3];  P.gustW = [0.9;1.3;0.7];
% sensors
P.kBaro1 = 5;  P.kBaro2 = 6.25;  P.kGps1 = 2.4;  P.kGps2 = 1.44;      % estimator gains
P.gyroSigma = 0.12*d;  P.gyroBias = [0.35;-0.25;0.15]*d;
P.accSigma = 0.05;  P.baroSigma = 0.35;  P.gpsPosSigma = 0.7;  P.gpsVelSigma = 0.15;
% timing
P.dt = 0.0025;  P.ctrlHz = 100;  P.tEnd = 90;  P.gpsEvery = 20;
% mission / guidance
P.tPad = 1.0;  P.hHover = 30;  P.tHover = 1.5;  P.target = [12;6];
P.Kh = 0.8;  P.vzUp = 6;  P.Kvz = 2.5;
P.Kpos = 0.5;  P.vLatMax = 3;  P.Kvlat = 1.2;  P.aLatMax = 2;
P.tiltMax = 15*d;  P.aDesc = 0.5;  P.vDescMax = 4;  P.vTouch = 0.30;
% attitude loop (angular-acceleration PID)
P.wn = 6;  P.zeta = 0.8;  P.Kp = P.wn^2;  P.Kd = 2*P.zeta*P.wn;  P.Ki = 8;  P.intMax = 0.15;
end

% =========================================================================
function [dX,fb] = dyn(t,X,cmd,P)
v = X(4:6);  q = X(7:10);  w = X(11:13);  m = X(14);
dP = X(15);  dY = X(16);  T = X(17);
R = quat2R(q);
[cg,I] = vehProps(m,P);
fuel = m > P.mDry + 1e-6;
Teff = T*fuel;

wind = P.windBase + P.gustAmp.*sin(P.gustW*t);
vrel = v - wind;
V = norm(vrel);
rho = P.rho0*exp(-max(X(3),0)/P.H);
qd = 0.5*rho*V^2;
vb = R'*vrel;
if V > 1e-3, ub = vb/V; else, ub = [0;0;0]; end
Fd = -qd*P.Cd*P.Aref*ub;
Fn = -qd*P.Aref*P.CNa*[0; ub(2); ub(3)];
Maero = cross([cg-P.cp;0;0],Fn) + [-P.Clp*qd*P.Aref*P.D*(P.D*w(1)/(2*max(V,1)));0;0];

dPa = dP + P.misalign(1);  dYa = dY + P.misalign(2);
tb = Teff*[cos(dPa)*cos(dYa); sin(dYa); sin(dPa)*cos(dYa)];
Mthr = cross([-(P.L-cg);0;0],tb);

Fb = tb + Fd + Fn;
a = R*Fb/m + [0;0;-P.g];
fb = Fb/m;
wd = I\(Maero + Mthr - cross(w,I*w));
Om = [0 -w(1) -w(2) -w(3); w(1) 0 w(3) -w(2); w(2) -w(3) 0 w(1); w(3) w(2) -w(1) 0];
qdot = 0.5*Om*q;
mdot = -Teff/(P.Isp*P.g0);
dPdot = max(min((cmd(2)-dP)/P.tauServo,P.servoRate),-P.servoRate);
dYdot = max(min((cmd(3)-dY)/P.tauServo,P.servoRate),-P.servoRate);
dTdot = (cmd(1)-T)/P.tauT;
dX = [v; a; qdot; wd; mdot; dPdot; dYdot; dTdot];
end

function [cg,I] = vehProps(m,P)
b = min(max((P.m0-m)/(P.m0-P.mDry),0),1);
cg = P.cgWet + b*(P.cgDry-P.cgWet);
I = P.Iwet + b*(P.Idry-P.Iwet);
end

% =========================================================================
function S = sensorSample(t,X,tick,onPad,P)
[~,fb] = dyn(t,X,[0;0;0],P);
if onPad
    R = quat2R(X(7:10));
    fw = R*fb;
    fb = fb + R'*[0;0;max(P.g-fw(3),0)];          % ground reaction felt by the IMU
end
S.gyro = X(11:13) + P.gyroBias + P.gyroSigma*randn(3,1);
S.acc  = fb + P.accSigma*randn(3,1);
S.baro = X(3) + P.baroSigma*randn;
S.gpsNew = (mod(tick,P.gpsEvery) == 0);
S.gpsPos = X(1:2) + P.gpsPosSigma*randn(2,1);
S.gpsVel = X(4:5) + P.gpsVelSigma*randn(2,1);
end

% =========================================================================
function C = initController(P)
C.phase = 0;  C.tPhase = 0;  C.padN = 0;
C.padGyro = zeros(3,1);  C.padAcc = zeros(3,1);  C.bE = zeros(3,1);
C.qE = [1;0;0;0];  C.pE = zeros(3,1);  C.vE = zeros(3,1);
C.intY = 0;  C.intZ = 0;  C.mE = P.m0;  C.Tf = 0;  C.Tprev = 0;
C.eB = zeros(3,1);
end

function [C,cmd] = flightSoftware(t,S,C,P)
dtc = 1/P.ctrlHz;
cmd = [0;0;0];

if C.phase == 0                                        % pad calibration
    C.padN = C.padN + 1;
    C.padGyro = C.padGyro + S.gyro;
    C.padAcc = C.padAcc + S.acc;
    if t >= P.tPad - dtc
        C.bE = C.padGyro/C.padN;
        fm = C.padAcc/C.padN;  fm = fm/norm(fm);
        qUp = [cos(-pi/4); 0; sin(-pi/4); 0];
        C.qE = quatMul(qUp, qFromTwoVec(fm,[1;0;0]));
        C.phase = 1;  C.tPhase = 0;
    end
    return
end

% ---- estimator ---------------------------------------------------------
wE = S.gyro - C.bE;
C.qE = quatIntegrate(C.qE,wE,dtc);
RE = quat2R(C.qE);
aW = RE*S.acc - [0;0;P.g];
C.pE = C.pE + C.vE*dtc + 0.5*aW*dtc^2;
C.vE = C.vE + aW*dtc;
r = S.baro - C.pE(3);                                  % baro: 2nd-order filter, wn = 2.5 rad/s
C.pE(3) = C.pE(3) + P.kBaro1*dtc*r;   C.vE(3) = C.vE(3) + P.kBaro2*dtc*r;
if S.gpsNew                                            % GPS: wn = 1.2 rad/s plus velocity blend
    Tg = P.gpsEvery*dtc;
    rg = S.gpsPos - C.pE(1:2);
    C.pE(1:2) = C.pE(1:2) + P.kGps1*Tg*rg;
    C.vE(1:2) = C.vE(1:2) + P.kGps2*Tg*rg + 0.3*(S.gpsVel - C.vE(1:2));
end
C.Tf = C.Tf + (C.Tprev - C.Tf)*dtc/P.tauT;
C.mE = max(C.mE - C.Tf/(P.Isp*P.g0)*dtc, P.mDry);
[cgE,IE] = vehProps(C.mE,P);

% ---- phase logic -------------------------------------------------------
C.tPhase = C.tPhase + dtc;
hE = C.pE(3);  vE = C.vE;
tgt = [0;0];
switch C.phase
    case 1                                             % powered ascent
        vzCmd = clampv(P.Kh*(P.hHover-hE),-1.5,P.vzUp);
        if hE > P.hHover-1.5 && abs(vE(3)) < 1.0, C.phase = 2; C.tPhase = 0; end
    case 2                                             % hover
        vzCmd = clampv(P.Kh*(P.hHover-hE),-1.5,P.vzUp);
        if C.tPhase > P.tHover, C.phase = 3; C.tPhase = 0; end
    case 3                                             % lateral divert
        tgt = P.target;
        vzCmd = clampv(P.Kh*(P.hHover-hE),-1.5,P.vzUp);
        if (norm(tgt-C.pE(1:2)) < 0.8 && norm(vE(1:2)) < 0.3) || C.tPhase > 25
            C.phase = 4; C.tPhase = 0;
        end
    otherwise                                          % controlled descent
        tgt = P.target;
        vzCmd = -max(P.vTouch, min(P.vDescMax, 0.8*sqrt(2*P.aDesc*max(hE,0))));
end

% ---- guidance: desired thrust vector -----------------------------------
azCmd = clampv(P.Kvz*(vzCmd - vE(3)),-4,8);
vCmd = clampv(P.Kpos*(tgt - C.pE(1:2)),-P.vLatMax,P.vLatMax);
aXY = clampv(P.Kvlat*(vCmd - vE(1:2)),-P.aLatMax,P.aLatMax);
Fw = C.mE*[aXY; P.g + azCmd];
horiz = norm(Fw(1:2));  maxH = Fw(3)*tan(P.tiltMax);
if horiz > maxH && horiz > 0, Fw(1:2) = Fw(1:2)*maxH/horiz; end
Tcmd = max(P.Tmin,min(P.Tmax,norm(Fw)));
dDes = Fw/norm(Fw);

% ---- quaternion attitude error (shortest arc nose -> desired thrust axis)
aNose = RE*[1;0;0];
qe = quatNorm([1 + dot(aNose,dDes); cross(aNose,dDes)]);
if qe(1) < 0, qe = -qe; end
eB = RE'*(2*qe(2:4));
C.eB = eB;

% ---- PID on angular acceleration, derivative on measured rate ---------
intY = max(min(C.intY + eB(2)*dtc, P.intMax),-P.intMax);
intZ = max(min(C.intZ + eB(3)*dtc, P.intMax),-P.intMax);
alY = P.Kp*eB(2) + P.Ki*intY - P.Kd*wE(2);
alZ = P.Kp*eB(3) + P.Ki*intZ - P.Kd*wE(3);
arm = P.L - cgE;
Teff = max(C.Tf,P.Tmin);
dPraw =  IE(2,2)*alY/(arm*Teff);
dYraw = -IE(3,3)*alZ/(arm*Teff);
if abs(dPraw) < P.gMax, C.intY = intY; end             % anti-windup: freeze when saturated
if abs(dYraw) < P.gMax, C.intZ = intZ; end
dPcmd = max(min(dPraw,P.gMax),-P.gMax);
dYcmd = max(min(dYraw,P.gMax),-P.gMax);

cmd = [Tcmd; dPcmd; dYcmd];
C.Tprev = Tcmd;
end

% =========================================================================
function y = clampv(x,lo,hi)
y = max(min(x,hi),lo);
end

function R = quat2R(q)
q = q/norm(q);  a = q(1); b = q(2); c = q(3); d = q(4);
R = [1-2*(c^2+d^2), 2*(b*c-a*d),   2*(b*d+a*c);
     2*(b*c+a*d),   1-2*(b^2+d^2), 2*(c*d-a*b);
     2*(b*d-a*c),   2*(c*d+a*b),   1-2*(b^2+c^2)];
end

function q = quatNorm(q)
q = q/norm(q);
end

function q = quatMul(a,b)
q = [a(1)*b(1)-dot(a(2:4),b(2:4));
     a(1)*b(2:4)+b(1)*a(2:4)+cross(a(2:4),b(2:4))];
q = q/norm(q);
end

function q = quatIntegrate(q,w,dt)
th = norm(w)*dt;
if th < 1e-12, dq = [1;0;0;0]; else, dq = [cos(th/2); sin(th/2)*w/norm(w)]; end
q = quatMul(q,dq);
end

function q = qFromTwoVec(a,b)
a = a/norm(a);  b = b/norm(b);
q = quatNorm([1+dot(a,b); cross(a,b)]);
end

function c = tiltCos(Q)
c = zeros(1,size(Q,2));
for i = 1:size(Q,2)
    R = quat2R(Q(:,i));  c(i) = R(3,1);
end
end

% =========================================================================
function makePlots(L,P,out)
t = L.t;  X = L.X;  d = 180/pi;
tilt = acos(max(-1,min(1,tiltCos(X(7:10,:)))))*d;
tiltE = acos(max(-1,min(1,tiltCos(L.qE))))*d;

figure('Name','Trajectory');
plot3(X(1,:),X(2,:),X(3,:),'LineWidth',1.5); hold on;
plot3(P.target(1),P.target(2),0,'rx','MarkerSize',10,'LineWidth',2);
grid on; axis equal; xlabel('East x (m)'); ylabel('North y (m)'); zlabel('Altitude (m)');
title(sprintf('6-DOF recovery trajectory (touchdown %.2f m/s)',out.touchSpeed));

figure('Name','Dashboard','Position',[100 100 1250 800]);
subplot(3,3,1); plot(t,X(3,:),t,L.pE(3,:),'--','LineWidth',1.2); grid on;
xlabel('t (s)'); ylabel('Altitude (m)'); legend('true','estimated'); title('Altitude');
subplot(3,3,2); plot(X(1,:),X(2,:),'LineWidth',1.3); hold on;
plot(P.target(1),P.target(2),'rx','LineWidth',2); grid on; axis equal;
xlabel('x (m)'); ylabel('y (m)'); title('Ground track');
subplot(3,3,3); plot(t,X(4,:),t,X(5,:),t,X(6,:),'LineWidth',1.2); grid on;
xlabel('t (s)'); ylabel('Velocity (m/s)'); legend('vx','vy','vz'); title('Velocity');
subplot(3,3,4); plot(t,L.cmd(1,:),t,X(17,:),'LineWidth',1.2); grid on;
xlabel('t (s)'); ylabel('Thrust (N)'); legend('command','actual'); title('Thrust');
subplot(3,3,5); plot(t,L.cmd(2,:)*d,t,X(15,:)*d,'LineWidth',1.2); grid on;
xlabel('t (s)'); ylabel('Pitch TVC (deg)'); legend('command','actual'); title('Pitch gimbal');
subplot(3,3,6); plot(t,L.cmd(3,:)*d,t,X(16,:)*d,'LineWidth',1.2); grid on;
xlabel('t (s)'); ylabel('Yaw TVC (deg)'); legend('command','actual'); title('Yaw gimbal');
subplot(3,3,7); plot(t,tilt,t,tiltE,'--','LineWidth',1.2); grid on;
xlabel('t (s)'); ylabel('Tilt from vertical (deg)'); legend('true','estimated'); title('Attitude');
subplot(3,3,8); plot(t,L.eB(2,:)*d,t,L.eB(3,:)*d,'LineWidth',1.2); grid on;
xlabel('t (s)'); ylabel('Error (deg)'); legend('pitch','yaw'); title('Quaternion attitude error');
subplot(3,3,9); plot(t,X(14,:),'LineWidth',1.3); grid on;
xlabel('t (s)'); ylabel('Mass (kg)'); title('Propellant burn');

cg = zeros(1,numel(t)); Iyy = cg;
for i = 1:numel(t), [cg(i),I] = vehProps(X(14,i),P); Iyy(i) = I(2,2); end
figure('Name','Mass properties and estimator');
subplot(1,3,1); plot(t,cg,'LineWidth',1.4); grid on; xlabel('t (s)'); ylabel('CG from nose (m)'); title('CG');
subplot(1,3,2); plot(t,Iyy,'LineWidth',1.4); grid on; xlabel('t (s)'); ylabel('I_{yy} (kg m^2)'); title('Inertia');
subplot(1,3,3); plot(t,vecnormCols(L.pE-X(1:3,:)),'LineWidth',1.2); grid on;
xlabel('t (s)'); ylabel('Position error (m)'); title('Estimator position error');
end

function n = vecnormCols(A)
n = sqrt(sum(A.^2,1));
end
