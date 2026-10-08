clear; clc; close all;

p.m0 = 0.422;
p.mDry = 0.320;
p.g = 9.80665;
p.Tmax = 18.0;
p.Isp = 70;
p.g0 = 9.80665;
p.L = 0.24;
p.Iyy = 0.018;
p.tauTVC = 0.08;
p.deltaMax = deg2rad(8);

dt = 0.01;
tEnd = 65;
t = 0:dt:tEnd;

s = [0; 0; 0; 0; 0; 0; 0; p.m0];

N = numel(t);
X = zeros(8,N);
U = zeros(3,N);

for k = 1:N
    tk = t(k);

    x = s(1);
    z = s(2);
    vx = s(3);
    vz = s(4);
    theta = s(5);
    q = s(6);
    delta = s(7);
    m = s(8);

    if m <= p.mDry
        T = 0;
    elseif tk <= 3.45
        T = p.Tmax;
    elseif tk >= 48 && tk < 50.2
        T = 0.72*p.Tmax;
    elseif tk >= 50.2 && tk < 53.0
        T = 0.90*p.Tmax;
    else
        T = 0;
    end

    if tk < 48
        zCmd = 5;
    elseif tk < 50.2
        zCmd = 28;
    else
        zCmd = 5;
    end

    vzCmd = 0.9*(zCmd-z);

    if tk < 48
        vzCmd = min(vzCmd,8);
    else
        vzCmd = min(vzCmd,3);
    end

    azCmd = 2.2*(vzCmd-vz);
    thetaCmd = -0.08*vx - 0.02*x;
    thetaError = atan2(sin(thetaCmd-theta),cos(thetaCmd-theta));
    qCmd = 3.0*thetaError;
    qError = qCmd-q;
    deltaCmd = 0.35*thetaError + 0.08*qError;
    deltaCmd = max(-p.deltaMax,min(p.deltaMax,deltaCmd));
    deltaDot = (deltaCmd-delta)/p.tauTVC;

    alpha = theta + delta;
    Tx = T*sin(alpha);
    Tz = T*cos(alpha);

    ax = Tx/m;
    az = Tz/m-p.g;
    qDot = T*sin(delta)*p.L/p.Iyy;

    if T > 0 && m > p.mDry
        mdot = -T/(p.Isp*p.g0);
    else
        mdot = 0;
    end

    if m + mdot*dt < p.mDry
        mdot = (p.mDry-m)/dt;
    end

    sDot = [vx; vz; ax; az; q; qDot; deltaDot; mdot];
    s = s + dt*sDot;

    if s(2) < 0
        s(2) = 0;
        s(4) = 0;
    end

    X(:,k) = s;
    U(:,k) = [T; rad2deg(delta); zCmd];
end

figure;
plot(t,X(2,:),'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Mini Rocket Altitude');

figure;
plot(t,U(1,:),'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('Thrust (N)');
title('Thrust Profile');

figure;
plot(t,U(2,:),'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('TVC Angle (deg)');
title('TVC Command');

figure;
plot(t,X(4,:),'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('Vertical Velocity (m/s)');
title('Vertical Velocity');

figure;
plot(X(1,:),X(2,:),'LineWidth',1.5);
grid on;
xlabel('Horizontal Position (m)');
ylabel('Altitude (m)');
title('Rocket Trajectory');

figure;
plot(t,rad2deg(X(5,:)),'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('Pitch Angle (deg)');
title('Rocket Attitude');

disp('Mini Rocket simulation complete.');
