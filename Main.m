clc
clear

% Lug geometry
A = 10;
Cd = 0.2;
mass = 0.015;

% constants
t_start = 0;
t_final = 1000;
g = 9.81;
rho = 1.225;

% Initial pos
X_release = 3;
Y_release = 20;
Z_release = 5;
pos_I = [X_release; Y_release; Z_release];

% Initial vel
U_release = 2;
V_release = -1;
W_release = 2;
vel_I = [U_release; V_release; W_release];

% state
state_I = [pos_I; vel_I];

% motion equations
f = @(t,state) Motion_equations(t,state,g,m,Cd,A);

% options
options = odeset('Events', @groundImpact);

% ode solved
[t, state_solution, te, state_e, ie] = ode45(@f, [t_start t_final], state_I, options);

% safety radius: launchpad position is [0; 0; 0]
x_e = state_e(1);
z_e = state_e(3);
speed = vecnorm(state_solution(4:6));

% impact
safety_radius = sqrt((x_e ^ 2) + (z_e ^ 2));
impact_speed = sqrt((state(4) ^ 2) + (state(5) ^ 2) + (state(6) ^ 2));
impact_KE = 0.5 * m * (impact_speed ^ 2);

print(safety_radius)
print(impact_speed)
print(impact_KE)

figure
plot3d(state(1), state(2), state(3))
title("Lug Trajectory")
xlabel("x / m")
ylabel("y / m")
zlabel("z / m")

figure
plot(state(2), speed)
title("Speed against Height")
xlabel("Height / m")
ylabel("Speed / (m/s)")


