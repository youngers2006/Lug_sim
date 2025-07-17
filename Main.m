clc
clear

% Lug geometry - all need to be checked and changed
A = 0.01;
Cd = 0.2;
m = 0.015;

% constants
t_start = 0;
t_final = 1000;
g = 9.81;
rho = 1.225;

% Initial pos
X_release = 3;
Y_release = 4;
Z_release = 20;
pos_I = [X_release; Y_release; Z_release];

% Initial vel: correct this
U_release = 2;
V_release = 3;
W_release = -4;
vel_I = [U_release; V_release; W_release];

% state
state_I = [pos_I; vel_I];

% motion equations
f = @(t,state) Motion_equations(t,state,g,rho,m,Cd,A);

% options
options = odeset('Events', @groundImpact);

% ode solved
[t, state_solution, te, state_e, ie] = ode45(f, [t_start t_final], state_I, options);

% safety radius: launchpad position is [0; 0; 0]
x_e = state_e(1);
z_e = state_e(3);

num = length(state_solution(:,1));
speed = zeros(length(state_solution(:,1)),1);

for i = 1:num
    speed(i) = sqrt((state_solution(i,4) ^ 2) + (state_solution(i,5) ^ 2) + (state_solution(i,6) ^ 2));
end 
   
% impact
safety_radius = sqrt((x_e ^ 2) + (z_e ^ 2));
impact_speed = sqrt((state_e(4) ^ 2) + (state_e(5) ^ 2) + (state_e(6) ^ 2));
impact_KE = 0.5 * m * (impact_speed ^ 2);

fprintf('The safety radius from the launchpad is %f m. ', safety_radius)
fprintf('The impact speed is %f m/s. ', impact_speed)
fprintf('The impact KE is %f j. ', impact_KE)

figure
plot3(state_solution(:,1), state_solution(:,2), state_solution(:,3))
hold on
title("Lug Trajectory")
xlabel("x / m")
ylabel("y / m")
zlabel("z / m")
xlim([0 state_e(1)])
ylim([0 state_e(2)])
zlim([0 state_solution(1,3)])
text(state_e(1),state_e(2),state_e(3), sprintf('Impact Point: (%.2f, %.2f, %.2f)',state_e(1),state_e(2),state_e(3)))
plot3(state_e(1),state_e(2),state_e(3),'ro', 'MarkerSize', 5, 'LineWidth', 2)
text(0,0,0, sprintf('Launch Pad: (%.2f, %.2f, %.2f)',0,0,0))
plot3(0,0,0,'ro', 'MarkerSize', 5, 'LineWidth', 2)
text(state_solution(1,1),state_solution(1,2),state_solution(1,3),sprintf('Release Point: (%.2f, %.2f, %.2f)',state_solution(1,1),state_solution(1,2),state_solution(1,3)))
plot3(state_solution(1,1),state_solution(1,2),state_solution(1,3),'ro', 'MarkerSize', 5, 'LineWidth', 2)
hold off

figure
plot(state_solution(:,3), speed)
title("Speed against Height")
xlabel("Height / m")
ylabel("Speed / (m/s)")

figure
plot(t,state_solution(:,3))
title("Height against Time")
ylabel("Height / m")
xlabel("Time / s")
